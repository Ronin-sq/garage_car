import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_pkg.action import Navigate
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs

class SequentialActionClient(Node):
    def __init__(self):
        super().__init__('sequential_action_client')
        
        # 1. 初始化状态锁：True 表示空闲，可以接收新任务
        self.is_ready = True 
        
        self._action_client = ActionClient(self, Navigate, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅路径点话题
        self.sub = self.create_subscription(Marker, 'path_points', self.call_back, 10)
        self.flag = 1
        self.get_logger().info('Ready to receive the first goal...')

    def call_back(self, msg):
        # --- 关键逻辑 1: 检查门锁 ---
        if not self.is_ready:
            # 如果机器人正在移动，直接无视这个消息
            return

        if not msg.points:
            return

        # --- 关键逻辑 2: 一旦进入处理流程，立刻关门 ---
        self.is_ready = False
        self.get_logger().info('New point received. Locking subscription...')

        target_point_msg = msg.points[self.flag]
        target_next_point_msg = msg.points[self.flag+1]
        
        # 坐标转换逻辑 (同前)
        odom_point = self.transform_laser_to_odom(
            target_point_msg.x, target_point_msg.y, target_point_msg.z, msg.header.frame_id,msg.header.stamp)
        target_next_point = self.transform_laser_to_odom(
            target_next_point_msg.x, target_next_point_msg.y, target_next_point_msg.z, msg.header.frame_id,msg.header.stamp)

        if odom_point:
            self.send_goal(odom_point.x, odom_point.y, target_next_point.y, self.flag)
            self.get_logger().info(f'Sent goal to {odom_point.x}, {odom_point.y}')
        else:
            # 如果转换失败，记得把门打开，否则节点就永远锁死了
            self.is_ready = True
            self.get_logger().warn('Transform failed, ready for next point again.')

    def send_goal(self, x, y, next_y, flag):
        goal_msg = Navigate.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.next_y = next_y
        goal_msg.flag = self.flag

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.is_ready = True # 被拒绝了也要开门
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Action 彻底结束（成功、失败或取消）
        result = future.result().result
        self.get_logger().info(f'Action Finished. Success: {result.success}')
        
        # --- 关键逻辑 3: 任务完成，重新开门 ---
        self.is_ready = True
        if result.success:
            self.flag = self.flag + 1
        self.get_logger().info('--- READY FOR NEXT GOAL ---')

    def feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
    def transform_laser_to_odom(self, x, y, z, source_frame, stamp):
        point_laser = PointStamped()
        # 【关键修复】使用 Time() 获取最新转换，避免时间戳超前问题
        point_laser.header.stamp = rclpy.time.Time().to_msg()
        point_laser.header.frame_id = source_frame
        point_laser.point.x = x
        point_laser.point.y = y
        point_laser.point.z = z

        try:
            # 转换到 odom 坐标系
            point_odom = self.tf_buffer.transform(point_laser, 'odom', timeout=rclpy.duration.Duration(seconds=0.5))
            return point_odom.point
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform {source_frame} to odom: {ex}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = SequentialActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
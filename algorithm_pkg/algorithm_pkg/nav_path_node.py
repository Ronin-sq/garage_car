import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class PathVisualizationNode(Node):
    def __init__(self):
        super().__init__("path_visualization_node")

        # 订阅路径点话题
        self.path_point_sub = self.create_subscription(
            Marker, "path_points", self.callback, 10)

        # 发布可视化路径
        self.vis_publisher = self.create_publisher(Path, '/global_plan_visual', 10)

        # TF转换
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Path visualization node started')

    def callback(self, msg):
        """将Marker中的路径点转换为Path消息并发布可视化"""
        if len(msg.points) == 0:
            return

        original_frame_id = msg.header.frame_id
        path_points = msg.points

        # 创建Path消息
        ros_path = Path()
        ros_path.header.frame_id = 'base_link'
        ros_path.header.stamp = self.get_clock().now().to_msg()

        current_ros_time = ros_path.header.stamp

        # 根据原始坐标系处理
        if original_frame_id == 'odom' or original_frame_id == 'base_link':
            # 直接使用该坐标系的点
            for i, pt in enumerate(path_points):
                pose = PoseStamped()
                pose.header.frame_id = 'base_link'
                pose.header.stamp = current_ros_time

                pose.pose.position.x = float(pt.x)
                pose.pose.position.y = float(pt.y)
                pose.pose.position.z = 0.0

                # 计算朝向（根据下一个点的方向）
                if i < len(path_points) - 1:
                    yaw = (path_points[i+1].y - pt.y) / (path_points[i+1].x - pt.x + 1e-6)
                    yaw = 0  # 简化处理
                    pose.pose.orientation.z = 0.0
                    pose.pose.orientation.w = 1.0
                else:
                    pose.pose.orientation.w = 1.0

                ros_path.poses.append(pose)
        else:
            # 需要进行TF转换到base_link坐标系
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link', original_frame_id, rclpy.time.Time())

                for i, pt in enumerate(path_points):
                    local_pose = PoseStamped()
                    local_pose.header.frame_id = original_frame_id
                    local_pose.header.stamp = current_ros_time

                    local_pose.pose.position.x = float(pt.x)
                    local_pose.pose.position.y = float(pt.y)
                    local_pose.pose.position.z = 0.0
                    local_pose.pose.orientation.w = 1.0

                    # TF变换到odom坐标系
                    global_pose = tf2_geometry_msgs.do_transform_pose(
                        local_pose.pose, transform)

                    pose_stamped_global = PoseStamped()
                    pose_stamped_global.header.frame_id = 'base_link'
                    pose_stamped_global.header.stamp = current_ros_time
                    pose_stamped_global.pose = global_pose
                    ros_path.poses.append(pose_stamped_global)

            except Exception as e:
                self.get_logger().warn(f"TF 变换失败: {e}")
                return

        # 发布可视化路径
        self.vis_publisher.publish(ros_path)
        self.get_logger().info(f'发布可视化路径，包含 {len(ros_path.poses)} 个点')


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
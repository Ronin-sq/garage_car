import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class RebarTaskExecutor(Node):
    def __init__(self):
        super().__init__('rebar_task_executor')
        
        # 1. 创建 Action 客户端
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        
        # 2. 订阅你之前生成的“弓字型”路径话题
        self.subscription = self.create_subscription(Path, '/global_plan', self.path_callback, 10)
        
        self.get_logger().info('钢筋楼面全覆盖执行器已启动，等待路径...')

    def path_callback(self, msg):
        if len(msg.poses) == 0:
            return
            
        # self.get_logger().info(f'收到路径，共 {len(msg.poses)} 个点，请求 Nav2 执行...')
        self.send_goal(msg.poses)

    def send_goal(self, poses):
        # 等待服务器响应
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 服务未启动，请检查 navigation_launch.py 是否运行')
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses # 这里就是你 generate_path_points 生成的所有点

        # 发送异步请求
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 拒绝了路径任务（可能是起点被认为在障碍物上）')
            return
        # self.get_logger().info('小车已接受指令，开始全覆盖作业！')

def main(args=None):
    rclpy.init(args=args)
    executor = RebarTaskExecutor()
    rclpy.spin(executor)
    executor.destroy_node()
    rclpy.shutdown()
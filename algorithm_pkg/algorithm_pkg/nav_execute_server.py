import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class ProgressActionServer(Node):

    def __init__(self):
        super().__init__('progress_action_server')
        # 3-1.创建动作服务端；
        self._action_server = ActionServer(
            self,
            NavigateThroughPoses,
            'navigate_through_poses',
            self.execute_callback)
        self.get_logger().info('动作服务已经启动！')

    def execute_callback(self, goal_handle):
        self.get_logger().info('开始执行任务....')


        # 3-2.生成连续反馈；
        feedback_msg = Progress.Feedback()

        sum = 0
        for i in range(1, goal_handle.request.num + 1):
            sum += i
            feedback_msg.progress = i / goal_handle.request.num
            self.get_logger().info('连续反馈: %.2f' % feedback_msg.progress)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        # 3-3.生成最终响应。
        goal_handle.succeed()
        result = Progress.Result()
        result.sum = sum
        self.get_logger().info('任务完成！')

        return result

    def execute_task(self):
        "执行点到点的任务"
        
        
        pass


def main(args=None):

    # 2.初始化 ROS2 客户端；
    rclpy.init(args=args)

    # 4.调用spin函数，并传入节点对象；
    Progress_action_server = ProgressActionServer()
    rclpy.spin(Progress_action_server)

    # 5.释放资源。
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
import math
import time
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_pkg.action import Navigate 
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor # 必须导入

class PDActionServer(Node):
    def __init__(self):
        super().__init__('pd_action_server')
        self.group = ReentrantCallbackGroup()
        # 1. 速度发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10, callback_group=self.group)
        
        # 2. 里程计订阅者 (使用 EKF 融合后的里程计)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10, callback_group=self.group)
        
        # 3. Action Server
        self._action_server = ActionServer(
            self, Navigate, 'navigate_to_pose', self.execute_callback)

        # 内部状态
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        
        # PD 参数
        self.kp_linear = 0.5
        self.kd_linear = 0.1
        self.kp_angular = 2.0
        self.prev_dist_error = 0.0

        self.get_logger().info('PD Action Server has been started.')

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        
        # 四元数转偏航角(Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.curr_yaw = math.atan2(siny_cosp, cosy_cosp)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Navigate.Feedback()
        
        target_x = goal_handle.request.x
        target_y = goal_handle.request.y
        next_y = goal_handle.request.next_y
        flag = goal_handle.request.flag
        
        y_error = next_y - target_y

        # 【新增】初始化误差值为初始距离，避免首次微分项突变
        initial_dx = target_x - self.curr_x
        initial_dy = target_y - self.curr_y
        self.prev_dist_error = math.sqrt(initial_dx**2 + initial_dy**2)
        
        rate = self.create_rate(10) # 10Hz
        
        while rclpy.ok():
            # 【关键修复】检查是否有取消请求
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancel requested, aborting...')
                goal_handle.canceled()  # 设置状态为已取消
                return Navigate.Result(success=False) # 返回结果对象
            
            # 1. 计算误差
            dx = target_x - self.curr_x
            dy = target_y - self.curr_y
            self.get_logger().info(f'current position: ({self.curr_x:.2f}, {self.curr_y:.2f}), target: ({target_x:.2f}, {target_y:.2f}), error: ({dx:.2f}, {dy:.2f})')
            # distance_error = math.sqrt(dx**2 + dy**2)
            distance_error = math.sqrt(dx**2) # 避免除零错误
            
            # 2. 如果到达目标点
            if distance_error < 0.1:
                cmd = Twist() # 停止机器人
                self.cmd_vel_pub.publish(cmd)
                # 到达目标后，只有需要跨行（y_error较大）时才执行旋转和换行宏动作
                if abs(y_error) > 0.01:
                    self.get_logger().info('Goal reached, now stabilizing before rotation...')
                    time.sleep(0.5)  # 停稳半秒钟，让因为急停造成的底盘晃动消除
                    self.rotate_90_degrees(flag, y_error)
                else:
                    self.get_logger().info('Goal reached (intermediate point), skipping rotation.')
                
                self.get_logger().info('Rotation completed. Stabilizing...')
                time.sleep(5.0)  # 旋转结束后也停稳半秒
                break
            
            # 3. PD 控制线速度 (Linear)
            derivative = distance_error - self.prev_dist_error
            v = self.kp_linear * distance_error + self.kd_linear * derivative
            self.prev_dist_error = distance_error
            
            # 4. 角度控制 (简单 P 控制让机器人面向目标)
            target_yaw = math.atan2(dy, dx)
            yaw_error = target_yaw - self.curr_yaw
            # 角度归一化到 [-pi, pi]
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            w = self.kp_angular * yaw_error
            
            # 5. 发布速度
            cmd = Twist()
            cmd.linear.x = min(v, 0.05) # 限制最大线速度
            cmd.angular.z = 0.0 # 启用转向控制，限制最大角速度
            self.cmd_vel_pub.publish(cmd)
            
            # 6. 发布反馈
            feedback_msg.distance_remaining = distance_error
            goal_handle.publish_feedback(feedback_msg)
            # self.get_logger().info(f'1')
            rate.sleep()
            # self.get_logger().info(f'2')
        self.get_logger().info('Goal reached.')
        # 完成任务
        # cmd = Twist() # 停止机器人
        # self.cmd_vel_pub.publish(cmd)
        
        goal_handle.succeed()
        result = Navigate.Result()
        result.success = True
        return result
    
    def rotate_90_degrees(self, flag, y_error):
        """机器人原地旋转90度（使用闭环反馈控制）"""
        target_rotation = math.pi / 2  # 90度 = π/2 弧度
        
        # 记录初始偏航角
        initial_yaw = self.curr_yaw
        
        # 确定旋转方向
        # 由于 client 的 flag 会对所有路径点递增（包含中间点），我们通过 (flag + 1)//2 计算实际的换行次数
        line_index = (flag + 1) // 2
        if line_index % 2 == 0:  # 偶数次换行，向左转
            direction = 1  # 正向
        else:  # 奇数次换行，向右转
            direction = -1  # 反向
        
        # P控制器参数
        kp = 1.5
        max_angular_vel = 0.5  # 最大角速度 rad/s (提高以加快旋转)
        min_angular_vel = 0.1  # 保证能克服静摩擦力且平滑的最小角速度 (提高以防止卡死)
        angle_threshold = 0.02  # 停止阈值 (略微放宽到约1度)
        
        # 计算目标最终角度
        target_yaw = initial_yaw + direction * target_rotation
        
        # 归一化目标角度到 [-pi, pi]
        target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))
        
        self.get_logger().info(f'Starting rotation: initial_yaw={math.degrees(initial_yaw):.1f}°, target_yaw={math.degrees(target_yaw):.1f}°')
        
        rate = self.create_rate(20)  # 提高控制频率到20Hz
        
        while rclpy.ok():
            # 计算当前角度误差
            current_yaw = self.curr_yaw
            remaining_angle = target_yaw - current_yaw
            
            # 归一化角度误差到 [-pi, pi]
            remaining_angle = math.atan2(math.sin(remaining_angle), math.cos(remaining_angle))
            
            # 检查是否到达目标角度
            if abs(remaining_angle) < angle_threshold:
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
                self.get_logger().info(f'Rotation completed: current_yaw={math.degrees(current_yaw):.1f}°')
                break
            
            # P控制计算角速度
            angular_vel = kp * remaining_angle
            
            # 限制最大角速度与最小角速度
            if angular_vel > 0:
                angular_vel = max(min_angular_vel, min(max_angular_vel, angular_vel))
            else:
                angular_vel = min(-min_angular_vel, max(-max_angular_vel, angular_vel))
            
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = angular_vel
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info(f'Rotating... error={math.degrees(remaining_angle):.1f}°, vel={angular_vel:.2f}')
            rate.sleep()
        
        # 停止机器人
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

        # ===== Y方向移动 =====
        # 使用反馈控制移动Y方向
        y_error = -0.1
        if abs(y_error) > 0.01:  # 只有当有Y方向移动时才执行
            move_speed = 0.15
            target_y = self.curr_y + y_error
            distance_threshold = 0.01
            
            self.get_logger().info(f'Starting Y movement: current_y={self.curr_y:.3f}, target_y={target_y:.3f}, distance={abs(y_error):.3f}')
            
            rate = self.create_rate(20)
            
            while rclpy.ok():
                current_y = self.curr_y
                remaining_y = target_y - current_y
                
                if abs(remaining_y) < distance_threshold:
                    cmd = Twist()
                    self.cmd_vel_pub.publish(cmd)
                    self.get_logger().info(f'Y movement completed: current_y={current_y:.3f}')
                    break
                
                # P控制
                cmd = Twist()
                # 根据当前朝向和剩余的Y距离决定是前进还是后退
                direction_multiplier = 1.0 if (remaining_y * math.sin(self.curr_yaw)) >= 0 else -1.0
                cmd.linear.x = move_speed * direction_multiplier
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                self.get_logger().info(f'Moving Y... remaining={remaining_y:.3f}')
                rate.sleep()
            
            # 停止
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)

        # ===== 第三次旋转（方向相反转回来，使用闭环反馈）=====
        if abs(y_error) > 0.01:  # 只有执行了Y方向移动才需要第三次旋转
            target_rotation = math.pi / 2
            initial_yaw_after_y = self.curr_yaw
            
            reverse_direction = direction  # 与第一次旋转方向相同
            
            target_yaw_final = initial_yaw_after_y + reverse_direction * target_rotation
            target_yaw_final = math.atan2(math.sin(target_yaw_final), math.cos(target_yaw_final))
            
            self.get_logger().info(f'Starting second rotation: initial_yaw={math.degrees(initial_yaw_after_y):.1f}°, target_yaw={math.degrees(target_yaw_final):.1f}°')
            
            rate = self.create_rate(20)
            
            while rclpy.ok():
                current_yaw = self.curr_yaw
                remaining_angle = target_yaw_final - current_yaw
                remaining_angle = math.atan2(math.sin(remaining_angle), math.cos(remaining_angle))
                
                if abs(remaining_angle) < angle_threshold:
                    cmd = Twist()
                    self.cmd_vel_pub.publish(cmd)
                    self.get_logger().info(f'second rotation completed: current_yaw={math.degrees(current_yaw):.1f}°')
                    break
                
                angular_vel = kp * remaining_angle
                if angular_vel > 0:
                    angular_vel = max(min_angular_vel, min(max_angular_vel, angular_vel))
                else:
                    angular_vel = min(-min_angular_vel, max(-max_angular_vel, angular_vel))
                
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = angular_vel
                self.cmd_vel_pub.publish(cmd)
                self.get_logger().info(f'second rotation... error={math.degrees(remaining_angle):.1f}°, vel={angular_vel:.2f}')
                rate.sleep()
            
            # 停止
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PDActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
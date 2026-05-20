#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import matplotlib.pyplot as plt
import numpy as np
import math

class ErrorAnalysisPlotter(Node):
    def __init__(self):
        super().__init__('error_analysis_plotter')
        
        # 存储最新的 Ground Truth
        self.current_gt_x = None
        self.current_gt_y = None
        self.current_gt_yaw = None
        
        # 存储时间序列和误差
        self.start_time = None
        
        self.odom_times = []
        self.odom_pos_errors = []
        self.odom_yaw_errors = []
        
        self.ekf_times = []
        self.ekf_pos_errors = []
        self.ekf_yaw_errors = []

        # 订阅者 (使用 sensor_data QoS 以兼容 Gazebo)
        self.create_subscription(Odometry, '/odom_ground_truth', self.gt_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_callback, qos_profile_sensor_data)

        # 定时器用于实时绘图
        self.timer = self.create_timer(1.0, self.plot_callback)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def gt_callback(self, msg):
        self.current_gt_x = msg.pose.pose.position.x
        self.current_gt_y = msg.pose.pose.position.y
        self.current_gt_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

    def odom_callback(self, msg):
        if self.current_gt_x is None:
            return
            
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = current_time
            
        t = current_time - self.start_time
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        
        pos_err = math.sqrt((x - self.current_gt_x)**2 + (y - self.current_gt_y)**2)
        
        yaw_err = abs(yaw - self.current_gt_yaw)
        # 角度归一化到 [0, pi]
        if yaw_err > math.pi:
            yaw_err = 2 * math.pi - yaw_err
            
        self.odom_times.append(t)
        self.odom_pos_errors.append(pos_err)
        self.odom_yaw_errors.append(math.degrees(yaw_err))

    def ekf_callback(self, msg):
        if self.current_gt_x is None:
            return
            
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = current_time
            
        t = current_time - self.start_time
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        
        pos_err = math.sqrt((x - self.current_gt_x)**2 + (y - self.current_gt_y)**2)
        
        yaw_err = abs(yaw - self.current_gt_yaw)
        if yaw_err > math.pi:
            yaw_err = 2 * math.pi - yaw_err
            
        self.ekf_times.append(t)
        self.ekf_pos_errors.append(pos_err)
        self.ekf_yaw_errors.append(math.degrees(yaw_err))

    def plot_callback(self):
        plt.ion()
        plt.clf()
        
        # 字体设置
        plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS']
        plt.rcParams['axes.unicode_minus'] = False

        # 创建上下两个子图：上面是位置误差，下面是航向误差
        plt.subplot(2, 1, 1)
        lines_plotted = False
        if len(self.odom_times) > 0:
            plt.plot(self.odom_times, self.odom_pos_errors, 'r-', label='Raw Odometry', linewidth=1.5)
            lines_plotted = True
        if len(self.ekf_times) > 0:
            plt.plot(self.ekf_times, self.ekf_pos_errors, 'b-', label='EKF Fused', linewidth=1.5)
            lines_plotted = True
            
        plt.title('Localization Error Analysis on Rebar Surface')
        plt.ylabel('Absolute Position Error (m)')
        if lines_plotted:
            plt.legend()
        plt.grid(True)

        plt.subplot(2, 1, 2)
        lines_plotted_yaw = False
        if len(self.odom_times) > 0:
            plt.plot(self.odom_times, self.odom_yaw_errors, 'r-', label='Raw Odometry', linewidth=1.5)
            lines_plotted_yaw = True
        if len(self.ekf_times) > 0:
            plt.plot(self.ekf_times, self.ekf_yaw_errors, 'b-', label='EKF Fused', linewidth=1.5)
            lines_plotted_yaw = True
            
        plt.xlabel('Running Time (s)')
        plt.ylabel('Heading Error (Degree)')
        if lines_plotted_yaw:
            plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = ErrorAnalysisPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.show()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        
        self.ground_truth_x = []
        self.ground_truth_y = []
        
        self.odom_x = []
        self.odom_y = []
        
        self.ekf_x = []
        self.ekf_y = []

        self.create_subscription(Odometry, '/odom_ground_truth', self.gt_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_callback, qos_profile_sensor_data)

        self.timer = self.create_timer(1.0, self.plot_callback)

    def gt_callback(self, msg):
        self.ground_truth_x.append(msg.pose.pose.position.x)
        self.ground_truth_y.append(msg.pose.pose.position.y)

    def odom_callback(self, msg):
        self.odom_x.append(msg.pose.pose.position.x)
        self.odom_y.append(msg.pose.pose.position.y)
        self.get_logger().info('Received /odom', once=True)

    def ekf_callback(self, msg):
        self.ekf_x.append(msg.pose.pose.position.x)
        self.ekf_y.append(msg.pose.pose.position.y)
        self.get_logger().info('Received /odometry/filtered', once=True)
        
    def gt_callback(self, msg):
        self.ground_truth_x.append(msg.pose.pose.position.x)
        self.ground_truth_y.append(msg.pose.pose.position.y)
        self.get_logger().info('Received /odom_ground_truth', once=True)
    def plot_callback(self):
        plt.ion()
        plt.clf()
        
        # 设置支持中文的字体，以适应用户的需求
        plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS']
        plt.rcParams['axes.unicode_minus'] = False

        lines_plotted = False
        if len(self.ground_truth_x) > 0:
            plt.plot(self.ground_truth_x, self.ground_truth_y, 'g-', label='Line A (Ground Truth)')
            lines_plotted = True
            
        if len(self.odom_x) > 0:
            plt.plot(self.odom_x, self.odom_y, 'r--', label='Line B (Raw Odometry)')
            lines_plotted = True
            
        if len(self.ekf_x) > 0:
            plt.plot(self.ekf_x, self.ekf_y, 'b-.', label='Line C (EKF Fused)')
            lines_plotted = True

        plt.title('Trajectory Comparison on Rebar Surface')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        if lines_plotted:
            plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.show() # keep the final plot open
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

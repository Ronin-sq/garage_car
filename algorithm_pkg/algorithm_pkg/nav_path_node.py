import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
import numpy as np
import math
import time

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class UltimateNavNode(Node):
    def __init__(self):
        super().__init__("ultimate_nav_node")
        
        self.path_point_sub = self.create_subscription(Marker, "path_points", self.call_back, 10)
        self.vis_publisher = self.create_publisher(Path, '/global_plan_visual', 10) 
        
        self.action_client = ActionClient(self, FollowPath, 'follow_path')
        self.get_logger().info("等待 /follow_path Action Server 启动...")
        self.action_client.wait_for_server()
        self.get_logger().info("连接成功！")
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.last_send_time = 0.0

    # ================= 新增：路径稠密化（插值）函数 =================
    def interpolate_points(self, raw_points, resolution=0.05):
        """
        将稀疏的点进行线性插值，变为密集的轨迹点。
        resolution: 插值分辨率，0.05表示每 5cm 一个点。
        """
        if len(raw_points) < 2:
            return raw_points
            
        dense_points = []
        for i in range(len(raw_points) - 1):
            p1 = raw_points[i]
            p2 = raw_points[i+1]
            
            dist = math.hypot(p2.x - p1.x, p2.y - p1.y)
            # 计算这一段需要插入多少个点
            num_samples = max(int(dist / resolution), 1)
            
            for j in range(num_samples):
                # 简单创建一个带 x, y 属性的临时对象来存储插值结果
                pt = type('Point', (), {})()
                pt.x = p1.x + (p2.x - p1.x) * (j / float(num_samples))
                pt.y = p1.y + (p2.y - p1.y) * (j / float(num_samples))
                dense_points.append(pt)
                
        # 别忘了加上最后一个端点
        dense_points.append(raw_points[-1])
        return dense_points
    # ==========================================================

    def call_back(self, msg):
        current_time = time.time()
        
        if current_time - self.last_send_time < 1.0:
            return
            
        original_frame_id = msg.header.frame_id
        if len(msg.points) == 0:
            return

        # 1. 对稀疏的 Marker 点进行插值（每 5cm 产生一个点）
        path_points = self.interpolate_points(msg.points, resolution=0.05)

        ros_path = Path()
        ros_path.header.frame_id = 'odom'
        
        # 获取统一的当前时间戳
        current_ros_time = self.get_clock().now().to_msg()
        ros_path.header.stamp = current_ros_time

        # ---------- TF 转换与构建 Pose ----------
        if original_frame_id == 'odom':
            for i, pt in enumerate(path_points):
                pose = PoseStamped()
                pose.header.frame_id = 'odom'
                pose.header.stamp = current_ros_time # 【关键修复】：给内部的Pose也打上时间戳
                
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(pt.x), float(pt.y), 0.0
                if i < len(path_points) - 1:
                    yaw = np.arctan2(path_points[i+1].y - pt.y, path_points[i+1].x - pt.x)
                    pose.pose.orientation.z, pose.pose.orientation.w = float(np.sin(yaw/2.0)), float(np.cos(yaw/2.0))
                else:
                    # 最后一个点继承前一个点的朝向
                    if len(ros_path.poses) > 0:
                        pose.pose.orientation = ros_path.poses[-1].pose.orientation
                    else:
                        pose.pose.orientation.w = 1.0
                ros_path.poses.append(pose)
        else:
            try:
                transform = self.tf_buffer.lookup_transform('odom', original_frame_id, rclpy.time.Time())
                for i, pt in enumerate(path_points):
                    local_pose = PoseStamped()
                    local_pose.header.frame_id = original_frame_id
                    local_pose.header.stamp = current_ros_time
                    
                    local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z = float(pt.x), float(pt.y), 0.0
                    if i < len(path_points) - 1:
                        yaw = np.arctan2(path_points[i+1].y - pt.y, path_points[i+1].x - pt.x)
                        local_pose.pose.orientation.z, local_pose.pose.orientation.w = float(np.sin(yaw/2.0)), float(np.cos(yaw/2.0))
                    else:
                        local_pose.pose.orientation.w = 1.0
                        
                    global_pose = tf2_geometry_msgs.do_transform_pose(local_pose.pose, transform)
                    pose_stamped_global = PoseStamped()
                    pose_stamped_global.header.frame_id = 'odom'
                    pose_stamped_global.header.stamp = current_ros_time # 【关键修复】
                    pose_stamped_global.pose = global_pose
                    ros_path.poses.append(pose_stamped_global)
            except Exception as e:
                self.get_logger().warn(f"TF 变换失败: {e}")
                return
        # --------------------------------

        goal_msg = FollowPath.Goal()
        goal_msg.path = ros_path
        goal_msg.controller_id = 'FollowPath'
        goal_msg.goal_checker_id = 'general_goal_checker'

        self.get_logger().info(f"成功将稀疏点插值为 {len(ros_path.poses)} 个密集的轨迹点，并发送给 Controller...")
        self.action_client.send_goal_async(goal_msg)
        self.last_send_time = current_time
        
        self.vis_publisher.publish(ros_path)

def main(args=None):
    rclpy.init(args=args)
    node = UltimateNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
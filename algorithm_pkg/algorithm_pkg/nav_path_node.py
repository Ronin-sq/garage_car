import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
# import tf_transformations
import numpy as np


class NavNode(Node):
    def __init__(self):
        super().__init__("Nav_node")
        self.path_point_sub = self.create_subscription(Marker,"path_points",self.call_back,10)
        self.path_publisher = self.create_publisher(Path,'/global_plan',10)
        print("导航路径节点已启动...")
    def call_back(self,msg):
        path_points = msg.points
        frame_id = msg.header.frame_id
        print(f"path_points: {len(path_points)}")
        ros_path = self._convert(frame_id, path_points)
        # print(f"path_points: {len(path_points)}")
        self.path_publisher.publish(ros_path)

    def _convert(self, frame_id, path_points):
        ros_path = Path()
        ros_path.header.frame_id = frame_id
        ros_path.header.stamp = self.get_clock().now().to_msg()

        for i, pt in enumerate(path_points):
            pose = PoseStamped()
            # print(f"pt: {pt}")
            pose.header.frame_id = frame_id
            pose.pose.position.x = float(pt.x)
            pose.pose.position.y = float(pt.y)
            pose.pose.position.z = 0.0
            
            # 简单的姿态计算：让车头指向下一个点（可选）
            if i < len(path_points) - 1:
                next_pt = path_points[i+1]
                yaw = np.arctan2(next_pt.y - pt.y, next_pt.x - pt.x)
                # q = tf_transformations.quaternion_from_euler(0, 0, yaw)
                ori_z = np.sin(yaw / 2.0)
                ori_w = np.cos(yaw / 2.0)
                # 存在问题
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = float(ori_z)
                pose.pose.orientation.w = float(ori_w)
            
            ros_path.poses.append(pose)
        return ros_path
    
def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


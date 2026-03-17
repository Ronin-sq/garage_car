import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
import numpy as np
import alphashape  # 核心算法库
from sklearn.cluster import DBSCAN  # 聚类算法库
from shapely.geometry import LineString, Polygon
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer



class SparseBorderExtractor(Node):
    def __init__(self):
        super().__init__('border_extractor')
        
        # 订阅雷达数据
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
            
        # 发布边界Marker用于RViz可视化
        self.marker_pub = self.create_publisher(Marker, 'floor_border_marker', 10)
        self.clean_points_pub = self.create_publisher(Marker, 'cleaned_rebar_centers', 10)
        self.path_points_pub = self.create_publisher(Marker, 'path_points', 10)
        # 算法参数
        self.alpha = 0.1  # Alpha值越小越接近凸包，越大越紧贴点云。需根据钢筋间距调试。
        self.get_logger().info("稀疏边界提取节点已启动...")
        # self._action_client = ActionServer(self, NavigateToPose, 'point',self.execute_callback)


        

    def scan_callback(self, msg):
        # 1. 极坐标转直角坐标 (x, y)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        
        # 过滤无效点 (inf, nan 以及超出范围的点)
        valid_mask = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        if not np.any(valid_mask):
            return

        x = ranges[valid_mask] * np.cos(angles[valid_mask])
        y = ranges[valid_mask] * np.sin(angles[valid_mask])
        points = np.column_stack((x, y))

        # 2. 使用 DBSCAN 聚类
        # eps: 两个点被视为同一簇的最大距离（建议设为钢筋直径的1.5倍，如 0.02m - 0.05m）钢筋直径为0.02
        # min_samples: 一个簇至少包含的点数（过滤单点噪声）
        # 当下并没有过滤单点噪声，因为钢筋可能只有一个点被扫描到（sim理想情况）
        clustering = DBSCAN(eps=0.05, min_samples=1).fit(points)
        labels = clustering.labels_

        # 3. 计算每个簇的中心点
        refined_points = []
        unique_labels = set(labels)
        
        for label in unique_labels:
            if label == -1: # 跳过噪声点
                continue
                
            # 提取属于当前簇的所有点
            class_member_mask = (labels == label)
            cluster_points = points[class_member_mask]
            
            # 计算质心 (Mean)
            centroid = cluster_points.mean(axis=0)
            refined_points.append(centroid)

        # 现在 refined_points 里的每个元素就是一根钢筋的中心坐标
        refined_points = np.array(refined_points)
        
        # 4. 后续处理：发布边界或标记
        if len(refined_points) > 0:
            self.publish_cleaned_markers(refined_points, msg.header.frame_id)

        # 2. 提取 Alpha Shape 边界
        try:
            # 计算凹包 (Concave Hull)
            hull = alphashape.alphashape(refined_points, self.alpha)
            # hull = alphashape.alphashape(refined_points)  # alpha值需要根据实际情况调试
            # print(f"边界类型: {hull.geom_type}, 点数: {len(refined_points)}")
            # 提取边界坐标
            if hull.geom_type == 'Polygon':
                border_coords = list(hull.exterior.coords)
                self.publish_marker(border_coords, msg.header.frame_id)
                # path_point_data = self.generate_path(hull, step_size=0.02)  # 生成路径，步距根据机器人宽度设定
                path_point_data = self.generate_path_points(hull, step_size=0.2)

            elif hull.geom_type == 'MultiPolygon':
                # 如果点云太稀疏，可能会产生多个区域
                # for poly in hull.geoms:
                #     self.publish_marker(list(poly.exterior.coords), msg.header.frame_id)
                # 多个区域选择面积最大的一个
                largest_polygon = max(hull.geoms, key=lambda poly: poly.area)
                print(f"找到 {len(hull.geoms)} 个区域，选择面积最大的区域: {largest_polygon.area:.4f}")
                self.publish_marker(list(largest_polygon.exterior.coords), msg.header.frame_id)
                # path_point_data = self.generate_path(largest_polygon, step_size=0.2)
                path_point_data = self.generate_path_points(largest_polygon, step_size=0.2)
            # print(f"生成的路径点:{len(path_point_data)}")
            self.publish_path_points(path_point_data, msg.header.frame_id)
        except Exception as e:
            self.get_logger().warn(f"边界计算失败: {e}")

    def publish_cleaned_markers(self, points, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.ns = "cleaned_points"
        marker.id = 1
        
        # 点样式
        marker.scale.x = 0.1  # 点大小
        marker.scale.y = 0.1
        marker.color.g = 1.0   # 绿色点表示清洗后的中心点
        marker.color.a = 1.0
        
        # 填充点数据
        for p in points:
            point_msg = Point()
            point_msg.x = float(p[0])
            point_msg.y = float(p[1])
            point_msg.z = 0.0
            marker.points.append(point_msg)
            
        self.clean_points_pub.publish(marker)
    def publish_marker(self, coords, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.ns = "border"
        marker.id = 0
        
        # 线条样式
        marker.scale.x = 0.05  # 线宽
        marker.color.r = 1.0   # 红色边界
        marker.color.a = 1.0
        
        # 填充点数据
        for c in coords:
            p = Point()
            p.x = float(c[0])
            p.y = float(c[1])
            p.z = 0.0
            marker.points.append(p)
            
        self.marker_pub.publish(marker)

    # def generate_path(self, polygon, step_size):
    #     """
    #     polygon: alphashape 生成的多边形对象
    #     step_size: 路径间距（根据机器人宽度设定，例如 0.2m）
    #     """
    #     min_x, min_y, max_x, max_y = polygon.bounds
    #     path_points = []
    #     print(f"边界范围: x[{min_x:.2f}, {max_x:.2f}], y[{min_y:.2f}, {max_y:.2f}]")
    #     # 1. 从左向右生成垂直扫描线
    #     current_x = min_x + step_size / 2
    #     direction = 1  # 1 为向上，-1 为向下，实现“弓”字型
        
    #     while current_x < max_x:
    #         # 创建一条极长的垂直线
    #         vertical_line = LineString([(current_x, min_y - 1), (current_x, max_y + 1)])
            
    #         # 2. 计算扫描线与边界多边形的交线
    #         intersection = vertical_line.intersection(polygon)
    #         # print("ronin login : 扫描一次...")
    #         if not intersection.is_empty:

    #             # 提取交线的两个端点坐标
    #             if intersection.geom_type == 'LineString':
    #                 coords = list(intersection.coords)
    #                 # 根据当前方向决定进入顺序
    #                 if direction == -1:
    #                     coords.reverse()
    #                 path_points.extend(coords)
    #                 direction *= -1  # 掉头
                    
    #         current_x += step_size
            
    #     return path_points


    def Adjacent_difference(self, coords):
        start_point = coords[0]
        end_point = coords[-1]
        point = [start_point]
        for i in range(5):
            x = start_point[0] + (end_point[0] - start_point[0]) * (i + 1) / 6
            y = start_point[1]
            point.append([x, y])
        point.append(end_point)
        return point

    def generate_path_points(self, polygon, step_size):
        min_x, min_y, max_x, max_y = polygon.bounds
        path_points = []
        current_y = min_y + step_size / 2
        direction = 1
        
        while current_y < max_y:
            vertical_line = LineString([(min_x+0.2, current_y), (max_x-0.2, current_y)])
            intersection = vertical_line.intersection(polygon)
            # print(f"ronin login : scan once at y={current_y}")
            if not intersection.is_empty:
                if intersection.geom_type == 'LineString':
                    # print(f"ronin login : compute once")
                    
                    coords = list(intersection.coords)
                    # coords = self.Adjacent_difference(coords)
                    # print(f"交线坐标: {coords}")
                    if direction == 1:
                        coords.reverse()
                    path_points.extend(coords)
                    direction *= -1
            current_y += step_size
        return path_points
    
    def publish_path_points(self, path_points, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.ns = "path_points"
        marker.id = 1
        
        # 点样式
        marker.scale.x = 0.05  # 点大小
        marker.scale.y = 0.05
        marker.color.r = 1.0   # 黄色点表示路径点
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        # start_point = path_points[0]
        # end_point = path_points[-1]
        # point = [start_point]
        # for i in range(5):
        #     x = start_point[0] + (end_point[0] - start_point[0]) * (i + 1) / 6
        #     y = start_point[1]
        #     point.append([x, y])
        # point.append(end_point)
        # 填充点数据
        for p in path_points:
        # for p in point:
            point_msg = Point()
            point_msg.x = float(p[0])
            point_msg.y = float(p[1])
            
            point_msg.z = 0.0
            marker.points.append(point_msg)
        self.path_points_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = SparseBorderExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
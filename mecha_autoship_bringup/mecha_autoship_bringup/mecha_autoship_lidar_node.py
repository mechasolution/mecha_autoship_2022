import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32

from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

import math

class MechaAutoshipLidarNode(Node) :
    def __init__(self) :
        super().__init__('mecha_autoship_lidar_node')
        self.get_logger().info('mecha_autoship_lidar_node Start')

        qos_profile = QoSProfile(depth = 10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.scan_points_pub_handler = self.create_publisher(PointCloud, 'scan_points', 10)

        self.scan_sub_handler = self.create_subscription(LaserScan, 'scan', self.scan_pub_callback, qos_profile)
        # print("asdf")


    def scan_pub_callback(self, data) :
        pointcloud_msg = PointCloud()
        pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
        pointcloud_msg.header.frame_id = "base_link"

        angle_increment = data.angle_increment
        angle = 0

        # 각도, 거리 -> 좌표 (m단위)
        for point in data.ranges :
            coordinate_x = math.cos(angle) * point
            coordinate_y = math.sin(angle) * point
            angle += angle_increment
            if abs(coordinate_x) == 0 and abs(coordinate_y) == 0 :
                continue
            point_msg = Point32()
            point_msg.x = coordinate_x
            point_msg.y = coordinate_y
            point_msg.z = float(0)

            pointcloud_msg.points.append(point_msg)

        self.scan_points_pub_handler.publish(pointcloud_msg)

        # 그룹화
        # point_groups = [] # 최종 점들 그룹
        # points_tmp = [] # 최종 점들의 후보 점 임시 저장
        # i_end = len(pointcloud_msg.points)
        # pointcloud_msg_ = PointCloud()
        # point_now = Point32()
        # point_last = Point32()
        # is_end_append = False # 마지막 점이 포함된 그룹이 append되었는지 여부.
        # point_last.x = pointcloud_msg.points[0].x
        # point_last.y = pointcloud_msg.points[0].y
        # for i in range(1, i_end) :
        #     point_now.x = copy.deepcopy(pointcloud_msg.points[i].x)
        #     point_now.y = copy.deepcopy(pointcloud_msg.points[i].y)

        #     #  삼각함수 사용해 점간 거리 계산
        #     length_a = point_last.x - point_now.x
        #     length_b = point_last.y - point_now.y
        #     length_c = math.sqrt((length_a * length_a) + (length_b * length_b))

        #     if length_c < 0.1 : # 이전 점과 거리가 짧으면 임시 그룹에 넣음
        #         points_tmp.append(copy.deepcopy(point_now))
        #     else : # 이전 점에서 멀리 떨어져 있으면 다음 그룹의 점으로 간주하고 기존 그룹은 저장
        #         if len(points_tmp) > 10 : # 구성이 10개보다 큰 경우에만 그룹으로 인정
        #             if i == i_end - 1 :
        #                 is_end_append = True
        #             point_groups.append(copy.deepcopy(points_tmp))
        #         points_tmp = []

        #     point_last.x = copy.deepcopy(point_now.x)
        #     point_last.y = copy.deepcopy(point_now.y)

        # if len(points_tmp) > 10 and is_end_append == False : # 마지막 그룹까지 계산
        #     point_groups.append(copy.deepcopy(points_tmp))


        # 디버깅
        # print(len(point_groups))
        # for group in point_groups :
        #     list_x = []
        #     list_y = []
        #     for point in group :
        #         # plt.scatter(point.x, point.y)
        #         list_x.append(copy.deepcopy(point.x))
        #         list_y.append(copy.deepcopy(point.y))
        #     plt.scatter(list_x, list_y)
        # plt.show()
        # exit()


def main(args=None) :
    rclpy.init(args=args)

    mecha_autoship_lidar_node = MechaAutoshipLidarNode()

    rclpy.spin(mecha_autoship_lidar_node)
    mecha_autoship_lidar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
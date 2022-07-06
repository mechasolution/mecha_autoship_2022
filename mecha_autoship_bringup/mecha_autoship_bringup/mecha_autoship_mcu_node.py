import string
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus

from rclpy.qos import QoSProfile

import serial
import numpy as np
import math

from mecha_autoship_interfaces.srv import Battery
from mecha_autoship_interfaces.srv import Actuator
from mecha_autoship_interfaces.srv import Color

class MechaAutoshipMcu(Node) :
    def __init__(self) :
        super().__init__('mecha_autoship_mcu')
        self.get_logger().info('mecha_autoship_mcu Start')

        qos_profile = QoSProfile(depth = 10)

        self.imu_pub_handler = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub_handler = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.gps_pub_handler = self.create_publisher(NavSatFix, 'gps/data', 10)

        self.create_service(Actuator, 'set_actuator', self.actuator_service_callback)
        self.create_service(Color, 'set_color', self.color_service_callback)

        # init serial
        self._serial = serial.Serial('/dev/ttyACM0', 0)
        self._serial.flushInput()
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

        # self.get_factor()
        self.get_common_data_handler = self.create_timer(0.01, self.get_common_data)
        self.get_gps_data_handler = self.create_timer(1, self.get_gps_data)

    def actuator_service_callback(self, req, res):
        self.get_logger().info('$CD,{0},{1}\n'.format(req.throttle_pwr, req.key_dgr))
        self._serial.write('$CD,{0},{1}\n'.format(req.throttle_pwr, req.key_dgr).encode())
        return res

    def color_service_callback(self, req, res):
        self.get_logger().info('$CL,{0},{1},{2}\n'.format(req.red, req.green, req.blue))
        self._serial.write('$CL,{0},{1},{2}\n'.format(req.red, req.green, req.blue).encode())

    def get_gps_data(self) :
        self._serial.write('$QG\n'.encode())


    def get_common_data(self) :
        packet_raw = self._serial.readline().split(b'\r')[0].decode("utf-8")
        if packet_raw :
            packet_split = packet_raw.split(',')
            try:
                # 평상시 데이터 전송
                if packet_split[0] == '#R0' :
                    # IMU GYR
                    gyro_x = float(packet_split[1]) / 50
                    gyro_y = float(packet_split[2]) / 50
                    gyro_z = float(packet_split[3]) / 50

                    # IMU ACC
                    accel_x = float(packet_split[4])
                    accel_y = float(packet_split[5])
                    accel_z = float(packet_split[6])

                    # IMU MAG
                    mag_x = float(packet_split[7])
                    mag_y = float(packet_split[8])
                    mag_z = float(packet_split[9])

                    # motors
                    servo = int(packet_split[10])
                    bldc = int(packet_split[11])

                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = "imu_link"

                    imu_msg.angular_velocity.x = gyro_x
                    imu_msg.angular_velocity.y = gyro_y
                    imu_msg.angular_velocity.z = gyro_z

                    imu_msg.linear_acceleration.x = accel_x
                    imu_msg.linear_acceleration.y = accel_y
                    imu_msg.linear_acceleration.z = accel_z


                    # 지자기
                    mag_msg = MagneticField()
                    mag_msg.header.stamp = self.get_clock().now().to_msg()
                    # mag_msg.header.frame_id = "imu_link"
                    mag_msg.magnetic_field.x = mag_x
                    mag_msg.magnetic_field.y = mag_y
                    mag_msg.magnetic_field.z = mag_z

                    self.imu_pub_handler.publish(imu_msg)
                    self.mag_pub_handler.publish(mag_msg)

                # GPS
                elif packet_split[0] == '#RG' :
                    latitude_d = float(str(packet_split[1])[0:2])
                    latitude_m = float(str(packet_split[1])[2:])
                    latitude = latitude_d + latitude_m / 60

                    longitude_d = float(str(packet_split[2])[0:3])
                    longitude_m = float(str(packet_split[2])[3:])
                    longitude = longitude_d + longitude_m / 60

                    status_ = int(packet_split[3])

                    gps_status_msg = NavSatStatus()
                    gps_status_msg.status = gps_status_msg.STATUS_NO_FIX if status_ == 0 else gps_status_msg.STATUS_FIX

                    gps_msg = NavSatFix()
                    gps_msg.header.stamp = self.get_clock().now().to_msg()
                    gps_msg.latitude = latitude
                    gps_msg.longitude = longitude
                    gps_msg.status = gps_status_msg

                    self.gps_pub_handler.publish(gps_msg)

            except Exception as e:
                self.get_logger().warn('[get_common_data] communication error! e: {0} / msg: {1}'.format(e, packet_raw))
                pass


def main(args=None) :
    rclpy.init(args=args)

    mecha_autoship_mcu = MechaAutoshipMcu()

    rclpy.spin(mecha_autoship_mcu)
    mecha_autoship_mcu.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
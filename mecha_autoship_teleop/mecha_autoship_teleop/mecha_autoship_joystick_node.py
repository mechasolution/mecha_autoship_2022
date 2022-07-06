#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Joy

from mecha_autoship_interfaces.srv import Actuator
from mecha_autoship_interfaces.srv import Color


def map(x,input_min,input_max,output_min,output_max):
    res = (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min
    return output_min if res < output_min else res


class MechaAutoshipJoystick(Node) :
    # 쓰로틀 최소, 최대
    THROTTLE_MIN = 100
    THROTTLE_MAX = 140

    # 서보모터 최소, 최대
    KEY_MIN = 70
    KEY_MAX = 110

    def __init__(self) :
        super().__init__('mecha_autoship_joystick')
        self.get_logger().info('mecha_autoship_joystick Start')

        qos_profile = QoSProfile(depth = 10)

        self._data_joy = Joy()
        self._is_data = False

        self.create_subscription(Joy, '/joy', self.joy_pub_callback, qos_profile)

        self.set_actuator_handler = self.create_client(Actuator, 'set_actuator')
        self.set_color_handler = self.create_client(Color, 'set_color')

        self.create_timer(0.1, self.set_mcu)


    def joy_pub_callback(self, data) :
        self._data_joy = data
        self._is_data = True

    def set_mcu(self) :
        if self._is_data == False :
            return
        # 위: 양수, 좌: 양수
        stick_left_vertical = self._data_joy.axes[1]
        stick_left_horizontal = self._data_joy.axes[0]

        stick_right_vertical = self._data_joy.axes[2]
        stick_right_horizontal = self._data_joy.axes[3]

        button_1 = self._data_joy.buttons[0]
        button_2 = self._data_joy.buttons[1]
        button_3 = self._data_joy.buttons[2]
        button_4 = self._data_joy.buttons[3]

        data_actuator = Actuator.Request()
        data_actuator.throttle_pwr = int(map(stick_left_vertical, 0, 1, self.THROTTLE_MIN, self.THROTTLE_MAX))
        data_actuator.key_dgr = int(map(stick_right_horizontal, -1, 1, 0, 180))

        data_color = Color.Request()
        change_color = button_1 or button_2 or button_3 or button_4
        if button_1 == 1 :
            data_color.red = 0
            data_color.green = 254
            data_color.blue = 0
        elif button_2 == 1 :
            data_color.red = 254
            data_color.green = 0
            data_color.blue = 0
        elif button_3 == 1 :
            data_color.red = 0
            data_color.green = 0
            data_color.blue = 254
        elif button_4 == 1 :
            data_color.red = 0
            data_color.green = 0
            data_color.blue = 0

        self.set_actuator_handler.call_async(data_actuator)
        if change_color == True :
            self.set_color_handler.call_async(data_color)

def main(args=None) :
    rclpy.init(args=args)

    mecha_autoship_joystick = MechaAutoshipJoystick()

    rclpy.spin(mecha_autoship_joystick)
    mecha_autoship_joystick.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
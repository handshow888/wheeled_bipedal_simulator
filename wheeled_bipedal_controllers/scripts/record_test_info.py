#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import csv
import os
from datetime import datetime


class TestInfoRecorder(Node):
    def __init__(self):
        super().__init__('test_info_recorder')

        base_log_dir = (
            '/home/handshow/wheeled_bipedal_ws/src/'
            'wheeled_bipedal_simulator/'
            'wheeled_bipedal_controllers/logs'
        )

        time_name = datetime.now().strftime('%y-%m-%d-%H%M%S')

        # 每次记录数据时，新建一个以时间命名的文件夹
        self.log_dir = os.path.join(base_log_dir, time_name)
        os.makedirs(self.log_dir, exist_ok=True)

        # CSV 文件也使用同样的时间命名
        self.csv_path = os.path.join(self.log_dir, time_name + '.csv')

        self.header = [
            'time',
            'pitch',
            'roll',

            'left_leg_length_target',
            'right_leg_length_target',
            'left_leg_length_actual',
            'right_leg_length_actual',

            'left_leg_phi0',
            'right_leg_phi0',

            'target_linear_vel',
            'actual_linear_vel',

            'target_angular_vel',
            'actual_angular_vel',

            'left_front_joint_torque',
            'left_rear_joint_torque',
            'right_front_joint_torque',
            'right_rear_joint_torque',

            'left_wheel_torque',
            'right_wheel_torque'
        ]

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(self.header)

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/test_info',
            self.test_info_callback,
            10
        )

        self.get_logger().info(f'Log folder: {self.log_dir}')
        self.get_logger().info(f'Start recording /test_info to: {self.csv_path}')

    def test_info_callback(self, msg):
        data = list(msg.data)

        if len(data) != len(self.header):
            self.get_logger().warn(
                f'Received data length {len(data)}, expected {len(self.header)}. '
                f'Skip this frame.'
            )
            return

        self.writer.writerow(data)
        self.csv_file.flush()

    def close(self):
        self.csv_file.close()
        self.get_logger().info(f'CSV file saved: {self.csv_path}')


def main(args=None):
    rclpy.init(args=args)

    node = TestInfoRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
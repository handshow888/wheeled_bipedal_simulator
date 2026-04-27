#!/usr/bin/env python3
"""
计算 IMU 加速度计的全局缩放因子，使静止时合加速度模长为 9.81 m/s²。
订阅 /imu_sensor_broadcaster/imu，采集 N 个样本后计算 scale = 9.81 / |mean_acc|
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math


class AccelScaleCalibrator(Node):
    def __init__(self):
        super().__init__('accel_scale_calibrator')

        # 订阅 IMU 话题
        self.sub = self.create_subscription(
            Imu,
            '/imu_sensor_broadcaster/imu',
            self.imu_callback,
            10
        )

        # 参数
        self.num_samples = 5000   # 采集样本数，可调整
        self.target_norm = 9.81   # 目标重力加速度 (m/s²)，可根据当地值修改

        # 累加器
        self.sample_count = 0
        self.sum_ax = 0.0
        self.sum_ay = 0.0
        self.sum_az = 0.0
        self.done = False

        self.get_logger().info(
            f'开始采集 {self.num_samples} 个加速度样本，请保持机器人静止...'
        )

    def imu_callback(self, msg: Imu):
        if self.done:
            return

        # 只采集指定数量的样本
        if self.sample_count < self.num_samples:
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z

            self.sum_ax += ax
            self.sum_ay += ay
            self.sum_az += az
            self.sample_count += 1

            if self.sample_count == self.num_samples:
                self.compute_scale()
                self.done = True
                self.get_logger().info('计算完成，节点即将关闭。')
                self.destroy_node()
                rclpy.shutdown()

    def compute_scale(self):
        # 计算平均加速度
        avg_ax = self.sum_ax / self.num_samples
        avg_ay = self.sum_ay / self.num_samples
        avg_az = self.sum_az / self.num_samples

        # 平均加速度的模长
        norm = math.sqrt(avg_ax**2 + avg_ay**2 + avg_az**2)

        # 缩放因子
        scale = self.target_norm / norm

        # 输出结果
        self.get_logger().info('========== 结果 ==========')
        self.get_logger().info(f'平均加速度: [{avg_ax:.6f}, {avg_ay:.6f}, {avg_az:.6f}]')
        self.get_logger().info(f'当前模长: {norm:.6f} m/s²')
        self.get_logger().info(f'目标模长: {self.target_norm:.6f} m/s²')
        self.get_logger().info(f'缩放因子 scale: {scale:.8f}')
        self.get_logger().info('')
        self.get_logger().info(
            '使用方式：将原始加速度各轴分别乘以该 scale 值即可。\n'
            '例如：acc_corrected = scale * raw_acc'
        )
        self.get_logger().info(
            '如果需要在 ros2_control 参数中保存，可添加一个参数 '
            '"accel_scale" 值为上述数字。'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AccelScaleCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
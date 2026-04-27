#!/usr/bin/env python3
"""
ROS2 IMU 零偏计算节点
订阅 IMU 话题，收集指定数量的样本，计算角速度和线加速度的均值与标准差。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from collections import deque


class ImuBiasCalculator(Node):
    def __init__(self):
        super().__init__('imu_bias_calculator')

        # 参数声明与获取
        self.declare_parameter('imu_topic', '/imu_sensor_broadcaster/imu')
        self.declare_parameter('sample_count', 5000)          # 收集样本数量
        self.declare_parameter('timeout_sec', 0.0)           # 可选超时（0表示仅用样本数）

        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.sample_count = self.get_parameter('sample_count').get_parameter_value().integer_value
        self.timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value

        # 数据存储队列
        self.angular_vel_x = deque(maxlen=self.sample_count)
        self.angular_vel_y = deque(maxlen=self.sample_count)
        self.angular_vel_z = deque(maxlen=self.sample_count)
        self.linear_acc_x = deque(maxlen=self.sample_count)
        self.linear_acc_y = deque(maxlen=self.sample_count)
        self.linear_acc_z = deque(maxlen=self.sample_count)

        # 订阅 IMU 话题
        self.subscription = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10
        )
        self.get_logger().info(f'订阅 IMU 话题: {imu_topic}')
        self.get_logger().info(f'目标样本数: {self.sample_count}')

        # 定时器：用于超时检测或周期性打印进度（可选）
        if self.timeout_sec > 0:
            self.timeout_timer = self.create_timer(self.timeout_sec, self.timeout_callback)
        else:
            self.timeout_timer = None

        self.start_time = self.get_clock().now()
        self.calculated = False  # 防止重复计算输出

    def imu_callback(self, msg: Imu):
        """IMU 数据回调：存储角速度和线加速度分量"""
        if self.calculated:
            return  # 已经完成计算，不再存储

        # 存储数据（使用队列自动维持最大长度）
        self.angular_vel_x.append(msg.angular_velocity.x * 180.0 / np.pi)
        self.angular_vel_y.append(msg.angular_velocity.y * 180.0 / np.pi)
        self.angular_vel_z.append(msg.angular_velocity.z * 180.0 / np.pi)
        self.linear_acc_x.append(msg.linear_acceleration.x)
        self.linear_acc_y.append(msg.linear_acceleration.y)
        self.linear_acc_z.append(msg.linear_acceleration.z)

        # 检查是否收集了足够样本
        if len(self.angular_vel_x) >= self.sample_count:
            self.compute_and_print_bias()

    def timeout_callback(self):
        """超时回调：若启用超时且数据不足，强制计算已有数据"""
        if not self.calculated and len(self.angular_vel_x) > 0:
            self.get_logger().warn(f'达到超时时间 {self.timeout_sec} 秒，使用当前 {len(self.angular_vel_x)} 个样本计算')
            self.compute_and_print_bias()

    def compute_and_print_bias(self):
        """计算均值和标准差并输出"""
        if self.calculated:
            return
        self.calculated = True

        # 转换为 numpy 数组
        ang_x = np.array(self.angular_vel_x)
        ang_y = np.array(self.angular_vel_y)
        ang_z = np.array(self.angular_vel_z)
        acc_x = np.array(self.linear_acc_x)
        acc_y = np.array(self.linear_acc_y)
        acc_z = np.array(self.linear_acc_z)

        # 计算统计量
        bias_ang = np.array([np.mean(ang_x), np.mean(ang_y), np.mean(ang_z)])
        std_ang = np.array([np.std(ang_x), np.std(ang_y), np.std(ang_z)])
        bias_acc = np.array([np.mean(acc_x), np.mean(acc_y), np.mean(acc_z)])
        std_acc = np.array([np.std(acc_x), np.std(acc_y), np.std(acc_z)])

        # 输出结果
        self.get_logger().info('========== IMU 零偏计算结果 ==========')
        self.get_logger().info(f'样本数量: {len(ang_x)}')
        self.get_logger().info(f'角速度零偏 (°/s):  X={bias_ang[0]:.6f}, Y={bias_ang[1]:.6f}, Z={bias_ang[2]:.6f}')
        self.get_logger().info(f'角速度标准差 (°/s): X={std_ang[0]:.6f}, Y={std_ang[1]:.6f}, Z={std_ang[2]:.6f}')
        self.get_logger().info(f'加速度零偏 (m/s²):  X={bias_acc[0]:.6f}, Y={bias_acc[1]:.6f}, Z={bias_acc[2]:.6f}')
        self.get_logger().info(f'加速度标准差 (m/s²): X={std_acc[0]:.6f}, Y={std_acc[1]:.6f}, Z={std_acc[2]:.6f}')
        self.get_logger().info('========================================')
        self.get_logger().info(f'角速度零偏 (rad/s):  X={bias_ang[0]*np.pi/180.0:.6f}, Y={bias_ang[1]*np.pi/180.0:.6f}, Z={bias_ang[2]*np.pi/180.0:.6f}')
        self.get_logger().info('========================================')

        # 可选：将结果写入文件
        # self.save_to_file(bias_ang, std_ang, bias_acc, std_acc)

        # 计算完成后可以关闭节点，或者继续保持运行
        rclpy.shutdown()

    # def save_to_file(self, bias_ang, std_ang, bias_acc, std_acc):
    #     """将结果保存到文本文件"""
    #     import os
    #     from datetime import datetime
    #     filename = f"imu_bias_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    #     with open(filename, 'w') as f:
    #         f.write("IMU Bias Calculation Results\n")
    #         f.write(f"Samples: {len(self.angular_vel_x)}\n")
    #         f.write(f"Gyro Bias (rad/s): {bias_ang[0]:.8f} {bias_ang[1]:.8f} {bias_ang[2]:.8f}\n")
    #         f.write(f"Gyro Std  (rad/s): {std_ang[0]:.8f} {std_ang[1]:.8f} {std_ang[2]:.8f}\n")
    #         f.write(f"Accel Bias (m/s^2): {bias_acc[0]:.8f} {bias_acc[1]:.8f} {bias_acc[2]:.8f}\n")
    #         f.write(f"Accel Std  (m/s^2): {std_acc[0]:.8f} {std_acc[1]:.8f} {std_acc[2]:.8f}\n")
    #     self.get_logger().info(f'结果已保存至: {filename}')


def main(args=None):
    rclpy.init(args=args)
    node = ImuBiasCalculator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
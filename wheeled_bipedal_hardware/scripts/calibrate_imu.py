#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuAlignmentCalib(Node):
    def __init__(self):
        super().__init__('imu_alignment_calib')
        self.sub = self.create_subscription(
            Imu,
            '/imu_sensor_broadcaster/imu',
            self.imu_cb,
            10)

        self.num_samples = 5000
        self.cnt = 0
        self.sum_ax = 0.0
        self.sum_ay = 0.0
        self.sum_az = 0.0
        self.done = False

        self.get_logger().info('IMU 校准节点启动，请保持机器人水平静止...')

    def imu_cb(self, msg: Imu):
        if self.done:
            return
        if self.cnt < self.num_samples:
            self.sum_ax += msg.linear_acceleration.x
            self.sum_ay += msg.linear_acceleration.y
            self.sum_az += msg.linear_acceleration.z
            self.cnt += 1
            if self.cnt == self.num_samples:
                self.compute()
                self.done = True
                self.get_logger().info('校准完成，节点即将关闭')
                rclpy.shutdown()

    def compute(self):
        ax_m = self.sum_ax / self.num_samples
        ay_m = self.sum_ay / self.num_samples
        az_m = self.sum_az / self.num_samples

        self.get_logger().info(f'平均加速度 (原始): [{ax_m:.4f}, {ay_m:.4f}, {az_m:.4f}]')

        norm = math.sqrt(ax_m**2 + ay_m**2 + az_m**2)
        gx = ax_m / norm
        gy = ay_m / norm
        gz = az_m / norm

        # 目标：重力指向 Z 轴正方向（向下），请根据你的坐标系调整！
        # 如果你的机体 Z 轴向上，静止时加速度应为 (0,0,-1)，此处需修改为 (0,0,-1)
        gx_t, gy_t, gz_t = 0.0, 0.0, 1.0

        # 叉积 = 旋转轴
        cx = gy * gz_t - gz * gy_t
        cy = gz * gx_t - gx * gz_t
        cz = gx * gy_t - gy * gx_t

        sin_a = math.sqrt(cx**2 + cy**2 + cz**2)  # = |sin(theta)|
        cos_a = gx * gx_t + gy * gy_t + gz * gz_t
        angle = math.atan2(sin_a, cos_a)

        if sin_a < 1e-6:
            # 已经对齐
            R = [[1.0, 0.0, 0.0],
                 [0.0, 1.0, 0.0],
                 [0.0, 0.0, 1.0]]
        else:
            ux = cx / sin_a
            uy = cy / sin_a
            uz = cz / sin_a
            c = math.cos(angle)
            s = math.sin(angle)
            one_minus_c = 1.0 - c

            R = [
                [c + ux*ux*one_minus_c, ux*uy*one_minus_c - uz*s, ux*uz*one_minus_c + uy*s],
                [uy*ux*one_minus_c + uz*s, c + uy*uy*one_minus_c, uy*uz*one_minus_c - ux*s],
                [uz*ux*one_minus_c - uy*s, uz*uy*one_minus_c + ux*s, c + uz*uz*one_minus_c]
            ]

        self.get_logger().info('===== 校准矩阵（旋转矩阵） =====')
        for row in R:
            self.get_logger().info(f'[{row[0]:8.6f}, {row[1]:8.6f}, {row[2]:8.6f}]')

        # 扁平列表，方便直接填入 ros2_control 参数
        flat = [str(v) for row in R for v in row]
        self.get_logger().info(f'\n适用于 ros2_control 的扁平列表：\n[{", ".join(flat)}]')
        self.get_logger().info('请将上面一行复制到你的 URDF/Xacro 参数中。')


def main():
    rclpy.init()
    node = ImuAlignmentCalib()
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
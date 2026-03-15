#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion
import sys

class TfListener(Node):
    def __init__(self, parent_frame, child_frame):
        super().__init__('tf_listener')
        self.parent_frame = parent_frame
        self.child_frame = child_frame

        # TF缓冲区与监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 定时器，每0.1秒输出一次（10Hz）
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        try:
            # 获取当前时刻的变换（等待最多0.1秒）
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # 平移部分
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            # 旋转部分（四元数转欧拉角，返回 roll, pitch, yaw）
            q = trans.transform.rotation
            quat = [q.x, q.y, q.z, q.w]
            roll, pitch, yaw = euler_from_quaternion(quat)

            # 打印结果
            self.get_logger().info(
                f"Translation: x={x:.4f}, y={y:.4f}, z={z:.4f} | "
                f"Rotation (rpy): roll={roll:.4f}, pitch={pitch:.4f}, yaw={yaw:.4f}"
            )

        except TransformException as e:
            self.get_logger().warn(f"Could not get transform: {e}")

def main():
    # 检查命令行参数
    if len(sys.argv) != 3:
        print("Usage: python3 tf_listener.py <parent_frame> <child_frame>")
        sys.exit(1)

    parent_frame = sys.argv[1]
    child_frame = sys.argv[2]

    # 初始化ROS2
    rclpy.init()

    # 创建节点并运行
    node = TfListener(parent_frame, child_frame)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import sys
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
from scipy.spatial.transform import Rotation as R
import math

class TFListener(Node):
    def __init__(self, parent_frame, child_frame):
        super().__init__('simple_tf_listener')
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        
        # 初始化 TF 缓存和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 定时器：每 0.1 秒监听一次 (10Hz)
        self.timer = self.create_timer(0.1, self.on_timer)
        
        self.get_logger().info(f'开始监听: {parent_frame} -> {child_frame}')

    def on_timer(self):
        try:
            # 查找最新转换
            now = rclpy.time.Time()
            t = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                now)

            # 获取位移
            pos = t.transform.translation
            
            # 获取四元数 [x, y, z, w]
            q = t.transform.rotation
            quat = [q.x, q.y, q.z, q.w]

            # --- 关键部分：使用 Scipy 转换欧拉角 ---
            # 我们指定旋转顺序为 'xyz'。在这种情况下：
            # r[0] = roll, r[1] = pitch, r[2] = yaw
            r = R.from_quat(quat)
            euler = r.as_euler('xyz', degrees=True)
            
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]

            rot_vec = r.as_rotvec() 
            # 如果主要旋转在 Y 轴，rot_vec[1] 就是你的弧度制 Pitch
            pitch_pure = math.degrees(rot_vec[1])

            # 打印结果，重点观察 Pitch
            # 如果你确定只有 Pitch，可以忽略 Roll 和 Yaw 的微小波动
            self.get_logger().info(
                f'\n--- Transform ---\n'
                f'Position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}\n'
                f'Rotation (deg): Roll={roll:.2f}, Pitch={pitch:.2f}, Yaw={yaw:.2f}\n'
                f'pitch_pure(radian)={pitch_pure / 180.0 * 3.14159265358979323846:.4f}, pitch_pure(deg)={pitch_pure:.4f}'
            )

        except TransformException as ex:
            self.get_logger().info(f'无法获取坐标变换: {ex}')

def main():
    if len(sys.argv) < 3:
        print("用法: python3 tf_listener.py <parentFrame> <childFrame>")
        return

    parent = sys.argv[1]
    child = sys.argv[2]

    rclpy.init()
    node = TFListener(parent, child)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
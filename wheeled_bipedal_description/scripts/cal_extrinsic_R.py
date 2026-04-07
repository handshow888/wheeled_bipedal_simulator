from scipy.spatial.transform import Rotation as R
from math import pi

deg2rad = pi/180.0

# 假设雷达相对于 IMU 绕 Z 轴转了 90 度 (0, 0, 1.57)
r = R.from_euler('xyz', [0*deg2rad, 15*deg2rad, 0])
print(r.as_matrix())
import math

# ================= 配置区域 =================
FILE_NAME = "trapezoid_road.world"

# 尺寸配置
ROAD_WIDTH = 2.0        # 路面宽度 (m)
THICKNESS = 0.05        # 路板厚度 (m) (设薄一点可避免底部干涉，但足够提供稳定的碰撞体积)

# 梯形路段配置
BUMP_COUNT = 4          # 连续起伏（梯形）的数量
SLOPE_DEG = 17.0        # 斜坡角度 (度) - 可改为 5.0
SLOPE_LENGTH = 0.5      # 斜坡表面的长度 (m)
TOP_LENGTH = 0.5        # 坡顶短平台(梯形上底)的长度 (m)
# BOTTOM_LENGTH = 1.484807753     # 两个梯形之间的平地(下底)长度 (m)
BOTTOM_LENGTH = SLOPE_LENGTH+math.cos(SLOPE_DEG/180.0*math.pi)     # 两个梯形之间的平地(下底)长度 (m)
START_FLAT = 0.0        # 机器人起始的缓冲平地长度 (m)
# ============================================

sdf_header = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    
    <model name="trapezoid_road">
      <static>true</static>
"""
sdf_footer = """
    </model>
  </world>
</sdf>
"""

current_x = 0.0
current_z = 0.0
link_counter = 0
links_xml = ""

def add_segment(name_prefix, length, angle_deg, color_name):
    global current_x, current_z, link_counter, links_xml
    
    # 将角度转为弧度
    theta = math.radians(angle_deg)
    
    # 1. 计算路段上表面（供机器人行驶的面）的起点和终点
    x0, z0 = current_x, current_z
    x1 = x0 + length * math.cos(theta)
    z1 = z0 + length * math.sin(theta)
    
    # 2. 计算上表面的中心点
    xc = (x0 + x1) / 2.0
    zc = (z0 + z1) / 2.0
    
    # 3. 计算长方体自身的中心点（从上表面中心沿法线向下偏移一半厚度）
    # 法线方向为 (-sin(theta), cos(theta))
    box_x = xc + (THICKNESS / 2.0) * math.sin(theta)
    box_z = zc - (THICKNESS / 2.0) * math.cos(theta)
    
    # 4. Gazebo 的 Pitch 规则：绕Y轴旋转，负值表示车头向上翘起（上坡）
    pitch = -theta
    
    link_xml = f"""
      <link name='{name_prefix}_{link_counter}'>
        <pose>{box_x:.4f} 0 {box_z:.4f} 0 {pitch:.4f} 0</pose>
        <collision name='collision'>
          <geometry><box><size>{length:.4f} {ROAD_WIDTH} {THICKNESS}</size></box></geometry>
          <surface>
            <friction>
              <ode><mu>100.0</mu><mu2>50.0</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{length:.4f} {ROAD_WIDTH} {THICKNESS}</size></box></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/{color_name}</name>
            </script>
          </material>
        </visual>
      </link>"""
      
    links_xml += link_xml
    link_counter += 1
    
    # 更新下一个路段的起点
    current_x = x1
    current_z = z1

def generate_world():
    # 1. 起始缓冲平地
    add_segment("start_flat", START_FLAT, 0.0, "Grey")
    
    # 2. 循环生成等腰梯形起伏
    for i in range(BUMP_COUNT):
        # 上坡
        add_segment(f"bump_{i}_up", SLOPE_LENGTH, SLOPE_DEG, "DarkGrey")
        # 坡顶短平台
        add_segment(f"bump_{i}_top", TOP_LENGTH, 0.0, "Grey")
        # 下坡 (角度取负)
        add_segment(f"bump_{i}_down", SLOPE_LENGTH, -SLOPE_DEG, "DarkGrey")
        
        # 两个起伏之间的连接平地
        if i < BUMP_COUNT - 1:
            add_segment(f"bump_{i}_bottom", BOTTOM_LENGTH, 0.0, "Grey")
            
    # 3. 结束缓冲平地
    add_segment("end_flat", START_FLAT, 0.0, "Grey")

    # 写入文件
    with open(FILE_NAME, "w") as f:
        f.write(sdf_header + links_xml + sdf_footer)
    print(f"成功生成完美的等腰梯形组合路面: {FILE_NAME}")

if __name__ == "__main__":
    generate_world()
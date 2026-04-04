import math

def calculate_box_size(m, ixx, iyy, izz):
    """根据质量和主惯性矩计算长方体边长"""
    try:
        dx = math.sqrt(6 * (iyy + izz - ixx) / m)
        dy = math.sqrt(6 * (ixx + izz - iyy) / m)
        dz = math.sqrt(6 * (ixx + iyy - izz) / m)
        return dx, dy, dz
    except ValueError:
        return None

def calculate_cylinder_size(m, i_axis, i_side):
    """
    根据质量和惯量计算圆柱体尺寸
    i_axis: 绕中心轴的惯量 (如 izz)
    i_side: 绕横向轴的惯量 (如 ixx 或 iyy)
    """
    try:
        r = math.sqrt(2 * i_axis / m)
        h = math.sqrt((12 * i_side - 6 * i_axis) / m)
        return r, h
    except ValueError:
        return None

def main():
    print("--- Gazebo 碰撞模型简化工具 ---")
    shape_type = input("请输入形状类型 (1: 长方体, 2: 圆柱体): ")
    m = float(input("质量 (mass) [kg]: "))

    if shape_type == "1":
        ixx = float(input("ixx: "))
        iyy = float(input("iyy: "))
        izz = float(input("izz: "))
        
        size = calculate_box_size(m, ixx, iyy, izz)
        if size:
            dx, dy, dz = size
            print(f"\n推荐 Collision Box 尺寸:")
            print(f"<box size=\"{dx:.4f} {dy:.4f} {dz:.4f}\"/>")
        else:
            print("\n错误：惯量参数不符合物理定律（三角不等式），请检查输入。")

    elif shape_type == "2":
        i_axis = float(input("绕中心轴的惯量 (旋转轴): "))
        i_side = float(input("绕横向轴的惯量: "))
        
        size = calculate_cylinder_size(m, i_axis, i_side)
        if size:
            r, h = size
            print(f"\n推荐 Collision Cylinder 尺寸:")
            print(f"<cylinder radius=\"{r:.4f}\" length=\"{h:.4f}\"/>")
        else:
            print("\n错误：惯量参数计算出负数，请检查输入。")

if __name__ == "__main__":
    main()
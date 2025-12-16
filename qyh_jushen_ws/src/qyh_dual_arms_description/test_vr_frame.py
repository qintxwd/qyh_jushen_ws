#!/usr/bin/env python3
"""
测试VR坐标系转换
验证base_vr坐标系的设置是否正确
"""
import numpy as np
from scipy.spatial.transform import Rotation as R

def test_coordinate_transform():
    """测试坐标系转换"""
    print("=" * 60)
    print("VR坐标系转换测试")
    print("=" * 60)
    
    # base_link到base_vr的旋转：rpy="0 0 -1.5708" (即绕Z轴逆时针90度)
    # 这个旋转使得：
    # - base_vr的X轴(向右) = base_link的Y轴(向左)方向的反向 = base_link的-Y
    # - base_vr的Y轴(向上) = base_link的X轴方向 = base_link的X... 不对
    
    # 让我重新计算
    # base_link: X前, Y左, Z上 (从图片看)
    # base_vr需要: X右, Y上, Z后
    
    # X右 = -Y(base_link)
    # Y上 = Z(base_link)  
    # Z后 = -X(base_link)
    
    # 这需要的旋转是：先绕Z轴转-90度，再绕新的X轴转90度
    rpy_z_only = [0, 0, -np.pi/2]
    rot_z = R.from_euler('xyz', rpy_z_only)
    
    print("\n方案1: 仅绕Z轴旋转-90度")
    print(f"RPY: {rpy_z_only}")
    print(f"旋转矩阵:\n{rot_z.as_matrix()}")
    
    # 测试各轴转换
    print("\nbase_link坐标系下的单位向量 -> base_vr坐标系:")
    for axis, vec in [("X(前)", [1, 0, 0]), ("Y(左)", [0, 1, 0]), ("Z(上)", [0, 0, 1])]:
        transformed = rot_z.as_matrix().T @ np.array(vec)  # 逆变换
        print(f"  base_link {axis} -> base_vr: {transformed}")
    
    print("\n" + "=" * 60)
    
    # 正确的转换应该是组合旋转
    # 方法：X右, Y上, Z后
    # 从base_link(X前Y左Z上)到base_vr(X右Y上Z后)
    # 需要：绕Z轴-90度 + 绕新X轴90度
    rpy_correct = [np.pi/2, 0, -np.pi/2]
    rot_correct = R.from_euler('xyz', rpy_correct)
    
    print("\n方案2: 组合旋转 (绕X 90度, 绕Z -90度)")
    print(f"RPY: {rpy_correct}")
    print(f"旋转矩阵:\n{rot_correct.as_matrix()}")
    
    print("\nbase_link坐标系下的单位向量 -> base_vr坐标系:")
    for axis, vec in [("X(前)", [1, 0, 0]), ("Y(左)", [0, 1, 0]), ("Z(上)", [0, 0, 1])]:
        transformed = rot_correct.as_matrix().T @ np.array(vec)
        print(f"  base_link {axis} -> base_vr: {transformed}")
    
    print("\nbase_vr坐标系下的单位向量 -> base_link坐标系:")
    for axis, vec in [("X(右)", [1, 0, 0]), ("Y(上)", [0, 1, 0]), ("Z(后)", [0, 0, 1])]:
        transformed = rot_correct.as_matrix() @ np.array(vec)
        print(f"  base_vr {axis} -> base_link: {transformed}")
    
    # 验证
    print("\n" + "=" * 60)
    print("验证结果:")
    vr_x_right = rot_correct.as_matrix() @ np.array([1, 0, 0])  # base_vr的X(右)
    vr_y_up = rot_correct.as_matrix() @ np.array([0, 1, 0])     # base_vr的Y(上)
    vr_z_back = rot_correct.as_matrix() @ np.array([0, 0, 1])   # base_vr的Z(后)
    
    print(f"base_vr X(右) 在 base_link: {vr_x_right} (应该是[0, -1, 0]或接近)")
    print(f"base_vr Y(上) 在 base_link: {vr_y_up} (应该是[0, 0, 1]或接近)")
    print(f"base_vr Z(后) 在 base_link: {vr_z_back} (应该是[-1, 0, 0]或接近)")

def test_vr_increment():
    """测试VR增量控制"""
    print("\n" + "=" * 60)
    print("VR增量控制测试")
    print("=" * 60)
    
    # 使用正确的旋转矩阵
    rpy = [np.pi/2, 0, -np.pi/2]
    rot = R.from_euler('xyz', rpy).as_matrix()
    
    print("\n场景1: VR手柄向右移动 [0.1, 0, 0]")
    vr_delta = np.array([0.1, 0, 0])
    robot_delta = rot @ vr_delta
    print(f"  -> 机械臂在base_link下应移动: {robot_delta}")
    print(f"  -> 应该是Y负方向(向右)")
    
    print("\n场景2: VR手柄向上移动 [0, 0.1, 0]")
    vr_delta = np.array([0, 0.1, 0])
    robot_delta = rot @ vr_delta
    print(f"  -> 机械臂在base_link下应移动: {robot_delta}")
    print(f"  -> 应该是Z正方向(向上)")
    
    print("\n场景3: VR手柄向后移动 [0, 0, 0.1]")
    vr_delta = np.array([0, 0, 0.1])
    robot_delta = rot @ vr_delta
    print(f"  -> 机械臂在base_link下应移动: {robot_delta}")
    print(f"  -> 应该是X负方向(向后)")
    
    print("\n场景4: VR手柄对角移动 [0.1, 0.1, 0.1]")
    vr_delta = np.array([0.1, 0.1, 0.1])
    robot_delta = rot @ vr_delta
    print(f"  -> 机械臂在base_link下应移动: {robot_delta}")

if __name__ == "__main__":
    test_coordinate_transform()
    test_vr_increment()
    
    print("\n" + "=" * 60)
    print("建议的URDF配置:")
    print("  <joint name=\"base_to_vr\" type=\"fixed\">")
    print("    <origin xyz=\"0 0 0\" rpy=\"1.5708 0 -1.5708\" />")
    print("    <parent link=\"base_link\" />")
    print("    <child link=\"base_vr\" />")
    print("  </joint>")
    print("=" * 60)

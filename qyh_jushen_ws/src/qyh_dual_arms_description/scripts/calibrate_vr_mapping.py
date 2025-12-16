#!/usr/bin/python3
"""
VR位置映射标定工具

使用方法:
1. 运行此脚本: ros2 run qyh_dual_arms_description calibrate_vr_mapping.py
2. 按提示移动VR手柄到5个标准姿态
3. 每个姿态: 按空格键记录VR位置，输入机械臂目标XYZ
4. 自动计算并保存映射参数到yaml文件

标准姿态建议:
1. 双臂打开，与肩平齐
2. 双臂垂直放下
3. 双臂举过头顶
4. 双臂水平前伸
5. 双臂弯曲靠近身体
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import yaml
import os
from pathlib import Path


class VRMappingCalibrator(Node):
    """VR映射标定工具"""
    
    def __init__(self):
        super().__init__('vr_mapping_calibrator')
        
        # 订阅VR位置话题
        self.left_sub = self.create_subscription(
            PoseStamped, '/vr_base_vr/left_pose', 
            self._left_callback, 10)
        self.right_sub = self.create_subscription(
            PoseStamped, '/vr_base_vr/right_pose',
            self._right_callback, 10)
        
        # 当前VR位置
        self.current_left_pos = None
        self.current_right_pos = None
        
        # 标定数据存储
        self.left_vr_points = []  # VR空间的点
        self.left_robot_points = []  # 机械臂空间的点
        self.right_vr_points = []
        self.right_robot_points = []
        
        # 配置文件路径
        self.config_dir = Path(__file__).parent.parent / 'config'
        self.config_file = self.config_dir / 'vr_mapping_calibration.yaml'
        self.poses_file = self.config_dir / 'robot_calibration_poses.yaml'
        
        # 加载机械臂标定姿态配置
        self.robot_poses = self._load_robot_poses()
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('VR位置映射标定工具')
        self.get_logger().info('=' * 60)
        self.get_logger().info('请确保vr_to_base_vr节点正在运行')
        self.get_logger().info('等待VR数据...')
    
    def _load_robot_poses(self):
        """加载机械臂标定姿态配置"""
        if not self.poses_file.exists():
            self.get_logger().warn(f'未找到姿态配置文件: {self.poses_file}')
            self.get_logger().warn('将使用手动输入模式')
            return None
        
        try:
            with open(self.poses_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            
            poses = config.get('calibration_poses', {})
            if not poses:
                self.get_logger().warn('姿态配置文件为空，使用手动输入模式')
                return None
            
            self.get_logger().info(f'✓ 已加载姿态配置: {self.poses_file}')
            return poses
        
        except Exception as e:
            self.get_logger().error(f'加载姿态配置失败: {e}')
            return None
    
    def _left_callback(self, msg):
        """接收左手VR位置"""
        self.current_left_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
    
    def _right_callback(self, msg):
        """接收右手VR位置"""
        self.current_right_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
    
    def calibrate_hand(self, hand_name):
        """标定单只手（4个姿态）"""
        vr_points = []
        robot_points = []
        
        # 姿态定义（4个精简姿态）
        pose_keys = ['arms_open', 'arms_down', 'arms_forward', 'arms_forward_low']
        pose_descriptions = {
            'arms_open': '双臂打开，与肩平齐',
            'arms_down': '双臂垂直放下',
            'arms_forward': '双臂水平前伸',
            'arms_forward_low': '双臂低位前伸（端托盘姿态）'
        }
        
        self.get_logger().info(f'\n开始标定{hand_name}手...')
        arm_key = 'left_arm' if hand_name == '左' else 'right_arm'
        
        for i, pose_key in enumerate(pose_keys, 1):
            pose_desc = pose_descriptions[pose_key]
            self.get_logger().info(f'\n姿态 {i}/4: {pose_desc} ({pose_key})')
            self.get_logger().info('-'*60)
            
            # 等待用户摆好姿势
            input(f'请摆好姿态{i}，然后按回车记录VR位置...')
            
            # 记录VR位置
            current_pos = self.current_left_pos if hand_name == "左" else self.current_right_pos
            
            if current_pos is None:
                self.get_logger().error(f'错误: 未接收到{hand_name}手VR数据！')
                self.get_logger().error('请确保VR手柄已开启并且vr_to_base_vr节点正在运行')
                return None
            
            vr_pos = current_pos.copy()
            self.get_logger().info(f'✓ VR位置: X={vr_pos[0]:.3f}, Y={vr_pos[1]:.3f}, Z={vr_pos[2]:.3f}')
            
            # 获取机械臂目标位置
            robot_pos = None
            
            # 尝试从配置文件读取
            if self.robot_poses and pose_key in self.robot_poses:
                pose_data = self.robot_poses[pose_key]
                if arm_key in pose_data:
                    pos_data = pose_data[arm_key]['position']
                    # 检查是否已填写（非全零）
                    if pos_data['x'] != 0.0 or pos_data['y'] != 0.0 or pos_data['z'] != 0.0:
                        robot_pos = np.array([pos_data['x'], pos_data['y'], pos_data['z']])
                        self.get_logger().info(f'✓ 从配置文件读取机械臂目标位置')
                        self.get_logger().info(f'  X={robot_pos[0]:.3f}, Y={robot_pos[1]:.3f}, Z={robot_pos[2]:.3f}')
                        
                        # 询问是否使用此值
                        confirm = input('  使用此位置？(Y/n): ').strip().lower()
                        if confirm in ['n', 'no']:
                            robot_pos = None
            
            # 手动输入
            if robot_pos is None:
                self.get_logger().info(f'\n请输入机械臂{hand_name}手在此姿态的目标位置:')
                
                while True:
                    try:
                        x = float(input('  X (米): '))
                        y = float(input('  Y (米): '))
                        z = float(input('  Z (米): '))
                        robot_pos = np.array([x, y, z])
                        break
                    except ValueError:
                        break
                except ValueError:
                    self.get_logger().warn('输入无效，请重新输入数字')
            
            self.get_logger().info(f'✓ 机械臂目标: X={robot_pos[0]:.3f}, Y={robot_pos[1]:.3f}, Z={robot_pos[2]:.3f}')
            
            vr_points.append(vr_pos)
            robot_points.append(robot_pos)
            self.get_logger().info(f'✓ 姿态{i}数据已记录')
        
        return vr_points, robot_points
    
    def compute_mapping(self, vr_points, robot_points):
        """计算映射参数 P' = scale * P + offset
        
        使用最小二乘法求解每个轴独立的线性映射
        """
        vr_array = np.array(vr_points)  # (5, 3)
        robot_array = np.array(robot_points)  # (5, 3)
        
        scale = np.zeros(3)
        offset = np.zeros(3)
        
        # 对每个轴独立求解线性回归: robot_i = scale_i * vr_i + offset_i
        for axis in range(3):
            vr_axis = vr_array[:, axis]  # (5,)
            robot_axis = robot_array[:, axis]  # (5,)
            
            # 构建最小二乘矩阵: [vr_1 1] [scale]   [robot_1]
            #                    [vr_2 1] [offset] = [robot_2]
            #                    [ ...  ]            [ ...   ]
            A = np.column_stack([vr_axis, np.ones(5)])  # (5, 2)
            b = robot_axis  # (5,)
            
            # 求解: (A^T A)^-1 A^T b
            result, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
            scale[axis] = result[0]
            offset[axis] = result[1]
            
            # 计算拟合误差
            predicted = scale[axis] * vr_axis + offset[axis]
            rmse = np.sqrt(np.mean((predicted - robot_axis)**2))
            
            axis_name = ['X', 'Y', 'Z'][axis]
            self.get_logger().info(
                f'  {axis_name}轴: scale={scale[axis]:.4f}, '
                f'offset={offset[axis]:.4f}, RMSE={rmse:.4f}m')
        
        return scale, offset
    
    def save_calibration(self):
        """保存标定结果到YAML文件"""
        # 确保目录存在
        self.config_dir.mkdir(parents=True, exist_ok=True)
        
        config = {
            'vr_mapping_calibration': {
                'left': {
                    'scale': {
                        'x': float(self.left_scale[0]),
                        'y': float(self.left_scale[1]),
                        'z': float(self.left_scale[2])
                    },
                    'offset': {
                        'x': float(self.left_offset[0]),
                        'y': float(self.left_offset[1]),
                        'z': float(self.left_offset[2])
                    }
                },
                'right': {
                    'scale': {
                        'x': float(self.right_scale[0]),
                        'y': float(self.right_scale[1]),
                        'z': float(self.right_scale[2])
                    },
                    'offset': {
                        'x': float(self.right_offset[0]),
                        'y': float(self.right_offset[1]),
                        'z': float(self.right_offset[2])
                    }
                }
            }
        }
        
        with open(self.config_file, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)
        
        self.get_logger().info(f'\n✓ 标定结果已保存到: {self.config_file}')
    
    def run_calibration(self):
        """运行完整的标定流程"""
        self.get_logger().info('\n请确保:')
        self.get_logger().info('1. 双手都握持VR手柄')
        self.get_logger().info('2. vr_to_base_vr节点正在运行')
        self.get_logger().info('3. 准备好机械臂各姿态的目标位置数据\n')
        
        input('准备好后按回车开始标定...')
        
        # 标定左手
        left_result = self.calibrate_hand("左")
        if left_result is None:
            return False
        
        self.left_vr_points, self.left_robot_points = left_result
        
        # 标定右手
        right_result = self.calibrate_hand("右")
        if right_result is None:
            return False
        
        self.right_vr_points, self.right_robot_points = right_result
        
        # 计算映射参数
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('计算映射参数...')
        self.get_logger().info('='*60)
        
        self.get_logger().info('\n左手映射参数:')
        self.left_scale, self.left_offset = self.compute_mapping(
            self.left_vr_points, self.left_robot_points)
        
        self.get_logger().info('\n右手映射参数:')
        self.right_scale, self.right_offset = self.compute_mapping(
            self.right_vr_points, self.right_robot_points)
        
        # 保存结果
        self.save_calibration()
        
        # 显示使用说明
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('标定完成！')
        self.get_logger().info('='*60)
        self.get_logger().info('\n使用方法:')
        self.get_logger().info('1. 启动vr_to_base_vr节点时会自动加载此配置')
        self.get_logger().info('2. 或手动设置ROS参数:')
        self.get_logger().info(f'   left_scale_x:={self.left_scale[0]:.4f}')
        self.get_logger().info(f'   left_scale_y:={self.left_scale[1]:.4f}')
        self.get_logger().info(f'   left_scale_z:={self.left_scale[2]:.4f}')
        self.get_logger().info(f'   left_offset_x:={self.left_offset[0]:.4f}')
        self.get_logger().info(f'   left_offset_y:={self.left_offset[1]:.4f}')
        self.get_logger().info(f'   left_offset_z:={self.left_offset[2]:.4f}')
        self.get_logger().info('   (右手参数类似)')
        
        return True


def main(args=None):
    rclpy.init(args=args)
    calibrator = VRMappingCalibrator()
    
    # 创建一个单独的线程来处理ROS回调
    import threading
    
    def spin_thread():
        rclpy.spin(calibrator)
    
    spin = threading.Thread(target=spin_thread, daemon=True)
    spin.start()
    
    # 等待一下确保收到VR数据
    import time
    time.sleep(1.0)
    
    try:
        # 运行标定
        success = calibrator.run_calibration()
        
        if success:
            calibrator.get_logger().info('\n标定成功！')
        else:
            calibrator.get_logger().error('\n标定失败！')
    
    except KeyboardInterrupt:
        calibrator.get_logger().info('\n标定已取消')
    except Exception as e:
        calibrator.get_logger().error(f'\n标定出错: {e}')
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

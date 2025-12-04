"""
预设加载器 (Preset Loader)

用于 ROS2 任务引擎从持久化存储加载预设数据
"""

import os
import json
from pathlib import Path
from typing import Dict, Any, Optional, List


class PresetLoader:
    """
    预设加载器
    
    从 ~/qyh_jushen_ws/persistent/preset/ 加载预设数据
    供 ROS2 任务引擎使用
    """
    
    FILE_MAP = {
        "location": "locations.json",
        "arm_pose": "arm_points.json",  # 机械臂点位
        "lift_height": "lift_heights.json",
        "waist_angle": "waist_angles.json",  # 腰部角度
        "head_position": "head_positions.json",
        "gripper_position": "gripper_positions.json",
        "task_template": "task_templates.json",
    }
    
    # 不同文件的数据字段名
    DATA_FIELD_MAP = {
        "arm_pose": "points",  # arm_points.json 使用 "points" 字段
    }
    
    def __init__(self, storage_path: str = None):
        if storage_path is None:
            home = Path.home()
            storage_path = home / "qyh_jushen_ws" / "persistent" / "preset"
        
        self.storage_path = Path(storage_path)
        
        # 地图文件路径 (在 ~/qyh_jushen_ws/maps 文件夹)
        home = Path.home()
        self.maps_dir = home / "qyh_jushen_ws" / "maps"
        
        self._cache: Dict[str, Dict[str, Any]] = {}
        self._load_all()
        self._load_map_stations()  # 从地图加载站点
        self._add_builtin_presets()
    
    def _load_all(self):
        """加载所有预设文件"""
        for preset_type, filename in self.FILE_MAP.items():
            self._cache[preset_type] = {}
            filepath = self.storage_path / filename
            if filepath.exists():
                try:
                    with open(filepath, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                    
                    # 获取数据字段名（默认为 items）
                    data_field = self.DATA_FIELD_MAP.get(preset_type, 'items')
                    items = data.get(data_field, data.get('items', []))
                    
                    for item in items:
                        item_id = item.get('id')
                        if item_id:
                            self._cache[preset_type][item_id] = item
                            # 同时用名称作为索引
                            name = item.get('name')
                            if name:
                                self._cache[preset_type][name] = item
                    print(f"✓ 加载预设文件 [{filename}]: {len(items)} 个条目")
                except Exception as e:
                    print(f"⚠️  加载预设文件失败 [{filename}]: {e}")
    
    def _load_map_stations(self):
        """从地图数据加载站点到 location 预设"""
        if not self.maps_dir.exists():
            print("⚠️  地图目录不存在，跳过站点加载")
            return
        
        # 读取当前地图名
        current_map_file = self.maps_dir / "current_map.txt"
        if not current_map_file.exists():
            print("⚠️  未找到当前地图信息，跳过站点加载")
            return
        
        current_map = current_map_file.read_text(encoding='utf-8').strip()
        if not current_map:
            return
        
        # 读取地图JSON数据
        map_json_file = self.maps_dir / current_map / f"{current_map}.json"
        if not map_json_file.exists():
            print(f"⚠️  地图数据文件不存在: {current_map}")
            return
        
        try:
            with open(map_json_file, 'r', encoding='utf-8') as f:
                map_data = json.load(f)
            
            stations = map_data.get('data', {}).get('station', [])
            self._cache.setdefault("location", {})
            
            count = 0
            for station in stations:
                station_id = station.get('id')
                station_name = station.get('name', f"站点{station_id}")
                
                # 转换为预设格式 (mm -> m, 1/1000 rad -> rad)
                loc_data = {
                    "id": f"station_{station_id}",
                    "name": station_name,
                    "x": station.get('pos.x', 0.0) / 1000.0,  # mm -> m
                    "y": station.get('pos.y', 0.0) / 1000.0,  # mm -> m
                    "theta": station.get('pos.yaw', 0.0) / 1000.0,  # 1/1000 rad -> rad
                    "station_id": station_id  # 保留原始站点ID
                }
                
                # 用 id 和 name 都作为索引
                self._cache["location"][f"station_{station_id}"] = loc_data
                self._cache["location"][station_name] = loc_data
                count += 1
            
            print(f"✓ 从地图 [{current_map}] 加载了 {count} 个站点")
        except Exception as e:
            print(f"⚠️  加载地图站点失败: {e}")
    
    def _add_builtin_presets(self):
        """添加内置预设（只在文件中没有时才添加）"""
        # 内置点位
        builtin_locations = {
            "loc_origin": {"id": "loc_origin", "name": "原点", "x": 0.0, "y": 0.0, "theta": 0.0},
            "原点": {"id": "loc_origin", "name": "原点", "x": 0.0, "y": 0.0, "theta": 0.0},
            "loc_charging": {"id": "loc_charging", "name": "充电桩", "x": 0.0, "y": 0.0, "theta": 0.0},
            "充电桩": {"id": "loc_charging", "name": "充电桩", "x": 0.0, "y": 0.0, "theta": 0.0},
        }
        for k, v in builtin_locations.items():
            if k not in self._cache.get("location", {}):
                self._cache.setdefault("location", {})[k] = v
        
        # 内置姿态
        builtin_arm_poses = {
            # 常用别名
            "zero": {
                "id": "zero", "name": "零位", "side": "both",
                "left_joints": [0.0] * 7, "right_joints": [0.0] * 7
            },
            "零位": {
                "id": "zero", "name": "零位", "side": "both",
                "left_joints": [0.0] * 7, "right_joints": [0.0] * 7
            },
            "home": {
                "id": "home", "name": "初始点", "side": "both",
                "left_joints": [0.0, -0.79, 0.0, -1.56, 0.0, 1.36, 0.0],
                "right_joints": [0.0, -0.75, 0.0, -1.45, 0.0, -1.14, 0.0]
            },
            "初始点": {
                "id": "home", "name": "初始点", "side": "both",
                "left_joints": [0.0, -0.79, 0.0, -1.56, 0.0, 1.36, 0.0],
                "right_joints": [0.0, -0.75, 0.0, -1.45, 0.0, -1.14, 0.0]
            },
            # 兼容旧命名
            "pose_home": {
                "id": "pose_home", "name": "初始姿态", "side": "both",
                "left_joints": [0.0] * 7, "right_joints": [0.0] * 7
            },
            "初始姿态": {
                "id": "pose_home", "name": "初始姿态", "side": "both",
                "left_joints": [0.0] * 7, "right_joints": [0.0] * 7
            },
            "pose_observe": {
                "id": "pose_observe", "name": "观察姿态", "side": "both",
                "left_joints": [0.0, -0.5, 0.0, 1.2, 0.0, 0.5, 0.0],
                "right_joints": [0.0, -0.5, 0.0, 1.2, 0.0, 0.5, 0.0]
            },
            "观察姿态": {
                "id": "pose_observe", "name": "观察姿态", "side": "both",
                "left_joints": [0.0, -0.5, 0.0, 1.2, 0.0, 0.5, 0.0],
                "right_joints": [0.0, -0.5, 0.0, 1.2, 0.0, 0.5, 0.0]
            },
            "pose_handover": {
                "id": "pose_handover", "name": "递物姿态", "side": "both",
                "left_joints": [0.0, 0.3, 0.0, 1.0, 0.0, 0.7, 0.0],
                "right_joints": [0.0, 0.3, 0.0, 1.0, 0.0, 0.7, 0.0]
            },
            "递物姿态": {
                "id": "pose_handover", "name": "递物姿态", "side": "both",
                "left_joints": [0.0, 0.3, 0.0, 1.0, 0.0, 0.7, 0.0],
                "right_joints": [0.0, 0.3, 0.0, 1.0, 0.0, 0.7, 0.0]
            },
            "pose_raise_hands": {
                "id": "pose_raise_hands", "name": "举起双手", "side": "both",
                "left_joints": [0.0, -1.5, 0.0, 0.5, 0.0, 0.3, 0.0],
                "right_joints": [0.0, -1.5, 0.0, 0.5, 0.0, 0.3, 0.0]
            },
            "举起双手": {
                "id": "pose_raise_hands", "name": "举起双手", "side": "both",
                "left_joints": [0.0, -1.5, 0.0, 0.5, 0.0, 0.3, 0.0],
                "right_joints": [0.0, -1.5, 0.0, 0.5, 0.0, 0.3, 0.0]
            },
        }
        for k, v in builtin_arm_poses.items():
            if k not in self._cache.get("arm_pose", {}):
                self._cache.setdefault("arm_pose", {})[k] = v
        
        # 内置升降高度
        builtin_lift_heights = {
            "lift_bottom": {"id": "lift_bottom", "name": "底部", "height": 0.0},
            "底部": {"id": "lift_bottom", "name": "底部", "height": 0.0},
            "lift_middle": {"id": "lift_middle", "name": "中间", "height": 250.0},
            "中间": {"id": "lift_middle", "name": "中间", "height": 250.0},
            "lift_top": {"id": "lift_top", "name": "顶部", "height": 500.0},
            "顶部": {"id": "lift_top", "name": "顶部", "height": 500.0},
            "lift_desk": {"id": "lift_desk", "name": "桌面平齐", "height": 180.0},
            "桌面平齐": {"id": "lift_desk", "name": "桌面平齐", "height": 180.0},
        }
        for k, v in builtin_lift_heights.items():
            if k not in self._cache.get("lift_height", {}):
                self._cache.setdefault("lift_height", {})[k] = v
        
        # 内置腰部角度
        builtin_waist_angles = {
            "upright": {"id": "upright", "name": "竖直", "angle": 0.0},
            "竖直": {"id": "upright", "name": "竖直", "angle": 0.0},
            "slight_lean": {"id": "slight_lean", "name": "轻微前倾", "angle": 15.0},
            "轻微前倾": {"id": "slight_lean", "name": "轻微前倾", "angle": 15.0},
            "medium_lean": {"id": "medium_lean", "name": "中等前倾", "angle": 30.0},
            "中等前倾": {"id": "medium_lean", "name": "中等前倾", "angle": 30.0},
            "max_lean": {"id": "max_lean", "name": "最大前倾", "angle": 45.0},
            "最大前倾": {"id": "max_lean", "name": "最大前倾", "angle": 45.0},
        }
        for k, v in builtin_waist_angles.items():
            if k not in self._cache.get("waist_angle", {}):
                self._cache.setdefault("waist_angle", {})[k] = v
        
        # 内置头部位置
        builtin_head_positions = {
            "head_center": {"id": "head_center", "name": "正前方", "pan": 0.0, "tilt": 0.0},
            "正前方": {"id": "head_center", "name": "正前方", "pan": 0.0, "tilt": 0.0},
            "head_left": {"id": "head_left", "name": "左侧", "pan": -0.8, "tilt": 0.0},
            "左侧": {"id": "head_left", "name": "左侧", "pan": -0.8, "tilt": 0.0},
            "head_right": {"id": "head_right", "name": "右侧", "pan": 0.8, "tilt": 0.0},
            "右侧": {"id": "head_right", "name": "右侧", "pan": 0.8, "tilt": 0.0},
            "head_up": {"id": "head_up", "name": "抬头", "pan": 0.0, "tilt": -0.5},
            "抬头": {"id": "head_up", "name": "抬头", "pan": 0.0, "tilt": -0.5},
            "head_down": {"id": "head_down", "name": "低头", "pan": 0.0, "tilt": 0.5},
            "低头": {"id": "head_down", "name": "低头", "pan": 0.0, "tilt": 0.5},
        }
        for k, v in builtin_head_positions.items():
            if k not in self._cache.get("head_position", {}):
                self._cache.setdefault("head_position", {})[k] = v
        
        # 内置夹爪位置
        builtin_gripper_positions = {
            "gripper_open": {"id": "gripper_open", "name": "完全打开", "side": "both", "left_position": 1.0, "right_position": 1.0},
            "完全打开": {"id": "gripper_open", "name": "完全打开", "side": "both", "left_position": 1.0, "right_position": 1.0},
            "gripper_close": {"id": "gripper_close", "name": "完全关闭", "side": "both", "left_position": 0.0, "right_position": 0.0},
            "完全关闭": {"id": "gripper_close", "name": "完全关闭", "side": "both", "left_position": 0.0, "right_position": 0.0},
            "gripper_half": {"id": "gripper_half", "name": "半开", "side": "both", "left_position": 0.5, "right_position": 0.5},
            "半开": {"id": "gripper_half", "name": "半开", "side": "both", "left_position": 0.5, "right_position": 0.5},
        }
        for k, v in builtin_gripper_positions.items():
            if k not in self._cache.get("gripper_position", {}):
                self._cache.setdefault("gripper_position", {})[k] = v
    
    def reload(self):
        """重新加载所有预设"""
        self._cache.clear()
        self._load_all()
        self._load_map_stations()  # 从地图加载站点
        self._add_builtin_presets()
    
    def get_location(self, name_or_id: str) -> Optional[Dict[str, Any]]:
        """获取点位预设"""
        return self._cache.get("location", {}).get(name_or_id)
    
    def get_arm_pose(self, name_or_id: str) -> Optional[Dict[str, Any]]:
        """获取手臂姿态预设"""
        return self._cache.get("arm_pose", {}).get(name_or_id)
    
    def get_lift_height(self, name_or_id: str) -> Optional[Dict[str, Any]]:
        """获取升降高度预设"""
        return self._cache.get("lift_height", {}).get(name_or_id)
    
    def get_waist_angle(self, name_or_id: str) -> Optional[Dict[str, Any]]:
        """获取腰部角度预设"""
        return self._cache.get("waist_angle", {}).get(name_or_id)
    
    def get_head_position(self, name_or_id: str) -> Optional[Dict[str, Any]]:
        """获取头部位置预设"""
        return self._cache.get("head_position", {}).get(name_or_id)
    
    def get_gripper_position(self, name_or_id: str) -> Optional[Dict[str, Any]]:
        """获取夹爪位置预设"""
        return self._cache.get("gripper_position", {}).get(name_or_id)
    
    def get_task_template(self, name_or_id: str) -> Optional[Dict[str, Any]]:
        """获取任务模板预设"""
        return self._cache.get("task_template", {}).get(name_or_id)
    
    def list_presets(self, preset_type: str) -> List[Dict[str, Any]]:
        """列出指定类型的所有预设"""
        presets = self._cache.get(preset_type, {})
        # 去重（因为同时用 id 和 name 索引）
        seen_ids = set()
        result = []
        for preset in presets.values():
            preset_id = preset.get("id")
            if preset_id and preset_id not in seen_ids:
                seen_ids.add(preset_id)
                result.append(preset)
        return result


# 全局单例
preset_loader = PresetLoader()

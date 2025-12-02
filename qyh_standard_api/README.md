# Standard Robots Matrix 地图同步工具

从 Standard Robots Matrix 机器人底盘获取地图数据。

## 功能

- ✅ 通过 WebSocket 获取当前激活的地图名称
- ✅ 通过 HTTP API 下载所有地图数据（JSON + 图片）
- ✅ 生成 `current_map.txt` 记录当前地图

## 安装

```bash
pip install -r requirements.txt
```

## 配置

编辑 `config.yaml` 配置机器人地址和凭据：

```yaml
matrix:
  host: "192.168.71.50"  # 机器人 IP
  port: 80               # HTTP API 端口
  ws_port: 5002          # WebSocket 端口
  username: "dev"        # 登录用户名
  password: "_sr_dev_"   # 登录密码
  timeout: 10            # 超时秒数
```

## 使用方法

```bash
python get_map.py
```

执行后会：
1. 清空 `~/qyh_jushen_ws/maps/` 目录
2. 从机器人获取当前地图名称
3. 下载所有地图的 JSON 数据和图片
4. 在 `maps/` 目录生成 `current_map.txt`

## 输出结构

```
qyh_jushen_ws/
└── maps/
    ├── current_map.txt      # 当前地图名称
    ├── yuanchang/
    │   ├── yuanchang.json   # 地图数据 (站点、路径等)
    │   └── yuanchang.png    # 地图图片
    ├── standard/
    │   ├── standard.json
    │   └── standard.png
    └── ...
```

## 地图数据结构

JSON 文件包含以下主要内容：

- `meta`: 地图元信息（分辨率、尺寸等）
- `data.node`: 导航节点列表
- `data.edge`: 路径/边列表
- `data.station`: 站点/工位列表

### 节点 (node)

```json
{
  "id": 1,
  "x": -2124.98,     // 位置 X (mm)
  "y": -1772.85,     // 位置 Y (mm)
  "yaw": 0,          // 朝向 (rad)
  "desc": ""
}
```

### 边 (edge)

```json
{
  "id": 1,
  "type": 1,         // 1=直线, 3=贝塞尔曲线
  "sx": -2124.98, "sy": -1772.85,  // 起点
  "ex": -7658.12, "ey": -1417.72,  // 终点
  "cx": 0, "cy": 0,  // 控制点1 (贝塞尔曲线)
  "dx": 0, "dy": 0,  // 控制点2 (贝塞尔曲线)
  "s_node": 1,       // 起始节点ID
  "e_node": 2        // 终点节点ID
}
```

### 站点 (station)

```json
{
  "id": 1,
  "name": "工位A",
  "type": "NORMAL",
  "pos": {
    "x": 22780,      // 位置 X (mm)
    "y": 5350,       // 位置 Y (mm)
    "yaw": 3141      // 朝向 (毫弧度, 1/1000 rad)
  }
}
```

## License

MIT

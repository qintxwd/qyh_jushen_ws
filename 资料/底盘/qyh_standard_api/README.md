# Standard Robots Matrix API

Python 客户端库和命令行工具，用于与 Standard Robots Matrix 系统交互。

## 功能

- ✅ 获取地图列表
- ✅ 下载地图数据（JSON + 图片）
- ✅ 获取导航节点、路径、工位等信息
- ✅ 获取账户列表
- ✅ 地图渲染（节点、边、站点方向）
- ✅ 猜测当前地图（基于修改时间）
- 🔲 精确获取当前地图（需要 WebSocket + Protobuf）
- 🔲 WebSocket 实时状态订阅（待实现）

## 安装

```bash
# 安装依赖
pip install requests pyyaml pillow
```

## 配置

编辑 `config.yaml`:

```yaml
matrix:
  host: "192.168.71.50"
  port: 80

# 可选：认证信息（当前 REST API 不需要）
auth:
  username: "dev"
  password: "_sr_dev_"
```

## 命令行用法

```bash
# 列出所有地图
python main.py list

# 下载所有地图
python main.py download

# 下载指定地图
python main.py download standard

# 显示地图详情
python main.py info standard

# 显示工位列表
python main.py stations standard

# 显示节点列表
python main.py nodes standard -n 10

# 渲染地图（带节点、边、站点方向）
python render_map.py
```

## Python API 用法

```python
from matrix import MatrixClient, EDGE_TYPE_BEZIER

# 创建客户端
client = MatrixClient("192.168.71.50")

# 猜测当前地图
current_map = client.guess_current_map()
print(f"当前地图: {current_map}")

# 获取地图列表
maps = client.get_map_list()
for m in maps:
    print(f"{m.name}: {m.modify_time}")

# 获取地图元数据
meta = client.get_map_meta("standard")
print(f"分辨率: {meta.resolution} mm/pixel")
print(f"尺寸: {meta.width}x{meta.height}")

# 获取导航节点
nodes = client.get_map_nodes("standard")
print(f"节点数: {len(nodes)}")

# 获取路径边（包含贝塞尔曲线信息）
edges = client.get_map_edges("standard")
for e in edges:
    if e.is_bezier():
        # 贝塞尔曲线
        p0, p1, p2, p3 = e.get_bezier_points()
        print(f"曲线: {p0} -> {p1} -> {p2} -> {p3}")
    else:
        # 直线
        print(f"直线: ({e.sx},{e.sy}) -> ({e.ex},{e.ey})")

# 获取工位列表（带方向）
stations = client.get_map_stations("standard")
for s in stations:
    yaw_deg = s.get_yaw_deg()  # 转换为度
    print(f"{s.name}: ({s.pos_x}, {s.pos_y}), 方向={yaw_deg:.1f}°")

# 下载地图
result = client.download_map("standard", "./maps")
print(f"JSON: {result['json']}")
print(f"图片: {result['image']}")
```

## Edge (边) 绘制规则

### 边类型

| type | 名称 | 说明 |
|------|------|------|
| 1 | LINE | 直线 |
| 2 | ARC | 圆弧（暂未使用）|
| 3 | BEZIER | 三次贝塞尔曲线 |

### 直线 (type=1)

```
起点: (sx, sy)
终点: (ex, ey)
```

### 贝塞尔曲线 (type=3)

使用三次贝塞尔曲线公式：

```
B(t) = (1-t)³P0 + 3(1-t)²tP1 + 3(1-t)t²P2 + t³P3

P0 = (sx, sy)    # 起点
P1 = (cx, cy)    # 控制点1
P2 = (dx, dy)    # 控制点2  
P3 = (ex, ey)    # 终点

t ∈ [0, 1]
```

JavaScript 示例：
```javascript
ctx.beginPath();
ctx.moveTo(sx, sy);
ctx.bezierCurveTo(cx, cy, dx, dy, ex, ey);
ctx.stroke();
```

## Station (站点) 方向

站点 `pos.yaw` 的单位是 **毫弧度** (1/1000 rad)：

```python
# 转换为弧度
yaw_rad = pos_yaw / 1000.0

# 转换为度
yaw_deg = pos_yaw / 1000.0 * 180 / math.pi

# 示例
# pos_yaw = 3141.6 → 约 180°
# pos_yaw = 1570.8 → 约 90°
```

方向是从 X 轴正方向逆时针测量。

## API 端点

### 地图 API

| 方法 | 端点 | 说明 |
|------|------|------|
| GET | `/api/v0/map` | 获取地图列表 |
| GET | `/api/v2/map` | 获取地图列表（v2，更详细） |
| GET | `/api/v0/map/{name}/data` | 获取地图 JSON 数据 |
| GET | `/api/v0/map/{name}/image?scale=1` | 获取地图图片 |
| GET | `/api/v0/map/{name}/export` | 导出地图 |
| POST | `/api/v0/map/{name}/load` | 加载地图 |

### 账户 API

| 方法 | 端点 | 说明 |
|------|------|------|
| GET | `/api/v0/accounts` | 获取账户列表 |
| GET | `/api/v0/account/{username}` | 获取账户信息 |

## 地图数据结构

### Meta（元数据）

```json
{
  "version": "1.12.2",
  "resolution": 2,           // mm/pixel
  "size.x": 7168,           // width in pixels
  "size.y": 7168,           // height in pixels
  "zero_offset.x": 2047,    // origin x in pixels
  "zero_offset.y": 5119,    // origin y in pixels
  "length_unit": "mm",
  "angle_unit": "1/1000 rad"
}
```

### Data（地图数据）

- `node`: 导航节点列表
- `edge`: 路径边列表
- `station`: 工位/站点列表
- `area`: 区域列表
- `dmcode`: 二维码列表
- `locationArea`: 定位区域列表

### Node（节点）

```json
{
  "id": 1,
  "x": 22780,    // mm
  "y": 5350,     // mm
  "yaw": 0.785,  // rad
  "desc": "节点描述"
}
```

### Station（工位）

```json
{
  "id": 1,
  "name": "s2-流水线",
  "type": "NORMAL",
  "pos.x": 22780,
  "pos.y": 5350,
  "pos.yaw": 0.785,
  "desc": "工位描述"
}
```

## 跨平台支持

此工具支持在以下平台运行：
- Windows (x64)
- Linux (x64)
- Linux (ARM64，如 Jetson)

## 项目结构

```
qyh_standard_api/
├── config.yaml          # 配置文件
├── main.py              # 命令行入口
├── requirements.txt     # Python 依赖
├── README.md            # 本文档
├── matrix/              # 客户端模块
│   ├── __init__.py
│   └── client.py        # MatrixClient 类
└── maps/                # 下载的地图
    ├── standard/
    │   ├── standard.json
    │   └── standard.png
    └── ...
```

## License

MIT

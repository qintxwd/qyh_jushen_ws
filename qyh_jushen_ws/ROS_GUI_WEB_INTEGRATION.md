# ROS GUI Web 集成说明文档

## 概述

本文档说明了如何在 QYH Jushen Web 管理界面中集成和使用三个 ROS Python GUI：
- **qyh_gripper_gui** - 夹爪控制界面
- **qyh_jaka_control_gui** - JAKA 双臂机器人控制界面
- **qyh_standard_robot_gui** - Standard 移动底盘控制界面

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                    Web 浏览器                            │
│  ┌────────────────────────────────────────────────┐    │
│  │  Vue3 前端 (http://localhost:3000/ros-gui)    │    │
│  │  - 启动/停止 GUI                                │    │
│  │  - 查看状态                                     │    │
│  │  - 复制 ROS2 命令                              │    │
│  └───────────────────┬────────────────────────────┘    │
└────────────────────────┼──────────────────────────────┘
                         │ HTTP API
                         ↓
┌─────────────────────────────────────────────────────────┐
│              FastAPI 后端 (localhost:8000)              │
│  ┌────────────────────────────────────────────────┐    │
│  │  /api/v1/ros-gui                               │    │
│  │  - POST /{gui_type}/start   启动 GUI          │    │
│  │  - POST /{gui_type}/stop    停止 GUI          │    │
│  │  - GET  /status             查询状态           │    │
│  │  - GET  /{gui_type}/logs    查看日志          │    │
│  └───────────────────┬────────────────────────────┘    │
└────────────────────────┼──────────────────────────────┘
                         │ subprocess.Popen
                         ↓
┌─────────────────────────────────────────────────────────┐
│                  ROS2 GUI 进程                          │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐            │
│  │ Gripper  │  │   JAKA   │  │ Chassis  │            │
│  │   GUI    │  │   GUI    │  │   GUI    │            │
│  │  (PyQt5) │  │  (PyQt5) │  │  (PyQt5) │            │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘            │
│       │             │             │                     │
│       └─────────────┴─────────────┘                     │
│                     │                                    │
│              ROS2 Topics/Services                       │
└────────────────────────┼──────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────┐
│                  ROS2 系统                              │
│  - /left/gripper_state                                  │
│  - /robot_state                                         │
│  - /chassis/status                                      │
│  - ... (各种 ROS2 话题和服务)                          │
└─────────────────────────────────────────────────────────┘
```

## 功能特性

### 1. Web 界面管理
- ✅ 在浏览器中启动/停止 ROS GUI
- ✅ 实时查看 GUI 运行状态
- ✅ 批量操作（一键启动/停止所有 GUI）
- ✅ 仅管理员可访问

### 2. 安全控制
- ✅ JWT 身份验证
- ✅ 管理员权限验证
- ✅ 进程隔离（每个 GUI 独立进程）
- ✅ 优雅关闭（SIGTERM → SIGKILL）

### 3. 用户体验
- ✅ 清晰的功能说明
- ✅ ROS2 命令示例（可复制）
- ✅ 状态实时刷新（5秒间隔）
- ✅ 响应式设计

## 文件结构

```
qyh_jushen_web/
├── frontend/src/
│   ├── views/
│   │   ├── Dashboard.vue           # 已修改：添加 ROS GUI 入口
│   │   └── RosGUI.vue              # 新增：ROS GUI 管理页面
│   └── router/
│       └── index.ts                # 已修改：添加 /ros-gui 路由
│
└── backend/app/
    ├── api/
    │   └── ros_gui.py              # 新增：ROS GUI 管理 API
    ├── dependencies.py             # 已存在：权限检查依赖
    └── main.py                     # 已修改：注册新路由

qyh_jushen_ws/src/
├── qyh_gripper_gui/
│   └── qyh_gripper_gui/
│       └── gripper_gui.py          # ROS 夹爪 GUI
├── qyh_jaka_control_gui/
│   └── qyh_jaka_control_gui/
│       └── jaka_complete_gui.py    # ROS JAKA GUI
└── qyh_standard_robot_gui/
    └── qyh_standard_robot_gui/
        └── standard_robot_monitor.py  # ROS 底盘 GUI
```

## 使用指南

### 前置条件

1. **ROS2 环境已配置**
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/qyh_jushen_ws/qyh_jushen_ws/install/setup.bash
   ```

2. **ROS GUI 包已编译**
   ```bash
   cd ~/qyh_jushen_ws/qyh_jushen_ws
   colcon build --packages-select qyh_gripper_gui qyh_jaka_control_gui qyh_standard_robot_gui
   ```

3. **Web 后端已启动**
   ```bash
   cd ~/qyh_jushen_ws/qyh_jushen_web/backend
   source venv/bin/activate
   uvicorn app.main:app --reload
   ```

4. **Web 前端已启动**
   ```bash
   cd ~/qyh_jushen_ws/qyh_jushen_web/frontend
   npm run dev
   ```

### 访问步骤

1. **登录管理员账户**
   - 访问: http://localhost:3000
   - 用户名: `admin`
   - 密码: `admin123`

2. **进入 ROS GUI 管理页面**
   - 在 Dashboard 点击 "ROS GUI控制" 按钮
   - 或直接访问: http://localhost:3000/ros-gui

3. **启动 GUI**
   - 切换到对应的标签页（夹爪/机械臂/底盘）
   - 点击 "启动" 按钮
   - GUI 窗口将在系统桌面上打开

4. **使用 GUI**
   - 在弹出的 PyQt5 窗口中进行操作
   - Web 页面会显示 GUI 运行状态

5. **停止 GUI**
   - 返回 Web 页面
   - 点击 "停止" 按钮
   - 或直接关闭 PyQt5 窗口

### 批量操作

在 "全部管理" 标签页中：
- **启动所有 GUI**: 一键启动三个 GUI
- **停止所有 GUI**: 一键停止所有运行中的 GUI

## API 接口

### 基础 URL
```
http://localhost:8000/api/v1/ros-gui
```

### 认证
所有接口需要在 Header 中携带 JWT Token：
```
Authorization: Bearer <your_token>
```

### 接口列表

#### 1. 获取所有 GUI 状态
```http
GET /status
```

**响应示例**:
```json
{
  "gripper": "running",
  "jaka": "stopped",
  "chassis": "running"
}
```

#### 2. 启动 GUI
```http
POST /{gui_type}/start
```

**参数**:
- `gui_type`: `gripper` | `jaka` | `chassis`

**响应示例**:
```json
{
  "success": true,
  "message": "夹爪控制 GUI 启动成功",
  "pid": 12345
}
```

#### 3. 停止 GUI
```http
POST /{gui_type}/stop
```

**响应示例**:
```json
{
  "success": true,
  "message": "夹爪控制 GUI 已停止"
}
```

#### 4. 停止所有 GUI
```http
POST /stop-all
```

**响应示例**:
```json
{
  "success": true,
  "message": "批量停止完成",
  "results": [
    {
      "gui_type": "gripper",
      "success": true,
      "message": "夹爪控制 GUI 已停止"
    }
  ]
}
```

#### 5. 查看 GUI 日志
```http
GET /{gui_type}/logs?lines=50
```

**响应示例**:
```json
{
  "success": true,
  "logs": "[INFO] GUI started successfully\n..."
}
```

## ROS2 命令参考

### 夹爪控制

```bash
# 启动夹爪控制节点
ros2 launch qyh_gripper_control dual_gripper.launch.py

# 激活左手夹爪
ros2 service call /left/activate_gripper qyh_gripper_msgs/srv/ActivateGripper

# 移动左手夹爪
ros2 service call /left/move_gripper qyh_gripper_msgs/srv/MoveGripper \
  "{position: 255, speed: 200, force: 150}"

# 查看夹爪状态
ros2 topic echo /left/gripper_state
```

### JAKA 控制

```bash
# 启动 JAKA 控制节点
ros2 run qyh_jaka_control jaka_control_node \
  --ros-args -p robot_ip:=192.168.2.200

# 启动伺服模式
ros2 service call /jaka/start_servo qyh_jaka_control_msgs/srv/StartServo

# 查看机器人状态
ros2 topic echo /robot_state

# 查看关节状态
ros2 topic echo /joint_states
```

### 底盘控制

```bash
# 启动底盘控制节点
ros2 run qyh_standard_robot standard_robot_node \
  --ros-args -p robot_ip:=192.168.1.100

# 手动控制底盘
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 查看底盘状态
ros2 topic echo /chassis/status
```

## 故障排查

### 1. GUI 无法启动

**问题**: 点击启动按钮后，GUI 状态仍为"未启动"

**解决方案**:
```bash
# 检查 ROS2 环境
echo $ROS_DISTRO  # 应输出 humble

# 检查包是否已编译
ros2 pkg list | grep qyh_gripper_gui

# 手动测试启动
ros2 run qyh_gripper_gui gripper_gui

# 查看后端日志
# 在 backend 终端查看错误信息
```

### 2. 权限不足

**问题**: 提示 "需要 Admin 权限"

**解决方案**:
- 确保使用管理员账户登录
- 检查 JWT Token 是否有效
- 重新登录

### 3. GUI 窗口不显示

**问题**: 状态显示"运行中"，但看不到窗口

**解决方案**:
```bash
# 检查 DISPLAY 环境变量
echo $DISPLAY  # 应输出 :0 或 :1

# 检查 X11 权限
xhost +local:

# 如果是远程服务器，需要 X11 转发
ssh -X user@server
```

### 4. 进程无法停止

**问题**: 点击停止按钮后，进程仍在运行

**解决方案**:
```bash
# 查找进程
ps aux | grep gripper_gui

# 手动杀死进程
kill -9 <pid>

# 或使用 pkill
pkill -f gripper_gui
```

### 5. Web 页面 404

**问题**: 访问 /ros-gui 提示 404

**解决方案**:
```bash
# 重启前端开发服务器
cd frontend
npm run dev

# 检查路由配置
cat src/router/index.ts | grep ros-gui
```

## 安全注意事项

1. **生产环境部署**
   - 修改默认管理员密码
   - 使用 HTTPS
   - 配置防火墙规则
   - 限制 API 访问频率

2. **进程管理**
   - 定期清理僵尸进程
   - 监控系统资源使用
   - 设置进程数量限制

3. **日志管理**
   - 定期清理日志文件
   - 配置日志轮转
   - 敏感信息脱敏

## 开发扩展

### 添加新的 ROS GUI

1. **创建 ROS 包**
   ```bash
   ros2 pkg create --build-type ament_python my_new_gui
   ```

2. **更新后端 API**
   ```python
   # backend/app/api/ros_gui.py
   GUI_COMMANDS['my_gui'] = ['ros2', 'run', 'my_new_gui', 'my_gui_node']
   GUI_NAMES['my_gui'] = '我的新 GUI'
   ```

3. **更新前端界面**
   ```vue
   <!-- frontend/src/views/RosGUI.vue -->
   <el-tab-pane label="新 GUI" name="my_gui">
     <!-- 添加标签页内容 -->
   </el-tab-pane>
   ```

### 添加日志查看功能

前端可以调用 `/api/v1/ros-gui/{gui_type}/logs` 接口显示实时日志：

```typescript
const fetchLogs = async (guiType: string) => {
  const response = await axios.get(
    `${API_BASE}/ros-gui/${guiType}/logs?lines=100`,
    { headers: { 'Authorization': `Bearer ${token}` } }
  )
  console.log(response.data.logs)
}
```

## 性能优化

1. **减少状态轮询频率**
   - 默认 5 秒，可根据需要调整
   - 使用 WebSocket 实时推送

2. **进程池管理**
   - 限制同时运行的 GUI 数量
   - 实现进程复用机制

3. **缓存优化**
   - 缓存 GUI 状态
   - 使用 Redis 共享状态

## 更新日志

### v1.0.0 (2025-11-27)
- ✅ 初始版本
- ✅ 支持三个 ROS GUI 的启动/停止
- ✅ Web 界面管理
- ✅ 管理员权限控制
- ✅ 状态实时刷新

## 相关文档

- [PACKAGE_DOCUMENTATION.md](./PACKAGE_DOCUMENTATION.md) - ROS 包完整说明
- [TELEOPERATION_INTEGRATION_GUIDE.md](../TELEOPERATION_INTEGRATION_GUIDE.md) - 遥操作集成指南
- [qyh_jushen_web/README.md](../qyh_jushen_web/README.md) - Web 项目说明

## 联系方式

- **项目维护**: qyh <jsqinyinghao@live.com>
- **问题反馈**: GitHub Issues

---

**文档版本**: 1.0.0  
**更新时间**: 2025-11-27

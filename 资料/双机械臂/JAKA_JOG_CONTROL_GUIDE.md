# JAKA 机器人 Jog 点动控制接口说明

## 概述

本文档说明如何使用 JAKA 官方 SDK 实现 Web 界面中的手动点动(Jog)控制功能，包括：
- **关节控制**: J1-J7 各关节的独立控制
- **笛卡尔空间控制**: X/Y/Z 位置控制
- **姿态控制**: RX/RY/RZ 旋转控制

## SDK 接口定义

### jog - 点动运动

```cpp
/**
* @brief 控制机器人手动模式下运动
* @param handle    机器人控制句柄
* @param aj_num    1-based标识值，关节空间下代表关节号(1-7)，
*                  笛卡尔空间下依次为 x(1), y(2), z(3), rx(4), ry(5), rz(6)
* @param move_mode 运动模式：ABS(绝对), INCR(增量), CONTINUE(连续)
* @param coord_type 坐标系类型：COORD_BASE(基坐标系), COORD_JOINT(关节空间), COORD_TOOL(工具坐标系)
* @param vel_cmd   指令速度，关节/旋转轴单位rad/s，移动轴单位mm/s
* @param pos_cmd   指令位置，关节/旋转轴单位rad，移动轴单位mm (连续模式下此参数无效)
* @return ERR_SUCC 成功，其他失败
*/
errno_t jog(const JKHD *handle, int aj_num, MoveMode move_mode, 
            CoordType coord_type, double vel_cmd, double pos_cmd);
```

### jog_stop - 停止点动

```cpp
/**
* @brief 控制机器人手动模式下运动停止
* @param handle 机器人控制句柄
* @param num    要停止的轴/关节编号
* @return ERR_SUCC 成功，其他失败
*/
errno_t jog_stop(const JKHD *handle, int num);
```

## 枚举类型定义

### MoveMode - 运动模式

```cpp
typedef enum {
    ABS = 0,    // 绝对运动 - 移动到指定的绝对位置
    INCR,       // 增量运动 - 相对当前位置移动指定距离
    CONTINUE    // 连续运动 - 持续运动直到调用 jog_stop
} MoveMode;
```

### CoordType - 坐标系类型

```cpp
typedef enum {
    COORD_BASE,   // 基坐标系（世界坐标系/用户坐标系）
    COORD_JOINT,  // 关节空间
    COORD_TOOL    // 工具坐标系
} CoordType;
```

## 参数对照表

### aj_num 参数映射

| aj_num | 关节空间 (COORD_JOINT) | 笛卡尔空间 (COORD_BASE/COORD_TOOL) |
|:------:|:----------------------:|:---------------------------------:|
| 1 | J1 关节 | X 轴 |
| 2 | J2 关节 | Y 轴 |
| 3 | J3 关节 | Z 轴 |
| 4 | J4 关节 | RX (绕X轴旋转) |
| 5 | J5 关节 | RY (绕Y轴旋转) |
| 6 | J6 关节 | RZ (绕Z轴旋转) |
| 7 | J7 关节 | - |

### 速度单位

| 控制类型 | 单位 | 建议值 |
|:-------:|:----:|:------:|
| 关节运动 | rad/s | 0.1 ~ 1.0 |
| 直线运动 (X/Y/Z) | mm/s | 10 ~ 100 |
| 姿态运动 (RX/RY/RZ) | rad/s | 0.1 ~ 0.5 |

## 使用示例

### 1. 关节点动控制 (J1-J7)

#### 连续运动模式

```cpp
#include "jakaAPI.h"

JKHD handle;

// 连接机器人
create_handler("192.168.2.200", &handle);

// 使能机器人
power_on(&handle);
enable_robot(&handle);

// J1 正向连续运动，速度 0.1 rad/s (约 5.7°/s)
jog(&handle, 1, CONTINUE, COORD_JOINT, 0.1, 0);

// 运动一段时间后停止
sleep(500);  // 运动500ms
jog_stop(&handle, 1);

// J3 反向连续运动（负速度表示反向）
jog(&handle, 3, CONTINUE, COORD_JOINT, -0.1, 0);
sleep(500);
jog_stop(&handle, 3);
```

#### 步进运动模式

```cpp
// J2 正向步进 5 度
// 注意：角度需要转换为弧度
double step_rad = 5.0 * M_PI / 180.0;  // 5° = 0.0873 rad
jog(&handle, 2, INCR, COORD_JOINT, 0.1, step_rad);

// J4 反向步进 10 度
step_rad = 10.0 * M_PI / 180.0;
jog(&handle, 4, INCR, COORD_JOINT, -0.1, step_rad);
```

### 2. 笛卡尔空间点动 (X/Y/Z)

```cpp
// X轴 正向连续运动，速度 10 mm/s
jog(&handle, 1, CONTINUE, COORD_BASE, 10.0, 0);
sleep(500);
jog_stop(&handle, 1);

// Y轴 反向连续运动
jog(&handle, 2, CONTINUE, COORD_BASE, -10.0, 0);
sleep(500);
jog_stop(&handle, 2);

// Z轴 正向步进 5mm
jog(&handle, 3, INCR, COORD_BASE, 10.0, 5.0);
```

### 3. 姿态点动 (RX/RY/RZ)

```cpp
// RX 正向连续运动，速度 0.1 rad/s
jog(&handle, 4, CONTINUE, COORD_BASE, 0.1, 0);
sleep(500);
jog_stop(&handle, 4);

// RZ 反向连续运动
jog(&handle, 6, CONTINUE, COORD_BASE, -0.1, 0);
sleep(500);
jog_stop(&handle, 6);

// RY 步进 5 度
double step_rad = 5.0 * M_PI / 180.0;
jog(&handle, 5, INCR, COORD_BASE, 0.1, step_rad);
```

### 4. 使用工具坐标系

```cpp
// 在工具坐标系下 Z轴正向运动（沿工具方向前进）
jog(&handle, 3, CONTINUE, COORD_TOOL, 10.0, 0);
sleep(500);
jog_stop(&handle, 3);
```

## Web界面操作与SDK调用对应

| Web界面操作 | SDK调用 |
|:-----------|:--------|
| 按下 J1+ | `jog(&handle, 1, CONTINUE, COORD_JOINT, 0.1, 0)` |
| 松开 J1+ | `jog_stop(&handle, 1)` |
| 按下 J1- | `jog(&handle, 1, CONTINUE, COORD_JOINT, -0.1, 0)` |
| 松开 J1- | `jog_stop(&handle, 1)` |
| J1 步进 1° | `jog(&handle, 1, INCR, COORD_JOINT, 0.1, 0.01745)` |
| 按下 X+ | `jog(&handle, 1, CONTINUE, COORD_BASE, 10.0, 0)` |
| 松开 X+ | `jog_stop(&handle, 1)` |
| 按下 Z- | `jog(&handle, 3, CONTINUE, COORD_BASE, -10.0, 0)` |
| 松开 Z- | `jog_stop(&handle, 3)` |
| 按下 RZ+ | `jog(&handle, 6, CONTINUE, COORD_BASE, 0.1, 0)` |
| 松开 RZ+ | `jog_stop(&handle, 6)` |

## 速度控制

Web界面底部的速度滑块(1%-100%)对应 `set_rapidrate` 接口：

```cpp
// 设置运行倍率为 30% (对应界面速度滑块)
set_rapidrate(&handle, 0.3);

// 获取当前倍率
double rate;
get_rapidrate(&handle, &rate);
```

## 完整示例代码

```cpp
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "jakaAPI.h"

#define DEG2RAD(x) ((x) * M_PI / 180.0)

int main() {
    JKHD handle;
    errno_t ret;
    
    // 1. 连接机器人
    ret = create_handler("192.168.2.200", &handle);
    if (ret != ERR_SUCC) {
        printf("连接失败: %d\n", ret);
        return -1;
    }
    printf("连接成功\n");
    
    // 2. 上电使能
    power_on(&handle);
    usleep(1000000);  // 等待1秒
    enable_robot(&handle);
    usleep(500000);
    printf("机器人已使能\n");
    
    // 3. 设置速度倍率 30%
    set_rapidrate(&handle, 0.3);
    
    // 4. 关节点动示例 - J1正向运动1秒
    printf("J1 正向点动...\n");
    jog(&handle, 1, CONTINUE, COORD_JOINT, 0.1, 0);
    usleep(1000000);  // 运动1秒
    jog_stop(&handle, 1);
    printf("J1 停止\n");
    
    usleep(500000);  // 等待0.5秒
    
    // 5. 笛卡尔点动示例 - X轴正向运动1秒
    printf("X轴 正向点动...\n");
    jog(&handle, 1, CONTINUE, COORD_BASE, 10.0, 0);
    usleep(1000000);
    jog_stop(&handle, 1);
    printf("X轴 停止\n");
    
    usleep(500000);
    
    // 6. 步进运动示例 - J2步进5度
    printf("J2 步进 5 度...\n");
    jog(&handle, 2, INCR, COORD_JOINT, 0.1, DEG2RAD(5.0));
    usleep(2000000);  // 等待运动完成
    printf("J2 步进完成\n");
    
    // 7. 断开连接
    destory_handler(&handle);
    printf("断开连接\n");
    
    return 0;
}
```

## 注意事项

1. **安全第一**
   - 操作前确保机器人周围无障碍物和人员
   - 调试时使用低速度（建议 10% 以下）
   - 随时准备按急停按钮

2. **使能状态**
   - 调用 `jog` 前必须确保机器人已上电 (`power_on`) 并使能 (`enable_robot`)
   - 可通过 `get_robot_state` 检查当前状态

3. **速度方向**
   - 正速度 = 正向运动
   - 负速度 = 反向运动
   - 界面的 +/- 按钮通过速度符号控制方向

4. **运动模式选择**
   - `CONTINUE`: 适用于按住按钮持续运动的场景
   - `INCR`: 适用于每次点击移动固定距离的场景
   - `ABS`: 适用于移动到指定位置的场景

5. **坐标系选择**
   - `COORD_JOINT`: 控制单个关节，不影响其他关节
   - `COORD_BASE`: 末端在世界坐标系下移动
   - `COORD_TOOL`: 末端在工具坐标系下移动（沿工具方向）

## 相关接口

| 接口 | 功能 |
|:-----|:-----|
| `power_on` | 打开机器人电源 |
| `enable_robot` | 使能机器人 |
| `disable_robot` | 下使能机器人 |
| `get_robot_state` | 获取机器人状态 |
| `get_joint_position` | 获取当前关节位置 |
| `get_tcp_position` | 获取当前TCP位置 |
| `set_rapidrate` | 设置运行倍率 |
| `motion_abort` | 终止当前运动 |

## 更新日志

- **2024-12-03**: 创建文档，完整说明 jog/jog_stop 接口用法

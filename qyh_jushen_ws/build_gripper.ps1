# PowerShell script to build gripper packages

Write-Host "=========================================" -ForegroundColor Cyan
Write-Host "编译夹爪控制系统" -ForegroundColor Cyan
Write-Host "=========================================" -ForegroundColor Cyan

Set-Location "D:\work\yc\qyh_jushen_ws"

Write-Host ""
Write-Host "步骤1: 编译消息定义包..." -ForegroundColor Yellow
colcon build --packages-select qyh_gripper_msgs

Write-Host ""
Write-Host "步骤2: Source环境..." -ForegroundColor Yellow
. "install\setup.ps1"

Write-Host ""
Write-Host "步骤3: 编译控制节点包..." -ForegroundColor Yellow
colcon build --packages-select qyh_gripper_control

Write-Host ""
Write-Host "步骤4: 编译GUI包..." -ForegroundColor Yellow
colcon build --packages-select qyh_gripper_gui

Write-Host ""
Write-Host "=========================================" -ForegroundColor Green
Write-Host "编译完成！" -ForegroundColor Green
Write-Host "=========================================" -ForegroundColor Green
Write-Host ""
Write-Host "使用方法：" -ForegroundColor White
Write-Host "  1. 启动双手夹爪: ros2 launch qyh_gripper_control dual_gripper.launch.py" -ForegroundColor White
Write-Host "  2. 启动GUI:      ros2 run qyh_gripper_gui gripper_gui" -ForegroundColor White
Write-Host ""

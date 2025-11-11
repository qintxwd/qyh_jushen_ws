我们的wsl-ubuntu22.04系统需要安装ROS2 Humble版本，以下是安装步骤：
# 安装ROS2 Humble
参考官方文档：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
## Set locale
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
## setup sources
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```
## install ROS2 packages
```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
sudo apt install ros-dev-tools
```
## 设置环境变量
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
## 安装常用ROS2工具
```bash
sudo apt install -y \
  ros-humble-ros-base \          # 基础ROS2包
  ros-humble-rmw-cyclonedds-cpp \ # CycloneDDS中间件实现
  ros-humble-ros2-control \       # ROS2控制框架
  ros-humble-ros2-controllers \   # ROS2控制器包
  ros-humble-gazebo-ros-pkgs \    # Gazebo与ROS2集成包
  ros-humble-navigation2 \        # ROS2导航2包
  ros-humble-slam-toolbox \       # SLAM工具包
  ros-humble-image-transport \    # 图像传输包
  ros-humble-rviz2 \              # RViz2可视化工具
```
## 因为我们的项目是具身智能的，所以需要有机械臂相关的包，安装如下：
```bash
sudo apt install -y \
  ros-humble-moveit \             # MoveIt运动规划框架
  ros-humble-moveit-ros-planning \ # MoveIt ROS规划接口
  ros-humble-moveit-ros-move-group \ # MoveIt移动组接口
  ros-humble-moveit-ros-visualization \ # MoveIt可视化工具
  ros-humble-moveit-ros-perception \ # MoveIt感知模块
```
## 安装完成后，可以通过以下命令验证ROS2是否安装成功：
```bash
ros2 --version
```
```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```
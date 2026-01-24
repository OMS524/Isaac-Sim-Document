# ROS2 humble Installation
## Infomation
| 항목 | 내용 |
|-|-|
| Author | 오민석 |
| Date | 2026-01-24 |
| OS | Ubuntu 22.04 |
| GPU | NVIDIA RTX 6000 Ada Generation |
| Driver Version | 580.126.09 |
| CUDA Version | 13.0 |

## ROS2 설치
### 1. ROS2 humble 설치
locale 설정
```bash
locale

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

reboot

locale
```

Sources 설정
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

ROS2 packages 설치
```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
```

(선택 사항) vision_msgs_package 설치
```bash
sudo apt install ros-humble-vision-msgs
```

(선택 사항) ackermann_msgs_package 설치
```bash
sudo apt install ros-humble-ackermann-msgs
```

Environment 설정
```bash
source /opt/ros/humble/setup.bash
```

## 워크스페이스 설정
### 1. IsaacSim-ros_workspaces 다운
https://github.com/isaac-sim/IsaacSim-ros_workspaces
```bash
cd ~
git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git
```

### 2. 빌드하기 전 추가 패키지 설치
```bash
sudo apt install python3-rosdep build-essential
sudo apt install python3-colcon-common-extensions
```

### 3. 소싱
```bash
source /opt/ros/humble/setup.bash
```

### 4. 패키지 종속성 해결
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws
git submodule update --init --recursive # If using docker, perform this step outside the container and relaunch the container

sudo rosdep init
rosdep update

rosdep install -i --from-path src --rosdistro humble -y
```

### 5. 빌드
```bash
colcon build
```

### 6. 소싱
```bash
source install/local_setup.bash
```



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

## ROS2 브리지 활성화
### Fast DDS를 사용하는 Linux 환경에서
**워크스페이스에 fastdds.xml 존재하지 않을 경우**
해당 워크스페이스에 fastdds.xml 파일을 만들고,
다음 코드를 적용
```bash
<?xml version="1.0" encoding="UTF-8" ?>

<license>Copyright (c) 2022-2024, NVIDIA CORPORATION.  All rights reserved.
NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto.  Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.</license>


<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles" >
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UdpTransport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="udp_transport_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>UdpTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
```

**ROS2를 사용할 터미널에 Fast DDS 미들웨어를 설정 및 UDP 전송 활성화**
export FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_ros2_ws>/fastdds.xml
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
```

**Isaac Sim을 실행하기 전 ROS2 라이브러리 및 작업 공간 소싱**
```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

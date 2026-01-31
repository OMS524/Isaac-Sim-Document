# ROS2 Transform Trees and Odometry
## Environment Infomation
| Item | Description |
|-|-|
| Author | 오민석 |
| Date | 2026-01-25 |
| OS | Ubuntu 22.04 |
| GPU | NVIDIA RTX 6000 Ada Generation |
| Driver Version | 580.126.09 |
| CUDA Version | 13.0 |
| Isaac Sim Installation Type | Container |
| Container Runtime | Docker |
| Isaac Sim Version | 5.1.0 |
| ROS 2 Distribution | Humble |
| ROS 2 Installation | Native (Host) |
| DDS Implementation | Fast DDS |

## Getting Started
- **ROS 2 Cameras** 튜토리얼의 [rtx-lidar-sensors.md](/docs/ROS%202/ROS%202%20Tutorials%20(Linux%20and%20Windows)/RTX%20Lidar%20Sensors/rtx-lidar-sensors.md)를 열어주세요.

## Transform Tree Publisher
이미 ROS 2 카메라 튜토리얼을 검토하고 이미 두 대의 카메라가 무대에 있다고 가정하면, 그 카메라를 변환 트리에 추가하여 글로벌 프레임에서 카메라의 위치를 추적할 수 있도록 합시다.

### Transform Publisher
1. 새로운 Action Graph 만들어 다음과 같이 구성합니다.<br>

<img width="500" alt="image" src="https://github.com/user-attachments/assets/ff4c18c1-d5a7-4dc1-a99f-90c9c643f59f" /><br>

2. *ROS 2 Publish Transform Tree*의 Property 탭에서 targetPrims에 `Camera_1`과 `Camera_2`를 추가합니다.<br>
<img width="500" alt="image" src="https://github.com/user-attachments/assets/b3846bae-2570-493f-a60b-9dba1acba9a3" />

3. 새로운 터미널에서 다음 명령어를 통해 두 카메라가 모두 TF 트리에 있는지 확인합니다. 뷰포트에서 카메라를 움직여 카메라의 자세가 어떻게 변하는지 확인하세요.<br>
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
```bash
ros2 topic echo /tf
```
<img width="250" alt="image" src="https://github.com/user-attachments/assets/3f4a9b91-c032-4f71-976a-6954933ee406" />









## Setting Up Odometry
로봇의 odometry를 설정하려면 odometry ROS message와 해당 transforms을 publish합니다.




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

### Articulation Transforms
관절 로봇의 각 링크 변환을 얻으려면 로봇의 관절 루트를 ROS2 Publish Transform Tree 노드의 대상 Prims 필드에 추가할 수 있습니다. 관절 루트 이후의 모든 링크는 자동으로 게시됩니다.<br>

> [!IMPORTANT]
> 관절형 로봇에 대해 생성된 TF 트리가 잘못된 링크를 root link로 선택한 것을 발견하면 다음 단계를 통해 관절형 root link를 수동으로 선택합니다.<br>
> - Stage Tree의 **Raw USD Properties** 탭에서 로봇의 root prim을 선택하고 **Articulation Root** 섹션을 찾습니다. 섹션 내부 오른쪽 상단 모서리에 있는 X를 클릭하여 삭제합니다.
> - Stage Tree의 **Raw USD Properties** 탭에서 원하는 link를 선택하고 +ADD 버튼을 클릭한 다음 **Physics > Articulation Root**를 추가합니다.
> - Articulation Root를 변경한 후 파일을 저장하고 다시 로드합니다.

### Publish Relative Transforms




## Setting Up Odometry
로봇의 odometry를 설정하려면 odometry ROS message와 해당 transforms을 publish합니다.




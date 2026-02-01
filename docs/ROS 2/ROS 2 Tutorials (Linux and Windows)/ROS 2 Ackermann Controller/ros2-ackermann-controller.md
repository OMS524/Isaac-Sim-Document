# ROS 2 Ackermann Controller
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
해당 예제를 진행하기 위해 다음을 진행하세요.
- ackermann_msgs ROS 2 package가 필요합니다.<br>다음 명령어를 통해 설치하세요.
> ```bash
> sudo apt install ros-humble-ackermann-msgs
> ```
- Isaac Sim 내에서 **Window > Extensions**로 이동하여 `isaacsim.ros2.bridge`를 Enable하세요.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/0fa2211b-b9fa-4a93-aa73-92dfd93973b3" />
- ROS 2 워크스페이스에 `isaac_tutorials`, `cmdvel_to_ackermann` 패키지가 있어야 합니다.<br>없을 경우 [ROS 2 Humble Installation](/docs/Installation/ROS%202/ros2-humble-installation.md)를 참고하세요.

## Ackermann Controller and Drive Setup
1. **Create > Environments > Flat Grid** 
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/a18ac30d-793d-46d1-8f44-11d7f64d0e98" />
2. Content Browser에서 **Isaac Sim>ROBOTS>NVIDIA>Leatherback**를 Stage로 드래그하여 Leatherback 로봇을 불러오세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/0a6c3c72-bcfe-48fe-a741-d80812f32ed7" />
3. 생성된 Leatherback의 Property에서 위치를 `(0,0,0)`으로 설정하세요.
> <img width="500" height="90" alt="image" src="https://github.com/user-attachments/assets/81f0ef4f-d000-4880-98b5-96c428d6c5d0" />
4. **Window > Graph Editors > Action Graph**으로 이동하여 새로운 Action Graph를 생성 후 다음과 같이 구성하세요.
> <img width="887" height="442" alt="image" src="https://github.com/user-attachments/assets/07fc7975-ddb9-4037-9d51-4d1e44a64466" /><br>
> - **On Playback Tick** 노드에서 모든 시뮬레이션 프레임에서 다른 그래프 노드를 실행합니다.<br>
> - **ROS 2 Context** 노드에서 주어진 Domain ID 또는 `ROS_DOMAIN_ID` 환경 변수를 사용하여 컨텍스트를 생성합니다.<br>
> - **Ackermann Controller** 노드에서 wheel steering angles와 wheel speed를 계산합니다.<br>
> - **ROS 2 Subscribe AckermannDrive** 노드에서 AckermannDrive commands을 subscribe합니다.<br>
> - **ROS 2 QoS Profile** 노드를 통해 QoS profile을 생성합니다.<br>
> - **Articulation Controller** 노드에서 Leatherback의 steering joints를 조작합니다.<br>**Property** 탭에서 다음을 설정하세요:
> > - **targetPrim**에 `/leatherback`를 추가하세요.
> > - *jointNames*에 다음을 설정하세요:
> > > - **Add Element**를 클릭하고 나타나는 텍스트 필드에 `Knuckle__Upright__Front_Left`를 입력합니다.
> > > - **Add Element**를 클릭하고 표시되는 텍스트 필드에 `Knuckle__Upright__Front_Right`를 입력합니다.
> - **Articulation Controller_01** 노드에서 Leatherback의 바퀴를 조작합니다.<br>**Property** 탭에서 다음을 설정하세요:
> > - **targetPrim**에 `/leatherback`를 추가하세요.
> > - *jointNames*에 다음을 설정하세요:
> > > - **Add Element**를 클릭하고 나타나는 텍스트 필드에 `Wheel__Upright__Rear_Left`를 입력합니다.
> > > - **Add Element**를 클릭하고 표시되는 텍스트 필드에 `Wheel__Upright__Rear_Right`를 입력합니다.
> > > - **Add Element**를 클릭하고 표시되는 텍스트 필드에 `Wheel__Knuckle__Front_Left`를 입력합니다.
> > > - **Add Element**를 클릭하고 표시되는 텍스트 필드에 `Wheel__Knuckle__Front_Right`를 입력합니다.
> - **ROS 2 Subscribe AckermannDrive** 노드에서 Property 탭에서 다음을 설정하세요.
> > **topicName**에 `ackermann_cmd`로 설정하세요.
> - **Ackermann Controller** 노드에서 Property 탭에서 아래 표와 같이 설정하세요.
> > | Input Field | Value |
> > |:-|:-|
> > | backWheelRadius | 0.052 |
> > | frontWheelRadius | 0.052 |
> > | maxWheelRotation | 0.7854 |
> > | maxWheelVelocity | 20.0 |
> > | trackWidth | 0.24 |
> > | wheelBase | 0.32 |
> > | maxAcceleration | 1.0 |
> > | maxSteeringAngleVelocity | 1.0 |
5. **Play**를 눌러 시뮬레이션을 시작하세요.
6. 새로운 터미널에서 다음 명령어를 실행하여 Ackermann commands를 publish 하세요.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 run isaac_tutorials ros2_ackermann_publisher.py
> ```




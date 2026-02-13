# ROS 2 Generic Publisher and Subscriber
## Environment Infomation
| Item | Description |
|-|-|
| Author | 오민석 |
| Date | 2026-02-13 |
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

## ROS 2 Messages Types
ROS 2의 주요 communication interfaces 스타일 중 하나는 이 topic에 관한 것입니다. robot state`(nav_msgs/msg/Odometry)`, sensors`(예: sensor_msgs/msg/Imu)` 등과 같은 연속 데이터 스트림의 메시지를 송수신하는 데 사용됩니다.<br>
<br>
새로운 터미널에서 다음 명령을 사용하여 현재 소싱된 ROS 2(및 작업 공간)에 대한 사용 가능한 message types(topic)을 확인할 수 있습니다.<br>
> ```bash
> ros2 interface list --only-msgs
> ```

## Generic Publisher
> [!NOTE]
> 해당 **Generic Publisher** 부분은 예시가 아닌 방법을 설명합니다.

**Basic Methodology:**
1. **Window > Graph Editors > Action Graph**로 이동합니다.

2.  Action Graph를 새로 생성 후 다음과 같이 구성합니다.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/d4d0c3b2-e8a0-4bb8-abf0-0593e33453c9" /><br>

3. 노드 **Property** 패널에서 다음 `messagePackage / messageSubfolder / messageName`을 정의합니다. 유효한 (기존) message 유형이 정의되면 노드는 input attributes을 재구성하여 publish할 값을 설정합니다.
> [!NOTE]
> OmniGraph에서 노드 input attributes reconfiguration은 다음 규칙을 따릅니다:
> - ROS 2 유형 임베디드 메시지 필드(예: `std_msgs/Header header`)는 새로운 속성으로 롤아웃됩니다.
> - ROS 2 타입 임베디드 메시지 배열 필드(예: `geometry_msgs/Point32[] points`)는 타입 토큰 배열의 고유 속성으로 취급됩니다. 각 토큰은 JSON으로 인코딩됩니다.

> 다양한 message 유형에 대한 노드 input attributes reconfiguration의 예:<br>
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/b3079c00-e638-415a-a442-20941da1cdff" /><br>

4. 다른 노드의 출력을 연결하거나 **Property** 패널에서 values을 입력에 설정하여 publish할 데이터를 채웁니다.

5. **Play**를 클릭하여 시뮬레이션을 시작합니다.

## Example: Publish Joint States
다음 예제는 일반적인 ROS 2 message type `sensor_msgs/msgs/JointState`를 사용하여 로봇의 joint states(positions, velocities and efforts)를 `/joint_states`에 publish하는 방법을 보여줍니다.<br>

1. **Window > Examples > Robotics Examples**로 이동하여 **Import Robots > Franka URDF**를 클릭하고 **LOAD**를 클릭합니다.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/b0fcf44a-d879-47bf-8f71-788a122a8552" /><br>

2. **Window > Graph Editors > Action Graph**로 이동하여 새로운 Action Graph를 생성하고 다음과 같이 구성합니다.
> [!NOTE]
> **ROS2 Publisher**의 **Property**에서 `messagePackage`, `messageName`를 정의해야 노드의 인풋이 생성됩니다.

> <img width="500" alt="image" src="https://github.com/user-attachments/assets/1130e0dd-874b-40f6-a6ca-4337f5eccf70" /><br>
> | Node | Input field | Value |
> |-|-|-|
> | ROS2 Publisher | messagePackage | sensor_msgs |
> |  | messageName | JointState |
> |  | topicName | joint_states |
> | Articulation State | targetPrim | /panda |
> 
> **Isaac Time Splitter** 노드는 현재 simulation time을 분할하여 ROS 2 `std_msgs/Header header` 메시지 timestamp(seconds와 nanosecond로 표시)를 채웁니다.

3. **Play**를 클릭하여 시뮬레이션을 시작 후 **Robotics Examples**에서 **Import Robots > Franka URDF**를 클릭하고 **MOVE**를 클릭합니다.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/db3170b7-fd36-4b78-b769-9a76a7ad9713" /><br>

4. 새로운 터미널에서 다음 명령을 실행하여 published messages를 확인합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 topic echo /joint_states
> ```
> <img width="250" alt="image" src="https://github.com/user-attachments/assets/6b635abe-dc94-420b-b199-48c41f8bdd78" /><br>

## Example: Publish Object Pose
다음 예제는 일반적인 ROS 2 message type `sgeometry_msgs/msgs/Pose`를 사용하여 객체의 pose를 `/object_pose`에 publish하는 방법을 보여줍니다.<br>

1. **Create > Shape > Cube**를 클릭 후 `/World/Cube`를 클릭합니다.<br>오른쪽 클릭 후 **Add > Physics > Rigid Body with Colliders Preset**을 클릭합니다.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/0ae64ee2-a434-4356-9dfe-ad8402d96068" /><br>

2. **Window > Graph Editors > Action Graph**로 이동하여 새로운 Action Graph를 생성하고 다음과 같이 구성합니다.
> [!NOTE]
> **ROS2 Publisher**의 **Property**에서 `messagePackage`, `messageName`를 정의해야 노드의 인풋이 생성됩니다.

> <img width="500" height="465" alt="image" src="https://github.com/user-attachments/assets/4f876c78-e48c-470a-8da1-004a9e530f13" /><br>
> | Node | Input field | Value |
> |-|-|-|
> | ROS2 Publisher | messagePackage | geometry_msgs |
> |  | messageName | Pose |
> |  | topicName | object_pose |
> | Read Prim Attribute (upper node) | Prim | /World/Cube |
> |  | Attribute Name | xformOp:translate |
> | Read Prim Attribute (lower node) | Prim | /World/Cube |
> |  | Attribute Name | xformOp:orient |
> 
> **Isaac Time Splitter** 노드는 현재 simulation time을 분할하여 ROS 2 `std_msgs/Header header` 메시지 timestamp(seconds와 nanosecond로 표시)를 채웁니다.

3. **Play**를 클릭하여 시뮬레이션을 시작합니다.

4. 새로운 터미널에서 다음 명령을 실행하여 published messages를 확인합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 topic echo /object_pose
> ```
> <img width="250" alt="image" src="https://github.com/user-attachments/assets/aa49dc0e-a1b7-4d62-8664-8224b44c4312" /><br>

## Generic Subscriber
> [!NOTE]
> 해당 **Generic Publisher** 부분은 예시가 아닌 방법을 설명합니다.

**Basic Methodology:**
1. **Window > Graph Editors > Action Graph**로 이동하여 새로운 Action Graph를 생성 후 다음과 같이 구성합니다.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/65b018a3-a29d-4f05-8c63-52ff090c55af" /><br>

2. 노드 **Property** 패널에서 다음 `messagePackage / messageSubfolder / messageName`을 정의합니다. 유효한 (기존) message 유형이 정의되면 노드는 subscription values을 얻기 위해 output attributes을 재구성합니다.
> [!NOTE]
> OmniGraph에서 노드 input attributes reconfiguration은 다음 규칙을 따릅니다:
> - ROS 2 유형 임베디드 메시지 필드(예: `std_msgs/Header header`)는 새로운 속성으로 롤아웃됩니다.
> - ROS 2 타입 임베디드 메시지 배열 필드(예: `geometry_msgs/Point32[] points`)는 타입 토큰 배열의 고유 속성으로 취급됩니다. 각 토큰은 JSON으로 인코딩됩니다.

> 다양한 message 유형에 대한 노드 output attributes reconfiguration의 예:<br>
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/6a91487f-bb07-4e4e-83f4-977382cc5458" /><br>

4. 노드 출력을 다른 노드의 입력에 연결하여 수신된 데이터를 소비합니다.

5. **Play**를 클릭하여 시뮬레이션을 시작합니다.

## Example: Subscribe to Object Pose
다음 예제는 일반적인 ROS 2 message type `sgeometry_msgs/msgs/Pose`를 사용하여 객체를 수신된 Pose로 텔레포트하기 위해 `/object_pose`대한 Subscriber를 생성하는 방법을 보여줍니다.<br>

1. **Create > Shape > Cube**를 클릭하여 object를 생성합니다.

2. **Window > Graph Editors > Action Graph**로 이동하여 Action Graph를 생성 후 다음과 같이 구성합니다.
> [!NOTE]
> **ROS2 Subscriber**의 **Property**에서 `messagePackage`, `messageName`를 정의해야 노드의 인풋이 생성됩니다.

> <img width="500" alt="image" src="https://github.com/user-attachments/assets/070a6dc8-5390-4144-9caa-25e0c4597964" /><br>
> | Node | Input field | Value |
> |-|-|-|
> | ROS2 Subscriber | messagePackage | geometry_msgs |
> |  | messageName | Pose |
> |  | topicName | object_pose |
> | Read Prim Attribute (upper node) | Prim | /World/Cube |
> |  | Attribute Name | xformOp:translate |
> | Read Prim Attribute (lower node) | Prim | /World/Cube |
> |  | Attribute Name | xformOp:orient |
>
> **Write Prim Attribute** 노드는 특정 Prim attribute values을 설정합니다.
> **Make 3-Vector** 노드는 개별 구성 요소로부터 three-component vector를 만듭니다.

3. **Play**를 클릭하여 시뮬레이션을 시작합니다.

4. 새로운 터미널에서 다음 명령을 실행하여 Isaac Sim의 객체를 지정된 pose로 텔레포트합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 topic pub -1 /object_pose geometry_msgs/msg/Pose "{position: {x: 1, y: 2, z: 3}, orientation: {x: 0.4619398, y: 0.1913417, z: 0.4619398, w: 0.7325378}}"
> ```
> [ros2_generic_publisher_and_subscriber.webm](https://github.com/user-attachments/assets/4adeb491-b8a8-4f5f-83ff-10693dbb0749)























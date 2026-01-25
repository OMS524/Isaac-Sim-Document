# ROS 2 Clock
## Environment Infomation
| Item | Description |
|-|-|
| Author | 오민석 |
| Date | 2026-01-24 |
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

## Simulation Time and Clock
외부 ROS 2 노드가 시뮬레이션 시간과 동기화되는 경우 일반적으로 clock 토픽이 사용됩니다.

RViz2와 같은 많은 ROS 2 노드는 매개변수 use_sim_time을 사용하며,

True로 설정하면 노드가 /clock 토픽에 가입하고 게시된 시뮬레이션 시간과 동기화를 시작하도록 표시합니다.

이 매개변수를 ROS 2 실행 파일에서 설정하거나 새 ROS 2 소스 터미널에서 다음 명령을 사용하여 매개변수를 설정할 수 있습니다:
```bash
ros2 param set /node_name use_sim_time true
```

현재 실행 중인 노드로 /node_name을 교체해야 합니다.

터미널을 사용하여 설정하는 경우 매개변수를 설정하기 전에 노드가 이미 먼저 실행 중이어야 합니다.

## Running ROS 2 Clock Publisher
1. Window > Graph Editors > Action Graph로 가서 Action graph를 생성하세요.
2. 다음 사진과 같이 Action graph에 OmniGraph를 추가하세요.
<img width="678" height="512" alt="image" src="https://github.com/user-attachments/assets/c4dba303-0a9b-456d-9210-25f65b68100c" />

- **On Playback Tick**: 모든 시뮬레이션 프레임에서 다른 그래프 노드를 실행합니다.
- **ROS 2 Context**: 주어진 도메인 ID 또는 ROS_DOMAIN_ID 환경 변수를 사용하여 컨텍스트를 생성합니다.
- **Isaac Read Simulation Time**: 현재 시뮬레이션 시간을 가져옵니다. 참고: 기본적으로 시뮬레이션 시간은 단조롭게 증가하므로 시뮬레이션이 중지되었다가 다시 재생되더라도 시간은 계속 증가합니다. 이는 주로 시뮬레이션이 초기화될 때 시간이 뒤로 이동할 때 발생할 수 있는 문제를 방지하기 위한 것입니다. 시뮬레이션이 초기화될 때마다 시계를 0에서 시작하도록 하려면 resetOnStop을 True로 설정할 수 있습니다.
- **ROS 2 Publish Clock**: /clock 토픽에 시뮬레이션 시간을 publish 합니다.

3. 새로운 ROS 2 소스 터미널에서 RViz를 시작합니다.
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 run rviz2 rviz2
```

4. 새로운 ROS 2 소스 터미널에서 RViz 노드에 대해 use_sim_time 매개변수를 true로 설정하세요. Isaac Sim에서 시뮬레이션이 중지되었는지 확인합니다.
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 param set /rviz use_sim_time true
```

5. Isaac Sim에서 Play를 클릭하세요.

[ROS 2 Clock_1.webm](https://github.com/user-attachments/assets/4ba2aac8-1bd1-4085-84c7-f700689a004e)

## Publishing System Time
시뮬레이션 시간을 게시하는 것이 가장 일반적인 워크플로이지만, 특정 메시지에 시스템 시간이 포함되어야 하는 특정 워크플로가 있을 수 있습니다.

클럭 주제에 대한 시스템 시간을 게시하려면 다음 단계를 따릅니다:

1. Window > Graph Editors > Action Graph로 가서 Action graph를 생성하세요.
2. 다음 사진과 같이 Action graph에 OmniGraph를 추가하세요.
<img width="584" height="485" alt="image" src="https://github.com/user-attachments/assets/e7f2faf7-600a-4b22-b35b-bf4518ba914b" />

- **On Playback Tick**: 모든 시뮬레이션 프레임에서 다른 그래프 노드를 실행합니다.
- **ROS 2 Context**: 주어진 도메인 ID 또는 ROS_DOMAIN_ID 환경 변수를 사용하여 컨텍스트를 생성합니다.
- **Isaac Read System Time**: 현재 시스템 시간을 검색합니다.
- **ROS 2 Publish Clock**: /clock 토픽에 시뮬레이션 시간을 publish 합니다.

3. Isaac Sim에서 재생을 클릭합니다. /clock 주제에 대해 Isaac Sim에서 게시된 시스템 타임스탬프를 관찰하려면 ROS 소스 터미널에서 다음 명령을 실행합니다.
```bash
ros2 topic echo /clock
```
[ROS 2 Clock_2.webm](https://github.com/user-attachments/assets/19fc05cc-82f8-4d17-a79e-2fb402c14783)

**Camera Helper and RTX Lidar nodes**
다가오는 튜토리얼에서는 ROS 2 카메라 헬퍼 노드와 ROS 2 RTX 라이다 헬퍼 노드를 관찰할 것입니다. 이 두 노드 모두 센서 퍼블리싱 파이프라인을 자동으로 생성하므로 퍼블리셔의 시스템 타임스탬프를 사용하려면 useSystemTime 입력 필드가 True로 설정되어 있는지 확인하세요.

## Running ROS 2 Clock Subscriber
1. Window > Graph Editors > Action Graph로 가서 Action graph를 생성하세요.
2. 다음 사진과 같이 Action graph에 OmniGraph를 추가하세요.
<img width="614" height="342" alt="image" src="https://github.com/user-attachments/assets/51b893f7-d3e1-44c2-93ea-a509a96c5883" />

3. Play를 클릭하여 시뮬레이션을 시작합니다. Action graph 내에서 ROS2 구독 시계 노드를 선택하여 Property 창에서 타임스탬프 출력을 확인합니다. 타임스탬프가 0인지 확인합니다.
<img width="511" height="268" alt="image" src="https://github.com/user-attachments/assets/55e29ce1-a0aa-4e89-ac03-9be1fbd7a203" />

4. 새로운 ROS2 소스 터미널에서 다음 명령을 실행하여 시계 메시지를 수동으로 한 번 게시합니다.
```bash
ros2 topic pub  -t 1 /clock rosgraph_msgs/Clock "clock: { sec: 0, nanosec: 0 }"
```
ROS2 Subscribe Clock OmniGraph 노드의 timeStamp 값이 1.2로 변경되는지 확인합니다.

5. 이전 명령을 다른 sec 및 nanosec 값으로 변경하여 ROS2 Subscribe Clock OmniGraph 노드의 timeStamp 필드에 반영된 값을 관찰합니다.
```bash
ros2 topic pub  -t 1 /clock rosgraph_msgs/Clock "clock: { sec: 1, nanosec: 200000000 }"
ros2 topic pub  -t 1 /clock rosgraph_msgs/Clock "clock: { sec: 0, nanosec: 0 }"
ros2 topic pub  -t 1 /clock rosgraph_msgs/Clock "clock: { sec: 5, nanosec: 500000000 }"
```
[ROS 2 Clock_3.webm](https://github.com/user-attachments/assets/429cce56-4e22-477c-9dc7-5dea20abc20f)

## Graph Shortcut
몇 번의 클릭만으로 시계 그래프를 만들 수 있는 메뉴 바로가기를 제공합니다.

| Step | Screenshot | Path / Description |
|------|-----------|--------------------|
| 1 | <img src="https://github.com/user-attachments/assets/b4c854a6-8648-4dcf-ac9d-4e53be4a3fa4" width="300"/> | **Tools > Robotics > ROS 2 OmniGraphs > Clock**로 이동합니다. 나열된 ROS2 그래프가 보이지 않는 경우 ROS2 브리지를 활성화해야 합니다. |
| 2 | <img src="https://github.com/user-attachments/assets/5a1361a3-2c5e-44f5-813a-38882db433ae" width="300"/> | 그래프를 채우는 데 필요한 매개변수를 묻는 팝업 상자가 표시됩니다. 그래프 경로를 제공하고 확인을 클릭합니다. |
| 3 | <img src="https://github.com/user-attachments/assets/c1a1ca60-32df-48da-b7b1-2e636f530862" width="300"/> | Action Graph 창에서 ROS_Clock을 선택하여 시뮬레이션된 시계를 게시하는 그래프가 나타나는지 확인합니다. |









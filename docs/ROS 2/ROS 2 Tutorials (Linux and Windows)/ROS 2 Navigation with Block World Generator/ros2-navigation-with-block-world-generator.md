# ROS 2 Navigation with Block World Generator
## Environment Infomation
| Item | Description |
|-|-|
| Author | 오민석 |
| Date | 2026-02-09 |
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

## Setting Up Environment and Robot
### Generate 3D World
먼저, Isaac Sim 내의 [Block World Generator](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/digital_twin/ext_isaacsim_asset_generator_occupancy_map.html#ext-omni-isaac-block-world-tool)를 사용하여 3D world를 로드해 보겠습니다.<br>
<br>
1. 상단 메뉴 바로 이동하여 **Tools > Robotics > Block World Generator**를 클릭합니다.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/703def4b-6b45-41c6-a91c-805e832a7630" /><br>

2. **Load Image**를 누르고 `carter_navigation/maps` 경로에 `carter_warehouse_navigation.png`을 엽니다.<br>**Visualization**라는 창이 나타납니다.
> <img width="500" height="555" alt="image" src="https://github.com/user-attachments/assets/62520b73-4b29-4ad5-8597-d7851d76478a" /><br>
> <img width="200" height="278" alt="image" src="https://github.com/user-attachments/assets/7a6700d8-b63b-4dd3-a58d-d104fa3e0d6d" /><br>

3. **Generate**를 눌러 Stage에서 입력 occupancy map에 해당하는 geometry를 생성합니다.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/d3ed7f44-610b-4647-8734-dc7a64449405" /><br>

<br>
생성된 3D world는 모든 occupied pixel에 자동으로 collision mesh가 적용됩니다.

### Add Robot in Scene
이 scene에 모든 ROS 2 OmniGraph Node가 설정된 Carter robot을 추가합니다.<br>
<br>

1. Content browser에서 **Isaac Sim>Samples>ROS2>Robots**에서 `Nova_Carter_ROS.usd`를 Stage로 드래그합니다.
> <img width="1767" height="986" alt="image" src="https://github.com/user-attachments/assets/7dcd7587-f053-4c4d-aec1-d0aea3be6a20" /><br>

### Add Clock in Scene
모든 외부 ROS 2 node가 simulation time을 참조하도록 하려면 `/clock` topic에 simulation time을 publish하는 역할을 하는 `Ros2PublishClock` 노드가 포함된 `ROS_Clock` 그래프를 추가해야 합니다.

1. **Tools > Robotics > ROS 2 OmniGraphs > Clock**를 클릭하여 그래프를 추가합니다.
> <img width="200" alt="image" src="https://github.com/user-attachments/assets/32d6ef9f-97dc-4755-aa12-23a6e35be1d0" /><br>
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/1e7270ad-33bd-4192-b2ed-3a56470aff55" /><br>

### Running Navigation
이제 3D scene과 로봇이 Nav2 stack을 실행하도록 설정되었습니다.

1. **Play**를 클릭하여 시뮬레이션을 시작합니다.

2. 새로운 터미널에서 다음 명령을 실행하여 Nav2를 실행합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch carter_navigation carter_navigation.launch.py
> ```

3. RViz2에서 **2D Pose Estimate**를 사용하여 로봇의 위치를 재설정합니다.

4. RViz2에서 Nav2 Goal을 클릭한 다음 원하는 위치를 클릭합니다.
> [ros2_navigation_with_block_world_generator.webm](https://github.com/user-attachments/assets/8b82fc10-cd59-4e29-892b-f7afb1b4c488)













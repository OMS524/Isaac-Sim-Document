# ROS 2 Navigation
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

## Nav2 Setup
이 시나리오에서 Nav2에 publish되는 topic와 message 유형은 다음과 같습니다:
| ROS2 Topic | ROS2 Message Type |
|-|-|
| /tf | tf2_msgs/TFMessage |
| /odom | nav_msgs/Odometry |
| /map | nav_msgs/OccupancyGrid |
| /point_cloud | sensor_msgs/PointCloud |
| /scan | sensor_msgs/LaserScan (published by an external pointcloud_to_laserscan node) |

### Occupancy Map
이 시나리오에서는 occupancy map가 필요합니다. 이 샘플의 경우 NVIDIA Isaac Sim 내의 [Occupancy Map Generator extension](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/digital_twin/ext_isaacsim_asset_generator_occupancy_map.html#ext-isaacsim-asset-generator-occupancy-map)을 사용하여 창고 환경의 occupancy map를 생성하고 있습니다.<br>
<br>

1. **Window > Examples > Robotics Examples**로 이동하세요.<br>**Robotics Examples** 탭을 클릭하세요.<br>**ROS2 > Navigation > Nova Carter**를 누르고 우측에서 **Load Sample Scene**을 클릭하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/5dea56d2-de9e-4599-896a-14e4a8dc1f91" /><br>
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/04346da6-8b05-46e8-8a4d-cd43380244c8" /><br>

> [!NOTE]
> Scene을 불러올 때 스트리밍 클라이언트가 멈출 시 스트리밍 클라이언트를 재실행하세요.

2. viewport에서 왼쪽 상단에 **Camera**를 클릭합니다.<br>**Top**을 클릭합니다.
> <img width="200" alt="image" src="https://github.com/user-attachments/assets/ad8ebc7e-1bac-48bd-b9bc-53411b0e5770" /><br>
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/66eb5a43-4b6a-44eb-958a-2e1f57ebfa4b" /><br>

3. **Tools > Robotics > Occupancy Map**을 클릭합니다.
> <img width="400" alt="image" src="https://github.com/user-attachments/assets/2516cb17-c758-4687-a17e-0cec9dadd219" /><br>

4. Occupancy Map extension에서 `Origin`이 `X: 0.0, Y: 0.0, Z: 0.0`으로 설정합니다.<br>`Lower bound`의 경우 `Z: 0.1`으로 설정합니다.<br>`Upper Bound`의 경우 ``Z: 0.62`으로 설정합니다.<br>upper bound Z distance는 ground에 대한 Lidar onboard Nova Carter의 vertical distance와 일치하도록 0.62미터로 설정되어 있다는 점을 명심하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/9b285411-fe13-4c55-99de-c5acda8815e7" /><br>

5. Stage에서 **warehouse_with_forklifts**을 클릭합니다.<br>Occupancy Map extension에서 **BOUND SELECTION**을 클릭합니다.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/c8be2019-db76-44c3-b94a-e64801b49392" /><br>

6. Stage에서 `Nova_Carter_ROS`를 삭제합니다.

7. Occupancy Map extension에서 **CALCULATE**를 클릭하고 **VISUALIZE IMAGE**를 클릭합니다.<br>Visualization popup이 표시됩니다.
> <img width="400" alt="image" src="https://github.com/user-attachments/assets/5add0725-1851-447e-8dc7-a56c76400aa0" /><br>

8. **Rotate Image**에서 180도를 선택합니다.<br>**Coordinate Type**에서 **ROS Occupancy Map Parameters File (YAML)**으로 선택합니다.<br>**RE-GENERATE IMAGE**를 클릭합니다.

9. Occupancy map parameters YAML 형식으로 아래 필드에 표시됩니다. 전체 텍스트를 복사하세요.<br>`~/IsaacSim-ros_workspaces/humble_ws/src/navigation/carter_navigation/maps` 경로에 `carter_warehouse_navigation.yaml`을 새로 생성하여 복사한 텍스트를 넣고 저장합니다.
> <img width="400" alt="image" src="https://github.com/user-attachments/assets/a5887507-78c1-4a66-a71a-0c2b9d964749" /><br>
>
> ```text
> image: carter_warehouse_navigation.png
> resolution: 0.05
> origin: [-11.975, -17.975, 0.0000]
> negate: 0
> occupied_thresh: 0.65
> free_thresh: 0.196
> ```

10. Visualization popup에서 **Save Image**를 클릭합니다.<br>`~/IsaacSim-ros_workspaces/humble_ws/src/navigation/carter_navigation/maps` 경로에 `carter_warehouse_navigation.png`라는 이름으로 저장하세요.
> <img width="400" alt="carter_warehouse_navigation" src="https://github.com/user-attachments/assets/6e5dc0c9-314c-45b7-8f0f-3e1d094b128c" /><br>

## Running Nav2
### Nav2 with Nova Carter in a Small Warehouse
1. **Window > Examples > Robotics Examples**로 이동하세요.<br>**Robotics Examples** 탭을 클릭하세요.<br>**ROS2 > Navigation > Nova Carter**를 누르고 우측에서 **Load Sample Scene**을 클릭하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/04346da6-8b05-46e8-8a4d-cd43380244c8" /><br>

2. **Play**를 클릭하여 시뮬레이션을 시작합니다.

3. 새로운 터미널에서 다음 명령을 실행합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch carter_navigation carter_navigation.launch.py
> ```
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/eb6b8eaa-eb93-4b1c-bdc8-3aa79dd3ce18" /><br>

4. RViz2에서 **Nav2 Goal**을 클릭한 다음 원하는 위치를 클릭합니다.
> [ros2_navigation_1.webm](https://github.com/user-attachments/assets/b0d83cac-09fc-4733-bf3f-02fa2d7122da)

> [!NOTE]
> Carter robot은 기본적으로 RTX Lidar를 사용합니다. 

### Nav2 with Nova Carter with Robot Description in a Small Warehouse
1. 다음 명령을 실행하여 **Nova Carter Description Package**를 설치합니다.
> [!NOTE]
> 해당 설치 내용은 Linux 환경에 ROS 2 Humble에서만 지원됩니다.

locale을 설정합니다 (locale 설정은 ROS 2 Humble 설치 시 이미 설정된 것이기에 넘어가셔도 됩니다.)
> ```bash
> locale
> 
> sudo apt update && sudo apt install locales
> sudo locale-gen en_US en_US.UTF-8
> sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
> export LANG=en_US.UTF-8
> 
> locale
> ```
종속성 설치
> ```bash
> sudo apt update && sudo apt install gnupg wget
> sudo apt install software-properties-common
> sudo add-apt-repository universe
> ```
NVIDIA의 GPG 키 및 저장소 등록
> ```bash
> wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
> grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
> echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | sudo tee -a /etc/apt/sources.list
> sudo apt-get update
> ```
nova_carter_description package 설치
> ```bash
> sudo apt install ros-humble-nova-carter-description
> ```

2. 새로운 터미널에서 다음 명령을 실행하여 Nova Carter description을 실행합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch carter_navigation nova_carter_description_isaac_sim.launch.py
> ```

3. **Window > Examples > Robotics Examples**로 이동하세요.<br>**Robotics Examples** 탭을 클릭하세요.<br>**ROS2 > Navigation > Nova Carter**를 누르고 우측에서 **Load Sample Scene**을 클릭하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/04346da6-8b05-46e8-8a4d-cd43380244c8" /><br>

4. **Play**를 클릭하여 시뮬레이션을 시작합니다.

5. 새로운 터미널에서 다음 명령을 실행하여 Nav2을 실행합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch carter_navigation carter_navigation.launch.py
> ```

6. RViz2에서 **Nav2 Goal**을 클릭한 다음 원하는 위치를 클릭합니다.
> [ros2_navigation_2.webm](https://github.com/user-attachments/assets/73b460d2-11a7-4881-aab2-83d5472e7f53)

### Nav2 with Nova Carter with robot_state_publisher in a Small Warehouse
1. **Window > Examples > Robotics Examples**로 이동하세요.<br>**Robotics Examples** 탭을 클릭하세요.<br>**ROS2 > Navigation > Nova Carter Joint States**를 누르고 우측에서 **Load Sample Scene**을 클릭하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/4f552d02-7914-4701-b353-8db04a26107f" /><br>

2. **Play**를 클릭하여 시뮬레이션을 시작합니다.

3. 새로운 터미널에서 다음 명령을 실행하여 Nova Carter description을 실행합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch carter_navigation nova_carter_description_isaac_sim.launch.py
> ```

4. 새로운 터미널에서 다음 명령을 실행하여 Nav2을 실행합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch carter_navigation carter_navigation.launch.py
> ```

5. RViz2에서 **Nav2 Goal**을 클릭한 다음 원하는 위치를 클릭합니다.
> [ros2_navigation_3.webm](https://github.com/user-attachments/assets/1b61cbb0-cebc-441f-9402-5e6e381fefab)

### Nav2 with iw.hub in Warehouse
1. **Window > Examples > Robotics Examples**로 이동하세요.<br>**Robotics Examples** 탭을 클릭하세요.<br>**ROS2 > Navigation > iw_hub**를 누르고 우측에서 **Load Sample Scene**을 클릭하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/f5f57f72-55c7-4d25-9259-5de3a2ce72b3" /><br>

2. **Play**를 클릭하여 시뮬레이션을 시작합니다.

3. 새로운 터미널에서 다음 명령을 실행하여 Nav2을 실행합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch iw_hub_navigation iw_hub_navigation.launch.py
> ```

4. RViz2에서 **Nav2 Goal**을 클릭한 다음 원하는 위치를 클릭합니다.
> [ros2_navigation_4.webm](https://github.com/user-attachments/assets/220b28fc-db40-48e8-82f6-a2a5b5fd8dc9)

## Sending Goals Programmatically
> [!NOTE]
> `isaac_ros_navigation_goal` 패키지는 Linux에서 완벽하게 지원됩니다.<br>
> Window에서는 오류가 발생할 수 있습니다.
ROS2 `isaac_ros_navigation_goal` 패키지는 파이썬 노드를 사용하여 로봇의 Goal Pose를 설정할 수 있습니다.<br>
이 패키지는 Goal Pose를 무작위로 생성하여 Nav2로 전송할 수 있습니다.<br>
필요한 경우 사용자 정의 Goal Pose를 전송할 수도 있습니다.<br>
<br>

1. 필요에 따라 실행 파일에 정의된 parameters를 변경하십시오
> parameters는 아래에 설명되어 있습니다:
> - **gal_generator_type**: 목표 생성기의 유형입니다. `RandomGoalGenerator`를 사용하여 무작위로 목표를 생성하거나 사용자 정의 목표를 특정 순서로 전송할 때 `GoalReader`를 사용합니다.
> - **map_yaml_path**: occupancy map parameters YAML 파일의 경로입니다. 예제 파일은 `isaac_ros_navigation_goal/assets/carter_warehouse_navigation.yaml`에 있습니다. 맵 이미지는 생성된 포즈 주변의 장애물을 식별하는 데 사용되고 있습니다. goal generator type이 `RandomGoalGenerator`로 설정된 경우 필수입니다.
> - **iteration_count**: goal 설정 횟수.
> - **action_server_name**: action server의 이름.
> - **obstacle_search_distance_in_meters**: 생성된 포즈가 어떤 종류의 장애물도 없는 Distance(미터 단위).
> - **gal_text_file_path**: 사용자 정의 정적 목표를 포함하는 텍스트 파일의 경로입니다. 파일의 각 줄에는 다음과 같은 형식의 단일 goal pose가 있습니다: `pose.x pose.y orientation.x orientation.y orientation.z orientation.w`. 샘플 파일은 다음 위치에 있습니다: `isaac_ros_navigation_goal/assets/goals.txt`. goal generator type이 GoalReader로 설정된 경우 필수입니다.
> - **initial_pose**: initial_pose가 설정되면 /initialpose 토픽에 publish되고 그 후 goal pose가 action server로 전송됩니다. 형식은 `[pose.x, pose.y, pose.z, orientation.x, orientation.y, orientation.z, orientation.w]`입니다.

2. **Window > Examples > Robotics Examples**로 이동하세요.<br>**Robotics Examples** 탭을 클릭하세요.<br>**ROS2 > Navigation > Nova Carter**를 누르고 우측에서 **Load Sample Scene**을 클릭하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/04346da6-8b05-46e8-8a4d-cd43380244c8" /><br>

3. **Play**를 클릭하여 시뮬레이션을 시작합니다.

4. 새로운 터미널에서 다음 명령을 실행하여 Nav2을 실행합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch carter_navigation carter_navigation.launch.py
> ```

5. 새로운 터미널에서 다음 명령을 실행하여 `isaac_ros_navigation_goal`을 실행합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch isaac_ros_navigation_goal isaac_ros_navigation_goal.launch.py
> ```
> [ros2_navigation_5.webm](https://github.com/user-attachments/assets/e012b42d-218c-4b86-abe6-5e8c1a908927)

> [!NOTE]
> 다음 조건 중 하나가 충족되면 패키지는 처리(목표 설정)를 중지합니다:<br>
> 지금까지 publish된 goal 수 >= iteration_count.
> `GoalReader` parameter가 사용되고 구성 파일의 모든 goal가 publish된 경우.
> goal가 action server에 의해 거부됩니다.
> 드물게 매우 조밀한 맵으로 인해 `RandomGoalGenerator`가 잘못된 포즈를 생성할 수 있습니다. `RandomGoalGenerator`가 생성한 잘못된 포즈의 수가 최대 반복 횟수를 초과하면 패키지가 처리를 중지합니다.

## Sending Goals Using ActionGraph
> [!IMPORTANT]
> Nav2가 설치되어 있어야 합니다.
> 설치되어 있지 않는 경우 다음을 수행하세요.
> > ```bash
> > sudo apt install ros-humble-navigation2
> > sudo apt install ros-humble-nav2-bringup
> > ```

1. **Window > Examples > Robotics Examples**로 이동하세요.<br>**Robotics Examples** 탭을 클릭하세요.<br>**ROS2 > Navigation > Nova Carter**를 누르고 우측에서 **Load Sample Scene**을 클릭하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/04346da6-8b05-46e8-8a4d-cd43380244c8" /><br>

2. **Robotics Examples > ROS2 > Navigation > Add Waypoint Follower**에서 필요에 따라 parameter를 변경합니다.
> <img width="100" alt="image" src="https://github.com/user-attachments/assets/78c00ed6-7309-4721-9936-c9ec9bdf7f4b" /><br>
>
> parameter는 아래에 설명되어 있습니다:
> - **Graph Path**: 단계 내의 경로를 지정합니다.
> - **Frame ID**: 탐색 작업의 참조 프레임을 지정합니다.
> - **Navigation Modes**:
>   - **Waypoint Mode**: 내비게이션 목표로 보낼 단일 웨이포인트를 만듭니다. 로봇이 이 웨이포인트를 향해 내비게이션합니다.
>   - **Patrolling Mode**: 연속 순찰을 위해 여러 경유지(2~50개 포함)를 생성합니다. 로봇은 이러한 미리 정의된 경유지 사이를 지속적으로 탐색합니다.
> - **Waypoint Count**: 순찰을 위해 생성할 웨이포인트 수입니다.

3. **Load Waypoint Follower ActionGraph**을 클릭하여 waypoints를 생성하고 **Graph Path** 경로에 action graph를 추가합니다.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/15caeb79-07dd-4408-8ba6-db8f38a0bcb9" /><br>

4. **Play**를 클릭하여 시뮬레이션을 시작합니다.

5. 새로운 터미널에서 다음 명령을 실행하여 Nav2을 실행합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch carter_navigation carter_navigation.launch.py
> ```

6. `ROS_Nav2_Waypoint_Follower` Graph에서 `OnImpulseEvent` 노드에서 **Send Impulse**를 클릭하여 navigation을 실행합니다.
> [ros2_navigation_6.webm](https://github.com/user-attachments/assets/79910199-af2e-428c-83ea-38d2c01b9d70)














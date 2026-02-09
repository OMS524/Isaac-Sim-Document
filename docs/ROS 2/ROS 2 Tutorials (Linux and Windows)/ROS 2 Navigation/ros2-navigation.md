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
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/5dea56d2-de9e-4599-896a-14e4a8dc1f91" /><br>
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
```bash
locale

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale
```
종속성 설치
```bash
sudo apt update && sudo apt install gnupg wget
sudo apt install software-properties-common
sudo add-apt-repository universe
```
NVIDIA의 GPG 키 및 저장소 등록
```bash
wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | sudo tee -a /etc/apt/sources.list
sudo apt-get update
```
nova_carter_description package 설치
```bash
sudo apt install ros-humble-nova-carter-description
```
















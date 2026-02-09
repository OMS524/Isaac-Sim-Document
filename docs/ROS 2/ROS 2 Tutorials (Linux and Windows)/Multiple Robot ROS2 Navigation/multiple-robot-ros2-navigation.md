# Multiple Robot ROS2 Navigation
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

## Getting Started
### Occupancy Map
NVIDIA Isaac Sim 내의 Occupancy Map Generator extension을 사용하여 Hospital Environment 및 Office Environment 모두의 지도를 생성합니다.<br>
<br>

> [!NOTE]
> 해당 내용에서는 Hospital Environment 환경으로 진행하였습니다.
> Office Environment로 진행하고 싶으시면 [Multiple Robot ROS2 Navigation Occupancy Map](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_multi_navigation.html#occupancy-map)을 참고하세요.

> 1. Content browser에서 **Isaac Sim>Environments>Hospital**에서 **hospital.usd**를 Stage로 드래그 합니다.<br>불러온 asset의 Translate을 0으로 설정합니다.<br>Stage에서 `/hospital` prim을 선택하고 **F**를 눌러 확대합니다.
> > <img width="1000" alt="image" src="https://github.com/user-attachments/assets/1a89c97a-139b-4aab-9d70-c4efab09fa1e" />
> 
> 2. viewport 상단 왼쪽에 **Perspective**를 클릭합니다.<br>**Top**을 클릭합니다.
> > <img width="200" alt="image" src="https://github.com/user-attachments/assets/58b9d860-65d3-4af3-9c27-08198e43ca24" />
> 
> 3. **Tools > Robotics > Occupancy Map**을 엽니다.<br>Origin을 `X: 0.0, Y: 0.0, Z: 0.0`으로 설정합니다.<br>Lower bound에서 `Z: 0.1`로 설정합니다.<br>Upper Bound에서 `Z: 0.62`로 설정합니다.
> 
> 4. Stage에서 `/hospital` prim을 선택하고 Occupancy Map extension에서 **BOUND SELECTION**를 클릭합니다.
> > <img width="1000" alt="image" src="https://github.com/user-attachments/assets/82d61fff-4a5d-4310-87e3-caf2a6ff48ba" />
> 
> 5. Occupancy Map extension에서 **CALCULATE**를 클릭 후 **VISUALIZE IMAGE**를 클릭합니다.
> > <img width="460" height="551" alt="image" src="https://github.com/user-attachments/assets/5d66f508-6bb7-4ce6-91f7-2fc6b6dd24b1" />
> 
> 6. **Rotate Image**에서 180도를 선택합니다.<br>**Coordinate Type**에서 **ROS Occupancy Map Parameters File (YAML)**으로 선택합니다.<br>**RE-GENERATE IMAGE**를 클릭합니다.
> 
> 7. Occupancy map parameters YAML 형식으로 아래 필드에 표시됩니다. 전체 텍스트를 복사하세요.<br>`~/IsaacSim-ros_workspaces/humble_ws/src/navigation/carter_navigation/maps` 경로에 `carter_hospital_navigation.yaml`을 새로 생성하여 복사한 텍스트를 넣고 저장합니다.
> > <img width="460" height="123" alt="image" src="https://github.com/user-attachments/assets/2884e730-c9ce-4859-b642-469cb6c69cee" /><br>
> > 
> > ```text
> > image: World0.png
> > resolution: 0.05
> > origin: [-49.62500152587891, -4.775000190734863, 0.0000]
> > negate: 0
> > occupied_thresh: 0.65
> > free_thresh: 0.196
> > ```
> 
> 8. Visualization popup에서 **Save Image**를 클릭합니다.<br>`~/IsaacSim-ros_workspaces/humble_ws/src/navigation/carter_navigation/maps` 경로에 `carter_hospital_navigation.png`라는 이름으로 저장하세요.
> > <img width="1000" alt="carter_hospital_navigation" src="https://github.com/user-attachments/assets/50c30dc8-34fc-4309-8770-d50d04781c1d" />

## Multiple Robot ROS2 Navigation Setup
**Window > Examples > Robotics Examples**로 이동하세요.<br>**Robotics Examples** 탭을 클릭하세요.<br>**ROS2 > Navigation > Multiple Robots > Hospital Scene**를 누르고 우측에서 **Load Sample Scene**을 클릭하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/af9d5f13-4dad-4d68-b74c-00f4d324cc6b" /><br>
<br>
ROS2 Navigation setup에 대한 자세한 내용은 [ROS2 Navigation Sample](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#isaac-sim-app-tutorial-ros2-navigation)을 참조하세요.<br>
<br>
동일한 환경에서 여러 로봇을 작동시키기 위해 namespaces가 사용됩니다. 이렇게 하면 서로 다른 ROS2 패키지의 rostopic 및 rosnode 이름이 수정되어 동일한 ROS2 노드의 여러 인스턴스가 동시에 실행될 수 있습니다.<br>
<br>
ROS2 message를 namespaces로 publish하고 receive하기 위해, `Nova_Carter_ROS_X` 아래의 각 action graph에 있는 `node_namespace` OmniGraph 노드가 해당 로봇 이름으로 설정되었습니다. 샘플 carter_navig ROS2 패키지에 있는 `multiple_robot_carter_navigation_hospital.launch.py` 및 `multiple_robot_carter_navigation_office.launch.py` 실행 파일도 동일한 로봇 namespace로 구성되어 있습니다.<br>

## Running Multiple Robot ROS2 Navigation
1. **Window > Examples > Robotics Examples**로 이동하세요.<br>**Robotics Examples** 탭을 클릭하세요.<br>**ROS2 > Navigation > Multiple Robots > Hospital Scene**를 누르고 우측에서 **Load Sample Scene**을 클릭하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/af9d5f13-4dad-4d68-b74c-00f4d324cc6b" /><br>

2. **Play**를 클릭하여 시뮬레이션을 시작합니다.

3. 새로운 터미널에서 다음 명령을 실행하여 Multiple Robot Navigation을 실행합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch carter_navigation multiple_robot_carter_navigation_hospital.launch.py
> ```

4. 각 RViz2 창에서 **Map**을 클릭해서 Topic 이름을 확인합니다.
> <img width="200" alt="image" src="https://github.com/user-attachments/assets/5fdb8eae-d094-4ff7-9570-262cd606c2b9" /><br>
> <img width="200" alt="image" src="https://github.com/user-attachments/assets/4c63382d-ff74-4eb1-9532-a84aef2c9130" /><br>
> <img width="200" alt="image" src="https://github.com/user-attachments/assets/bb771d29-a5ef-4477-b68c-f7a0d65e400c" /><br>

5. 각 로봇의 위치는 `carter_navigation/params/hospital/` 또는 `carter_navigation/params/office/`에 정의되어 있습니다.

6. 각 RViz2에서 **Nav2 Goal**을 클릭한 다음 원하는 위치를 클릭합니다.
> [multiple_robot_ros2_navigation.webm](https://github.com/user-attachments/assets/7cfce602-4261-4238-b473-9b5ef81cc7a8)

## Troubleshooting
오류 발생 시 [Multiple Robot ROS2 Navigation Troubleshooting](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_multi_navigation.html#troubleshooting)를 참고하세요.
















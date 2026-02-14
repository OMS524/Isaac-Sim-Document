# ROS 2 Service for Manipulating Prims Attributes
## Environment Infomation
| Item | Description |
|-|-|
| Author | 오민석 |
| Date | 2026-02-14 |
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

## Service Message Types
ROS2 Service Prim 노드는 다음과 같은 메시지 유형의 네 가지 Service를 제공합니다:
> - 특정 경로 아래의 모든 prim path(및 types) 가져오기
> > `isaac_ros2_messages/srv/GetPrims`
> > ```text
> > string path             # get prims at path
> > ---
> > string[] paths          # list of prim paths
> > string[] types          # prim type names
> > bool success            # indicate a successful execution of the service
> > string message          # informational, for example, for error messages
> > ```
> - 특정 프림에 대한 모든 attribute names과 types 가져오기
> > `isaac_ros2_messages/srv/GetPrimAttributes`
> > ```text
> > string path             # prim path
> > ---
> > string[] names          # list of attribute base names (name used to Get or Set an attribute)
> > string[] displays       # list of attribute display names (name displayed in Property tab)
> > string[] types          # list of attribute data types
> > bool success            # indicate a successful execution of the service
> > string message          # informational, for example, for error messages
> > ```
> - prim attribute type 및 values 가져오기
> > `isaac_ros2_messages/srv/GetPrimAttribute`
> > ```text
> > string path             # prim path
> > string attribute        # attribute name
> > ---
> > string value            # attribute value (as JSON)
> > string type             # attribute type
> > bool success            # indicate a successful execution of the service
> > string message          # informational, for example, for error messages
> > ```
> - prim attribute value 설정
> > `isaac_ros2_messages/srv/SetPrimAttribute`
> > ```text
> > string path             # prim path
> > string attribute        # attribute name
> > string value            # attribute value (as JSON)
> > ---
> > bool success            # indicate a successful execution of the service
> > string message          # informational, for example, for error messages
> > ```

> [!NOTE]
> 기본 속성은 JSON(키 없이 데이터에 직접 적용됨)으로 읽고 씁니다. Arrays, vectors, matrixes 및 numeric containers(예: `pxr.Gf.Vec3f`, `pxr.Gf.Matrix4d`, `pxr.Gf.Quatd`)는 숫자 목록(먼저 행)으로 해석됩니다.

## Manipulating Prims Attributes
다음 예제는 ROS2 Service Prim 노드를 사용하여 Prim과 attributes을 나열하고 stage에서 object의 pose를 읽고 쓰는 방법을 보여줍니다.<br>

1. **Create > Shape > Cube**를 클릭하여 object를 생성합니다.

2. **Window > Graph Editors > Action Graph**로 이동하여 Action Graph를 생성하고 다음과 같이 구성합니다.
> <img width="582" height="352" alt="image" src="https://github.com/user-attachments/assets/dbfff9a5-f0b7-45d8-bd31-b36216fb7a7c" />

3. **Play**를 눌러 시뮬레이션을 시작합니다.

4. 새로운 터미널에서 다음 명령을 실행합니다.
> ROS2 소싱
> > ```bash
> > cd ~/IsaacSim-ros_workspaces/humble_ws/
> > export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> > source /opt/ros/humble/setup.bash
> > source install/local_setup.bash
> > ```
> 
> 사용 가능한 services 목록
> > ```bash
> > ros2 service list
> > ```
> > <img width="500" alt="image" src="https://github.com/user-attachments/assets/3d9d5929-7be5-4aa4-9dca-598d820bb5d9" /><br>
> 
> 모든 child prim paths와 types을 prim `/World` 아래로 가져옵니다:
> > ```bash
> > ros2 service call /get_prims isaac_ros2_messages/srv/GetPrims "{path: /World}"
> > ```
> > <img width="500" alt="image" src="https://github.com/user-attachments/assets/f3f9d6ed-effb-462f-b29f-1f5f1cf6e836" /><br>
> 
> Cube(`/World/Cube`) prim의 모든 attribute names과 types을 가져옵니다:
> > ```bash
> > ros2 service call /get_prim_attributes isaac_ros2_messages/srv/GetPrimAttributes "{path: /World/Cube}"
> > ```
> > <img width="500" alt="image" src="https://github.com/user-attachments/assets/998d1d2a-9b66-4294-8103-3e303a07fe50" /><br>
> 
> Cube(`/World/Cube`) prim의 pose(position 및 orientation)를 가져옵니다
> > ```bash
> > # get position
> > ros2 service call /get_prim_attribute isaac_ros2_messages/srv/GetPrimAttribute "{path: /World/Cube, attribute: xformOp:translate}"
> > # get orientation (quaternion: wxyz)
> > ros2 service call /get_prim_attribute isaac_ros2_messages/srv/GetPrimAttribute "{path: /World/Cube, attribute: xformOp:orient}"
> > ```
> > <img width="500" alt="image" src="https://github.com/user-attachments/assets/27c92936-0c76-4d94-b54c-294656dcb8ca" /><br>
> > <img width="500" alt="image" src="https://github.com/user-attachments/assets/467c011a-06ff-4742-aa5a-b7fb0e949b04" /><br>
> 
> Cube(`/World/Cube`) prim의 새 pose(position 및 orientation)를 설정합니다:
> > ```bash
> > # set position
> > ros2 service call /set_prim_attribute isaac_ros2_messages/srv/SetPrimAttribute "{path: /World/Cube, attribute: xformOp:translate, value: [1, 2, 3]}"
> > # set orientation (quaternion: wxyz)
> > ros2 service call /set_prim_attribute isaac_ros2_messages/srv/SetPrimAttribute "{path: /World/Cube, attribute: xformOp:orient, value: [0.7325378, 0.4619398, 0.1913417, 0.4619398]}"
> > ```



























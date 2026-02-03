# Automatic ROS 2 Namespace Generation
## Environment Infomation
| Item | Description |
|-|-|
| Author | 오민석 |
| Date | 2026-02-03 |
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

## ROS 2 Namespaces
ROS 2에서 namespaces를 관리하는 것은 다중 로봇 시뮬레이션에서 각 로봇의 topic를 고유하게 식별할 수 있도록 하는 데 매우 중요합니다.<br>
현재 OmniGraph 내에서 ROS publisher, subscriber 및 서비스를 위한 namespaces를 설정하는 방법은 크게 두 가지가 있습니다.<br>
1. nodeNamespace 필드의 namespaces 필드를 수동으로 설정합니다.
> <img width="736" height="218" alt="image" src="https://github.com/user-attachments/assets/cbc90cda-6e98-4743-a719-b5e21675275b" /><br>
2. (Recommended) 모든 Isaac Sim ROS OmniGraph 노드의 namespaces를 자동으로 생성하도록 asset을 구성합니다.<br>이 튜토리얼은 Isaac Sim에서 namespaces를 설정하는 과정을 안내하여 효율적인 topic 관리와 충돌 방지를 가능하게 합니다.
<br>

### Configuring the Asset
#### Setting Up the Base Asset
1. **Window > Script Editor**을 열고 다음 코드를 넣고 실행하세요.
> ```python
> # Import necessary modules
> from pxr import UsdGeom
> import omni.usd
> 
> # Retrieve the current stage
> stage = omni.usd.get_context().get_stage()
> 
> # Ensure a stage is loaded
> if not stage:
>     print("No stage is currently loaded. Please load a stage and try again.")
> else:
>     # Create the mock_robot Xform as the root
>     mock_robot = UsdGeom.Xform.Define(stage, "/mock_robot")
> 
>     # Create the base_link Xform under mock_robot
>     base_link = UsdGeom.Xform.Define(stage, "/mock_robot/base_link")
> 
>     # Create lidar_link and position it 0.4 meters above the base_link (Z-axis)
>     lidar_link = UsdGeom.Xform.Define(stage, "/mock_robot/base_link/lidar_link")
>     lidar_link.AddTranslateOp().Set(value=(0, 0, 0.4))  # Offset along Z-axis
> 
>     # Create camera_link and position it 0.2 meters above the base_link (Z-axis)
>     camera_link = UsdGeom.Xform.Define(stage, "/mock_robot/base_link/camera_link")
>     camera_link.AddTranslateOp().Set(value=(0, 0, 0.2))  # Offset along Z-axis
> 
>     # Create wheel_left and wheel_right Xforms under base_link
>     wheel_left = UsdGeom.Xform.Define(stage, "/mock_robot/base_link/wheel_left")
>     wheel_right = UsdGeom.Xform.Define(stage, "/mock_robot/base_link/wheel_right")
> 
>     # Position wheel_left 0.2 meters to the left of the center (X-axis)
>     wheel_left.AddTranslateOp().Set(value=(-0.2, 0, 0))
> 
>     # Position wheel_right 0.2 meters to the right of the center (X-axis)
>     wheel_right.AddTranslateOp().Set(value=(0.2, 0, 0))
> ```
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/805b7546-94e0-4cd1-92aa-8a5963e8cf0d" /><br>

2. **Create > Sensors > RTX Lidar > NVIDIA > Example Rotary 2D**로 이동하여 2D RTX Lidar sensor를 추가하고 `/mock_robot/base_link/lidar_link` 아래로 드래그합니다.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/8e2d56a4-3b48-4825-809e-fcda46e6f84b" /><br>

3. **Create > Sensors > Camera and Depth Sensors > LeopardImaging > Hawk**로 이동하여 Hawk stereo camera system을 추가하고 `/mock_robot/base_link/camera_link` 아래로 드래그합니다.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/51d54eab-6dd3-44ed-8a32-1a274953152f" /><br>

4. **Tools > Robotics > ROS 2 OmniGraphs > Generic Publisher**로 이동하여 Generic Publisher를 만듭니다.<br>**Generic Publisher Graph**를 `Publish String`로 설정하고 **Graph Path**를 `/mock_robot/base_link/wheel_left/String_graph`로 설정합니다.<br>그런 다음 **OK**을 누릅니다.
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/680a9a1a-4e7f-4214-9cdb-e96f37801520" /><br>
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/010011b6-09c2-46b0-b205-2a1319707ac0" /><br>

5. **Tools > Robotics > ROS 2 OmniGraphs > TF Publisher**로 이동하여 TF Publisher를 만듭니다.<br>**Target Prim**을 `/mock_robot`으로 설정하고 **Graph Path**를 `/mock_robot/base_link/wheel_left/TF_graph`로 설정합니다.<br>그런 다음 **OK**을 누릅니다.
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/8f0cd0fc-7b43-4288-bb59-e8ffcef78b84" /><br>
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/9d10f3e7-399b-40d5-8a19-6d9e31416191" /><br>

6. **Tools > Robotics > ROS 2 OmniGraphs > Camera**로 이동하여 Camera Publisher를 만듭니다.<br>**Camera Prim**을 `/mock_robot/base_link/camera_link/Hawk/left/camera_left`로 설정하고 **Graph Path**를 `/mock_robot/base_link/camera_link/Hawk/Camera_Left_Graph`로 설정합니다. **Depth** 항목을 선택 취소한 다음 **OK**을 누릅니다.
> <img width="400" alt="image" src="https://github.com/user-attachments/assets/5aec00df-b602-47cb-aedd-44c66cbc043d" /><br>
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/e6dddd43-eed6-4a54-9526-e8148ce6a270" /><br>

7. **Tools > Robotics > ROS 2 OmniGraphs > Camera**로 이동하여 second Camera Publisher를 만듭니다.<br>**Camera Prim**을 `/mock_robot/base_link/camera_link/Hawk/right/camera_right`로 설정하고 **Graph Path**를 `/mock_robot/base_link/camera_link/Hawk/Camera_Right_Graph`로 설정합니다. **Depth** 항목을 선택 취소한 다음 **OK**을 누릅니다.
> <img width="400" alt="image" src="https://github.com/user-attachments/assets/73c0805f-e49e-46b1-9aee-398d106bc01a" /><br>
> <img width="500" height="388" alt="image" src="https://github.com/user-attachments/assets/f644d8eb-0064-49f6-a484-2a233c475bb0" /><br>

8. **Tools > Robotics > ROS 2 OmniGraphs > RTX Lidar**로 이동하여 2D RTX Lidar Publisher를 만드세요.<br>**Lidar Prim**을 `/mock_robot/base_link/lidar_link/Example_Rotary_2D`로 설정하고 **Graph Path**를 `/mock_robot/base_link/lidar_link/Lidar_Graph`로 설정합니다. **Laser Scan**만 활성화된 다음 **OK**을 누르세요.
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/58b2bce7-3032-4196-b3cb-b63481e1ab9f" /><br>
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/6722d598-31c8-47d5-a210-01bb71e5075d" /><br>
<br>

#### Configuring Namespace Attributes
이제 기본 asset이 설정되었으므로 namespace 값을 원하는 각 prim에 대해 `isaac:namespace` 속성을 추가해야 합니다.<br>
namespace는 prim 계층의 상단에서 각 ROS publisher에 설정된 각 `isaac:namespace` 속성 값을 추가하여 생성됩니다.<br>
namespace 생성 동작은 ROS publisher의 유형과 단계에서 위치에 따라 달라집니다.<br>
<br>
**ROS 2 TF OmniGraph Nodes**: 생성된 namespace에는 namespace 속성 집합이 있는 최상위 prim의 namespace value만 포함됩니다.<br>
이는 한 로봇 내에서 publish된 모든 TF가 해당 로봇의 namespace(즉, `robot1/tf`) 아래에만 위치하기 때문입니다.<br>
<br>
**ROS 2 Camera & Lidar OmniGraph Helper Nodes**: Camera 또는 Lidar render product의 path는 namespace 검색 알고리즘이 취하는 path를 식별하고 그에 따라 namespace value를 추가하는 데 사용됩니다.<br>
따라서 이 경우 Camera/Lidar Helper node의 위치는 관련이 없으며, 오히려 Camera/Lidar sensor prim의 위치가 사용됩니다.<br>
<br>
**All other OmniGraph nodes**: OmniGraph 노드로의 path는 namespace 검색 알고리즘이 선택한 path를 식별하고 그에 따라 namespace value를 추가하는 데 사용됩니다. 이 경우 이러한 OmniGraph 노드의 위치가 중요합니다.<br>
<br>

#### Adding the `isaac:namespace` Prim Attribute
prim에 `isaac:namespace` 속성을 추가하려면 다음 단계를 따릅니다:
> 1. prim을 선택하고 property 창에서 **Add**를 클릭합니다. popup 메뉴에서 **Isaac > Namespace**로 이동합니다. 이 속성이 prim에 적용됩니다.<br>
> <img width="200" alt="image" src="https://github.com/user-attachments/assets/032e3771-fcd1-4c08-95a2-35d7b70823bb" /><br>
> 2. property 패널에서 namespace 필드에 namespace value을 추가합니다.<br>
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/04815def-ecd8-481e-ac0f-905a59c0e07c" />

#### Testing the isaac:namespace Prim Attribute
다음 prim에 `isaac:namespace` 속성을 적용합니다. 이 튜토리얼에서는 각 namespace value을 prim 이름으로 설정합니다(사용자 지정 namespace value을 시도해 볼 수는 있지만):
> - `/mock_robot/base_link/lidar_link`<br>
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/389d4747-c305-4e09-a8cb-8dd310dcbbf1" /><br>
> - `/mock_robot/base_link/camera_link`<br>
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/b785f462-c3d5-4845-b611-a25409bccf5b" /><br>
> - `/mock_robot/base_link/camera_link/Hawk`<br>
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/d8ff9d1f-7b03-49ca-a096-76a521293517" /><br>
> - `/mock_robot/base_link/camera_link/Hawk/left`<br>
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/d853e2d6-f047-4f56-8684-58ceeb39eb3f" /><br>
> - `/mock_robot/base_link/camera_link/Hawk/right`<br>
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/4fec3de0-10b4-4b44-874f-5634ecaafb73" /><br>
> - `/mock_robot/base_link/wheel_left`<br>
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/2a153d5d-5455-4f1d-8888-df7c878afe5c" /><br>

1. **Play**을 클릭하고 시뮬레이션을 시작합니다.
2. 새로운 터미널에서 다음 명령어를 실행하여, 다음 topic들이 출력되는지 확인합니다.
```bssh
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
```bash
ros2 topic list
```
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/ea6e997c-dc16-4388-8a63-2ab856b9c420" /><br>
> - `/camera_link/Hawk/left/camera_info`<br>
> - `/camera_link/Hawk/left/rgb`<br>
> - `/camera_link/Hawk/right/camera_info`<br>
> - `/camera_link/Hawk/right/rgb`<br>
> - `/lidar_link/laser_scan`<br>
> - `/wheel_left/tf`<br>
> - `/wheel_left/topic`<br>
> 
> 위 목록에서 자동으로 생성된 topic을 확인할 수 있습니다.<br>
> namespace에 사용자 지정 이름 체계가 필요한 경우 각 ROS OmniGraph 노드에 대한 `nodeNamespace` 입력 필드를 입력할 수 있습니다.

3. 시뮬레이션을 중지합니다. `/mock_robot` prim을 선택하고 `isaac:namespace` 속성을 추가합니다. 그런 다음 namespace 값을 prim 이름으로 설정합니다.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/5e737f01-e3d8-488d-8610-2568787f8436" /><br>

4. `/mock_robot` Prim을 선택하고, 마우스 오른쪽 버튼을 클릭한 다음, **Duplicate**를 눌러 복제합니다.<br>새로 생성된 `/mock_robot_01`의 경우, Prim을 선택하고 속성 패널로 이동한 다음, `isaac:namespace` 속성을 `mock_robot_01`로 변경합니다.
> <img width="200" alt="image" src="https://github.com/user-attachments/assets/e56d3bb6-4a4f-4fad-9c29-ffb2f027495b" /><br>
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/4d900506-8f33-4407-a899-e2869d9aeae3" /><br>

5. **Play**을 눌러 시뮬레이션을 시작합니다.

6. 새로운 터미널에서 다음 명령어를 실행하여, 다음 topic들이 출력되는지 확인합니다.
```bssh
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
```bash
ros2 topic list
```
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/ade01818-fe6e-474d-bb3a-496ccdfe9b08" /><br>
> **Topics from mock_robot**
> > - `/mock_robot/camera_link/Hawk/left/camera_info`
> > - `/mock_robot/camera_link/Hawk/left/rgb`
> > - `/mock_robot/camera_link/Hawk/right/camera_info`
> > - `/mock_robot/camera_link/Hawk/right/rgb`
> > - `/mock_robot/lidar_link/laser_scan`
> > - `/mock_robot/tf`
> > - `/mock_robot/wheel_left/topic`
> 
> **Topics from mock_robot_01**
> > - `/mock_robot_01/camera_link/Hawk/left/camera_info`
> > - `/mock_robot_01/camera_link/Hawk/left/rgb`
> > - `/mock_robot_01/camera_link/Hawk/right/camera_info`
> > - `/mock_robot_01/camera_link/Hawk/right/rgb`
> > - `/mock_robot_01/lidar_link/laser_scan`
> > - `/mock_robot_01/tf`
> > - `/mock_robot_01/wheel_left/topic`

> [!IMPORTANT]
> 위 목록에서 topic들이 자동으로 생성된 것을 확인할 수 있습니다.<br>namespace에 사용자 지정 이름 체계가 필요한 경우 각 ROS OmniGraph 노드에 대한 nodeNamespace 입력 필드를 입력할 수 있습니다.








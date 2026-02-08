# ROS 2 Bridge in Standalone Workflow
## Environment Infomation
| Item | Description |
|-|-|
| Author | 오민석 |
| Date | 2026-02-08 |
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

## Manually Stepping ROS2 Components
Standalone scripting은 일반적으로 시뮬레이션 단계를 수동으로 제어하는 데 이상적입니다. OnImpulseEvent OmniGraph 노드는 publisher와 subscriber의 frequency를 신중하게 제어할 수 있도록 모든 ROS2 OmniGraph 노드에 연결할 수 있습니다.<br>
<br>
ROS2 Publish Clock 노드가 있는 새 action graph를 ROS2 Domain ID 1로 정밀하게 제어하도록 설정하는 방법의 예입니다:<br>
```python
import omni.graph.core as og
# Create a new graph with the path /ActionGraph
og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
        ],
        og.Controller.Keys.CONNECT: [
            # Connecting execution of OnImpulseEvent node to PublishClock so it will only publish when an impulse event is triggered
            ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
            # Connecting simulationTime data of ReadSimTime to the clock publisher node
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            # Connecting the ROS2 Context to the clock publisher node so it will run under the specified ROS2 Domain ID
            ("Context.outputs:context", "PublishClock.inputs:context"),
        ],
        og.Controller.Keys.SET_VALUES: [
            # Assigning topic name to clock publisher
            ("PublishClock.inputs:topicName", "/clock"),
            # Assigning a Domain ID of 1 to Context node
            ("Context.inputs:domain_id", 1),
            # Disable useDomainIDEnvVar to ensure we use the above set Domain ID
            ("Context.inputs:useDomainIDEnvVar", False),
        ],
    },
)
```
어떤 frame에서든 다음을 실행하여 impulse event를 설정하면 clock publisher를 한 번 tick합니다:<br>
```python
og.Controller.set(og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)
```
> [!NOTE]
> standalone scripting에서 렌더링 및 물리 시뮬레이션 단계를 명시적으로 제어하기 때문에 각 단계를 완료하는 데 걸리는 시간은 계산 부하에 따라 달라지며 실시간으로 일치하지 않을 가능성이 높습니다. 이로 인해 standalone scripting을 사용하는 애플리케이션과 GUI를 사용하는 애플리케이션을 실행할 때 관찰된 action speed에 차이가 발생할 수 있습니다. 이 경우 시뮬레이션 clock을 reference로 사용하세요.

## Examples
튜토리얼 예제 중 일부는 standalone Python 예제로 변환되었습니다. 다음은 이를 실행하기 위한 지침입니다.

### ROS 2 Clock
다음 두 샘플은 ROS 2 RGB image, depth image 및 camera info publishers를 설정하는 데 사용되는 ROS 2 Camera Helper 및 Camera Info Helper OmniGraph 노드를 사용하여 action graph를 생성하는 방법을 보여줍니다. 두 샘플 모두 ROS 2 image data를 다른 rate로 publish하는 동일한 결과를 얻지만 다른 솔루션을 사용합니다.<br>
<br>

- On each frame:
> - Camera Info is published
- Every 5 frames:
> - RGB image is published
- Every 60 frames:
> - Depth image is published

### Periodic Image Publishing
각 ROS 2 image 및 camera info publishers의 execution rate(every N frames)은 SDGPipeline graph에서 각 Isaac Simulation Gate OmniGraph 노드를 수정하여 설정됩니다. execution rate을 설정하면 image publisher는 렌더링된 모든 N frames에 자동으로 체크됩니다.<br>
<br>

1. 컨테이너에서 다음을 명령을 실행하세요.
```bash
./python.sh standalone_examples/api/isaacsim.ros2.bridge/camera_periodic.py
```
2. 새로운 터미널에서 다음 명령을 실행하여 Rviz2에서 시각화하세요.
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
```bash
rviz2 -d ./src/isaac_tutorials/rviz2/camera_manual.rviz
```
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/c8881307-117b-490b-9658-afda7101c0ee" />

> [!NOTE]
> RViz2 문제로 인해 depth image displays에 black frames이 나타날 수 있습니다. Isaac Sim이 depth images를 올바르게 publish하고 있는지 확인하려면 다음을 진행하세요.
> 1. 새로운 터미널에서 다음 명령을 실행하세요.
> ```bash
> ros2 run rqt_image_view rqt_image_view
> ```
> 2. topic을 `/depth`로 설정하세요.
> <img width="500" height="568" alt="image" src="https://github.com/user-attachments/assets/16c4ba09-bcdf-4a55-b10e-e5c4d7222836" />

### Manual Image Publishing
ROS 2 image 및 camera info publishers는 각 publisher node와 해당 Isaac Simulation Gate OmniGraph node 사이에 Branch OmniGraph nodes를 주입하여 수동으로 제어됩니다. 브랜치 노드는 사용자 지정 gate처럼 작동하며 언제든지 활성화하거나 비활성화할 수 있습니다. 브랜치 노드가 활성화될 때마다 연결된 ROS 2 publisher node가 tick됩니다.<br>
<br>

1. 컨테이너에서 다음을 명령을 실행하세요.
```bash
./python.sh standalone_examples/api/isaacsim.ros2.bridge/camera_manual.py
```
2. 새로운 터미널에서 다음 명령을 실행하여 Rviz2에서 시각화하세요.
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
```bash
rviz2 -d ./src/isaac_tutorials/rviz2/camera_manual.rviz
```
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/0dc5be67-8c10-446d-bf3f-d7075160afd6" />

### Carter Stereo
이 샘플은 ROS 2 component nodes가 포함된 action graph를 사용하여 기존 USD stage를 수행하고 기본 설정을 수정하는 방법을 보여줍니다. stereo camera pair가 자동으로 활성화되고 두 번째 viewport 창이 UI에 도킹됩니다.<br>
<br>

- On each frame:
  - The ROS 2 clock is published
  - A ROS 2 PointCloud2 message originating from an RTX Lidar is published
  - Odometry is published
  - The Twist subscriber is spun
  - TF messages are published
  - Left and right cameras are published
- Every Two Frames:
  > - The Twist command message is published

1. 컨테이너에서 다음을 명령을 실행하세요.
```bash
./python.sh standalone_examples/api/isaacsim.ros2.bridge/carter_stereo.py
```
2. 새로운 터미널에서 다음 명령을 실행하여 Rviz2에서 시각화하세요.
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
```bash
rviz2 -d ./src/isaac_tutorials/rviz2/carter_stereo.rviz
```
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/e9c23bb3-76ff-4136-b364-d686f01ae30a" />

### Multiple Robot ROS 2 Navigation
이 샘플은 기존 USD stage를 실행하는 방법을 보여줍니다.<br>
출력을 시각화하려면 [Multiple Robot ROS2 Navigation]()을 참조하세요:<br>
<br>

- On each frame:
    - The ROS 2 clock component is published
    - ROS 2 PointCloud2 messages originating from RTX Lidars are published
    - Odometry is published
    - The Twist subscriber is spun
    - TF messages are published

샘플은 hospital 및 office 환경 모두에서 실행할 수 있습니다. 컨테이너에서 다음 명령 중 하나를 실행하여 지정된 환경에서 샘플을 실행합니다:
Hospital Environment
```bash
./python.sh standalone_examples/api/isaacsim.ros2.bridge/carter_multiple_robot_navigation.py --environment hospital
```
Office Environment
```bash
./python.sh standalone_examples/api/isaacsim.ros2.bridge/carter_multiple_robot_navigation.py --environment office
```

### MoveIt2
이 샘플은 여러 USD stage를 추가하는 방법을 보여줍니다. 또한 ROS 2 component nodes로 action graph를 수동으로 생성한 다음 수동으로 tick하는 방법도 보여줍니다.<br>
출력을 시각화하려면 [MoveIt 2]()을 참조하세요:<br>
<br>

- On each frame:
    - The ROS 2 clock is published
    - Joint State messages are published
    - Joint State subscriber is spun
    - TF messages are published

컨테이너에서 다음을 명령을 실행하세요.
```bash
./python.sh standalone_examples/api/isaacsim.ros2.bridge/moveit.py
```

### Receiving ROS 2 Messages
다음은 빈 ROS2 message를 receive하면 장면의 큐브가 무작위 위치로 텔레포트하는 basic subscriber example입니다. 렌더링이 활성화된 상태에서 실행 중인 이 예제는 장면과 큐브가 움직이는 것을 관찰할 수 있는지 확인할 수 있습니다.<br>
<br>

컨테이너에서 다음을 명령을 실행하세요.
```bash
./python.sh standalone_examples/api/isaacsim.ros2.bridge/subscriber.py
```
큐브가 있는 장면이 로드된 후 새로운 터미널에서 다음 명령을 실행하여 empty message를 수동으로 publish할 수 있습니다. 1Hz rate를 사용합니다.
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
```bash
ros2 topic pub -r 1 /move_cube std_msgs/msg/Empty
```







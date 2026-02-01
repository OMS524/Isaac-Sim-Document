# ROS2 Joint Control: Extension Python Scripting
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

## Add Joint States in UI
1. **Isaac Sim > Robots > FrankaRobotics > FrankaPanda > franka.usd**를 Stage로 드래그하여 Asset을 가져옵니다.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/a2831870-a70f-47c9-ad2b-31894d54fe47" /><br>

2. **Window > Graph Editors > Action Graph**로 이동하여 새로운 Action Graph를 생성하여 다음과 같이 구성합니다.
> <img width="700" alt="image" src="https://github.com/user-attachments/assets/e40bf492-7774-4e0f-9daa-78b0654b2ce5" /><br>

3. **ROS2 Publish Joint State** Property에서 `targetPrim`에 `/franka`를 추가합니다.
> <img width="500" height="295" alt="image" src="https://github.com/user-attachments/assets/9e3da777-daa4-4c4c-81f1-dff91ddefbea" /><br>

4. **Articulation Controller** Property에서 `targetPrim`에 `/franka`를 추가합니다.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/1e5dbb7e-c3ab-48fa-ae6f-d017a410ccef" /><br>

5. **Play**를 눌러 시뮬레이션을 시작하세요.

6. 새로운 터미널에서 다음 명령어를 실행하여 franka를 제어하세요.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 run isaac_tutorials ros2_publisher.py
> ```

7. 새로운 터미널에서 다음 명령어를 실행하여 franka의 `joint_states`를 확인하세요.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 topic echo /joint_states
> ```

> [ros2_joint_control_extension_python_scripting_1.webm](https://github.com/user-attachments/assets/2ac8eb90-3d21-4720-b372-9a0e7cd8eafa)

> [!NOTE]
> Articulation root는 시뮬레이션에서 로봇을 구성하는 link와 joint의 집합인 Articulation tree의 시작을 설명합니다.<br>
> franka와 같은 고정 기반 로봇의 경우, Articulation root는 world와의 root joint에서 지정되며,<br>
> 이동 가능한 물체의 경우, Articulation root는 가장 깊은 tree(일반적으로 torso 또는 chassis_link)를 가진 rigid body에서 지정됩니다.<br>

## Graph Shortcut
**Tools > Robotics > ROS 2 OmniGraphs > JointStates**를 통해 불러 올 수 있습니다.

## Add Joint States in Extension
UI를 사용하여 수행한 동일한 작업은 파이썬 스크립트를 사용하여 수행할 수도 있습니다.<br>
NVIDIA Isaac Sim 사용의 다양한 Workflows.에 대한 자세한 내용은 Workflows에 대한 문서에서 확인할 수 있습니다.

1. **Isaac Sim > Robots > FrankaRobotics > FrankaPanda > franka.usd**를 Stage로 드래그하여 Asset을 가져옵니다.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/a2831870-a70f-47c9-ad2b-31894d54fe47" /><br>

2. **Window > Script Editor**로 이동하여 다음 코드를 넣습니다.
> ```python
> import omni.graph.core as og
> 
> og.Controller.edit(
>     {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
>     {
>         og.Controller.Keys.CREATE_NODES: [
>             ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
>             ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
>             ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
>             ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
>             ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
>         ],
>         og.Controller.Keys.CONNECT: [
>             ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
>             ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
>             ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
> 
>             ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
> 
>             ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
>             ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
>             ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
>             ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
>         ],
>         og.Controller.Keys.SET_VALUES: [
>             # Providing path to /panda robot to Articulation Controller node
>             # Providing the robot path is equivalent to setting the targetPrim in Articulation Controller node
>             # ("ArticulationController.inputs:usePath", True),      # if you are using an older version of Isaac Sim, you can  uncomment this line
>             ("ArticulationController.inputs:robotPath", "/World/franka"),
>             ("PublishJointState.inputs:targetPrim", "/World/franka")
>         ],
>     },
> )
> ```
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/716945b3-70f8-42f4-9f7c-7bb6154f55b2" /><br>

3. **Script Editor**에서 **Run**을 클릭하여 Stage에서 Action Graph가 생기는지 확인합니다.
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/227c3199-22d0-46d6-a240-638633873e80" /><br>

4. **Play**를 눌러 시뮬레이션을 시작하세요.

5. 새로운 터미널에서 다음 명령어를 실행하여 franka를 제어하세요.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 run isaac_tutorials ros2_publisher.py
> ```

6. 새로운 터미널에서 다음 명령어를 실행하여 franka의 `joint_states`를 확인하세요.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 topic echo /joint_states
> ```

> [ros2_joint_control_extension_python_scripting_2.webm](https://github.com/user-attachments/assets/c1670166-fdab-429b-9d38-e2548db5d13f)

## Position and Velocity Control Modes
joint state subscriber는 position 및 velocity control를 지원합니다. 각 조인트는 한 번에 단일 모드로만 제어할 수 있지만, 동일한 articulation tree의 다른 joint는 다른 모드로 제어할 수 있습니다. 각 joint의 stiffness 및 damping 매개변수가 원하는 control mode(position control: stiffness >> damping, velocity control: stiffness = 0)에 맞게 적절하게 설정되었는지 확인합니다.

이 snippet은 동일한 모드를 사용하는 관절을 하나의 메시지로 그룹화하여 position control와 velocity control를 모두 사용하여 로봇을 명령하는 방법을 보여주는 예시입니다. 이를 분리하여 position control joint와 velocity control joint에 대해 두 가지 다른 메시지를 생성합니다. 이를 분리하는 것은 조직화를 위한 것이며, 잠재적으로 서로 다른 rate로 전송할 수 있습니다.
```python
import threading

import rclpy
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node('position_velocity_publisher')
pub = node.create_publisher(JointState, 'joint_command', 10)

# Spin in a separate thread
thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
thread.start()

joint_state_position = JointState()
joint_state_velocity = JointState()

joint_state_position.name = ["joint1", "joint2","joint3"]
joint_state_velocity.name = ["wheel_left_joint", "wheel_right_joint"]
joint_state_position.position = [0.2,0.2,0.2]
joint_state_velocity.velocity = [20.0, -20.0]

rate = node.create_rate(10)
try:
    while rclpy.ok():
        pub.publish(joint_state_position)
        pub.publish(joint_state_velocity)
        rate.sleep()
except KeyboardInterrupt:
    pass
rclpy.shutdown()
thread.join()
```

원하는 경우 하나의 메시지로 결합할 수 있습니다. 해당 control mode로 control되지 않는 joint에는 'nan'을 사용합니다.

```python
joint_state = JointState()
joint_state.name = ["joint1", "joint2","joint3", "wheel_left_joint", "wheel_right_joint"]
joint_state.position = [0.2,0.2,0.2, float('nan'), float('nan')]
joint_state.velocity = [float('nan'), float('nan'), float('nan'), 20.0, -20.0]
```






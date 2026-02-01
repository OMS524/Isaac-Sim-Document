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

5. **Play**를 눌러 시뮬레이션을 시작하세요

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

> [ros2_joint_control_extension_python_scripting.webm](https://github.com/user-attachments/assets/080a992a-cc7d-4a57-836f-6e03b83f84b3)

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
>             ("ArticulationController.inputs:robotPath", "/panda"),
>             ("PublishJointState.inputs:targetPrim", "/panda")
>         ],
>     },
> )


## Position and Velocity Control Modes










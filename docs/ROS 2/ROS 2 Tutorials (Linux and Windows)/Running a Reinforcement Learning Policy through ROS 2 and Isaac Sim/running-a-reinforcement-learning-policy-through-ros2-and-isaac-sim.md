# Running a Reinforcement Learning Policy through ROS 2 and Isaac Sim
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

## Getting Started
해당 예제를 진행하기 위해 다음을 진행하세요.
- Isaac Sim 내에서 **Window > Extensions**로 이동하여 `isaacsim.ros2.bridge`를 Enable하세요.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/0fa2211b-b9fa-4a93-aa73-92dfd93973b3" />
- ROS 2 워크스페이스에 `h1_fullbody_controller` 패키지가 있어야 합니다.<br>없을 경우 [ROS 2 Humble Installation](/docs/Installation/ROS%202/ros2-humble-installation.md)를 참고하세요.
- torch package가 필요합니다.<br>[PyTorch](https://pytorch.org/get-started/locally/) 설치 지침을 따라 설치합니다.
> 1. CUDA 지원 버전 확인
> ```bash
> nvidia-smi
> ```
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/d1e40da6-f539-41aa-aa84-acda1b6a4a3a" />
> 
> 2. [PyTorch](https://pytorch.org/get-started/locally/)에서 다음과 같이 옵션 선택 후 설치 명령어 확인<br>안정성을 위해 CUDA 버전을 최전 버전이 아닌 12.8 선택
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/91ce023c-573b-4e24-9e45-b906fbe5fc6c" />
> 
> 3. 설치 명령어 실행
> ```bash
> sudo apt update
> ```
> pip 설치 (없을 시)
> ```bash
> sudo apt install -y python3-pip python3-venv
> ```
> pip 확인
> ```bash
> python3 -m pip --version
> ```
> PyTorch 설치
> ```bash
> pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu128
> ```
> GPU 동작 확인
> ```bash
> python3 - <<EOF
> import torch
> print("torch:", torch.__version__)
> print("cuda available:", torch.cuda.is_available())
> print("torch cuda:", torch.version.cuda)
> if torch.cuda.is_available():
>     print("gpu:", torch.cuda.get_device_name(0))
>     x = torch.randn(1024, 1024, device="cuda")
>     print("cuda tensor ok:", x.mean().item())
> EOF
> ```
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/34bedf61-9f68-49b2-ba6e-62714233ae3b" />

## Set Up Robot Joint Configurations
Set Up Robot Joint Configurations은 [Tutorial 13: Rigging a Legged Robot for Locomotion Policy](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup_tutorials/tutorial_rig_legged_robot.html#isaac-sim-app-tutorial-rig-legged-robot)의 내용을 진행합니다.<br>
<br>

1. content browser에서 `Isaac Sim/Robots/Unitree/H1`에 있는 `h1.usd`를 Stage로 드래그합니다.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/f4451ce5-f89f-4a56-acbe-012ab98b176d" /><br>

2. 로봇이 떨어지는 것을 방지하기 위해 `/h1/torso_link`를 오른쪽 클릭하고 **Create > Physics > Joint > Fixed Joint**를 클릭합니다.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/b3d075c4-fbf1-418f-9dfc-1b7880f10354" /><br>

3. Stage 우측 상단에 `funnel` 아이콘을 선택하고 `Physics Joints`를 선택합니다.
> <img width="250" alt="image" src="https://github.com/user-attachments/assets/6c0f159a-268f-4a4b-9c4d-98468f0f8555" /><br>

4. `left_hip_yaw`를 클릭하고 Shift를 누른 상태에서 `right_elbow`를 눌러 모든 joint를 선택합니다.<br>**Add > Physics > Joint State Angular**를 추가하세요.<br>**Add > Physics > Angular drive**를 추가하세요.
> <img width="250" alt="image" src="https://github.com/user-attachments/assets/9f274499-8c80-49ef-9f86-dd4aad3f1f79" />
> <img width="250" alt="image" src="https://github.com/user-attachments/assets/dc82953e-2b54-467d-aae5-b9b986cfc152" /><br>

5. 각 joint 마다 Property에서 joint drive API의 `Target Position`를 다음 내용의 `joint_pos`를 참고해서 설정하세요.<br>각 joint 마다 Property에서 joint drive API의 `Target Velocity`를 다음 내용의 `joint_vel`를 참고해서 설정하세요.<br>
> [!NOTE]
> `joint_pos`, `joint_vel`는 radian 단위이기에 `Target Velocity`에 넣을 때 degree로 변환해서 넣어야 합니다.<br>
> 
> **joint_pos**<br>
> | Joint | Rad | Deg |
> |-|-|-|
> | *_hip_yaw | 0.0 | 0.0 |
> | *_hip_roll | 0.0 | 0.0 |
> | *_hip_pitch | -0.28 | -16.04282 |
> | *_knee | 0.79 | 45.26367 |
> | *_ankle | -0.52 | -29.79381 |
> | torso | 0.0 | 0.0 |
> | *_shoulder_pitch | 0.28 | 16.04282 |
> | *_shoulder_roll | 0.0 | 0.0 |
> | *_shoulder_yaw | 0.0 | 0.0 |
> | *_elbow | 0.52 | 29.79381 |

> ```python
> robot:
>   init_state:
>     joint_pos:
>       .*_hip_yaw: 0.0
>       .*_hip_roll: 0.0
>       .*_hip_pitch: -0.28
>       .*_knee: 0.79
>       .*_ankle: -0.52
>       torso: 0.0
>       .*_shoulder_pitch: 0.28
>       .*_shoulder_roll: 0.0
>       .*_shoulder_yaw: 0.0
>       .*_elbow: 0.52
>     joint_vel:
>       .*: 0.0
> ```
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/8b7b4c58-6545-4683-8c92-7390d4a8658d" />

6. **Play**를 눌러 시뮬레이션을 실행하여 로봇이 지정된 초기 위치로 이동하는 것을 확인합니다.
> [running-a-reinforcement-learning-policy-through-ros2-and-isaac-sim_1.webm](https://github.com/user-attachments/assets/eb1f2290-f7a9-4a2d-a83f-3c7b377c6eec)
<br>

joint state API 값이 재설정되지 않도록 하려면 로봇 상태를 정지 상태로 재설정하지 않도록 시뮬레이션 설정을 변경해야 합니다.
1. **Edit > Preferences**를 클릭하세요.
2. 왼쪽 카테고리에서 **Physics**를 클릭하세요.
3. **Reset Simulation on Stop**를 체크 해제하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/17e78913-81e8-46cc-b2fd-cb437ecd061a" /><br>

4. 생성했던 **Fixed Joint**를 삭제하세요.
5. 다시 **Reset Simulation on Stop**를 체크하세요.

## Setting Joint Configuration
1. 각 joint 마다 Property에서 **joint drive API**의 `Stiffness`를 다음 내용의 `stiffness`를 참고해서 설정하세요.<br>각 joint 마다 Property에서 **joint drive API**의 `Damping`를 다음 내용의 `damping`를 참고해서 설정하세요.<br>`d435_left_imager`, `d435_rgb_module`, `imu`, `logo`, `mid360`을 제외한 joint의 Property에서 **joint drive API**의 `Max Force`를 다음 내용의 `effort_limit`를 참고해서 설정하세요.<br>`d435_left_imager`, `d435_rgb_module`, `imu`, `logo`, `mid360`을 제외한 joint의 Property에서 **Raw USD Properties**의 `Maximum Joint Velocity`를 다음 내용의 `velocity_limit`를 참고해서 설정하세요.<br>
> [!NOTE]
> `stiffness`, `damping`, `velocity_limit`는 다음 수식을 이용하여 degree로 변환해서 넣어야 합니다.<br>
> <br>
> $S_{\text{deg}} = S_{\text{rad}} \times \frac{\pi}{180}$<br>
> $D_{\text{deg}} = D_{\text{rad}} \times \frac{\pi}{180}$<br>
> $\omega_{\text{deg}} = \omega_{\text{rad}} \times \frac{180}{\pi}$<br>
> <br>
> 
> **stiffness**<br>
> | Joint | Rad | Deg |
> |-|-|-|
> | *_hip_yaw | 150.0 | 2.62 |
> | *_hip_roll | 150.0 | 2.62 |
> | *_hip_pitch | 200.0 | 3.49 |
> | *_knee | 200.0 | 3.49 |
> | torso | 200.0 | 3.49 |
> 
> **damping**<br>
> | Joint | Rad | Deg |
> |-|-|-|
> | *_hip_yaw | 5.0 | 0.087 |
> | *_hip_roll | 5.0 | 0.087 |
> | *_hip_pitch | 5.0 | 0.087 |
> | *_knee | 5.0 | 0.087 |
> | torso | 5.0 | 0.087 |
> 
> **velocity_limit**<br>
> | Rad | Deg |
> |-|-|
> | 100.0 | 5729.58 |

> ```python
> actuators:k
>   legs:
>     class_type: omni.isaac.lab.actuators.actuator_pd:ImplicitActuator
>     joint_names_expr:
>     - .*_hip_yaw
>     - .*_hip_roll
>     - .*_hip_pitch
>     - .*_knee
>     - torso
>     effort_limit: 300
>     velocity_limit: 100.0
>     stiffness:
>       .*_hip_yaw: 150.0
>       .*_hip_roll: 150.0
>       .*_hip_pitch: 200.0
>       .*_knee: 200.0
>       torso: 200.0
>     damping:
>       .*_hip_yaw: 5.0
>       .*_hip_roll: 5.0
>       .*_hip_pitch: 5.0
>       .*_knee: 5.0
>       torso: 5.0
>     armature: null
>     friction: null
> ```
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/8b7b4c58-6545-4683-8c92-7390d4a8658d" /><br>
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/bbc51792-3fb5-42dd-bd14-51229ea2c8ea" /><br>

## Verify Joint Configuration
1. **Play**를 클릭하여 시뮬레이션을 실행하세요.
2. **Window > Script Editor**를 클릭하여 script editor를 열어주세요.
3. 다음 명령어를 입력하고 **Run**을 클릭하여 snippet을 실행하세요.
> ```python
> from isaacsim.core.prims import SingleArticulation
> 
> prim_path = "/World/h1"
> prim = SingleArticulation(prim_path=prim_path, name="h1")
> print(prim.dof_names)
> print(prim.dof_properties)
> ```
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/bd7517c4-2b98-4cb1-a4cd-36259e982a36" /><br>

콘솔 출력 값은 radian 단위입니다.<br>
각 행은 첫 번째 목록과 동일한 순서로 나열된 joint에 대한 값입니다.<br>
각 행의 마지막 네 가지 값, 즉 maxVelocity, maxEffort, stiffness, damping을 각각 확인합니다.

## Add IMU Sensor
IMU 센서를 사용하여 신체 프레임의 linear acceleration, angular velocity 및 orientation을 구합니다. 평지 지형 정책에는 골반 링크의 linear acceleration, angular velocity 및 gravity vector가 필요합니다. 이러한 값을 계산하려면 pelvis link에 IMU 센서를 추가해야 합니다.<br>
<br>

1. `/h1/pelvis`를 클릭하고 **Create > Isaac > Sensors > Imu Sensor**를 클릭하여 센서를 추가하세요.
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/55ba3a12-28b4-41e7-bad9-2c36a34f3021" /><br>
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/11e6c745-4eed-4a50-a11c-d86c7deeb849" /><br>

## Set up ROS 2 Node for the H1 Humanoid Robot
ROS 2 노드는 관측치를 publish하고 Isaac Sim으로부터 action을 receive합니다.<br>
환경 정의 파일에 명시된 대로 관측치에는 다음 정보가 필요합니다:
- Body frame linear velocity
- Body frame angular velocity
- Body frame gravity vector
- Command (linear and angular velocity)
- Relative joint position
- Relative joint velocity
- Previous Action
<br>
IMU 데이터를 처리하여 body frame linear acceleration, angular velocity 및 gravity vector를 얻을 수 있습니다. 이 명령은 로봇의 원하는 linear acceleration와 angular velocity이며, ROS 2 twist message에서 가져올 수 있습니다. 상대적인 관절 위치와 속도는 Isaac Sim joint state topic에서 계산할 수 있습니다. 이전 작업은 마지막 반복에 적용된 작업이며 정책 노드에서 추적할 수 있습니다.

### Create an On Demand OmniGraph
1. **Stage**에서 오른쪽 클릭 후 **Create > Scope**를 클릭하여 Scope를 생성하세요.<br>생성 된 Scope의 이름을 Graph로 변경하세요.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/24734d6d-6a93-4ab0-956b-6d5c5fe9973b" />

2. **Stage**에서 오른쪽 클릭 후 **Create > Visual Scripting > ActionGraph**를 클릭하여 ActionGraph를 생성하세요.<br>생성 된 ActionGraph의 이름을 ROS_Imu로 변경하세요.<br>ActionGraph를 "Graph" scope에 드래그하여 Graph 하위로 이동하세요.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/7325298b-579f-40e9-9503-ddaed9125921" />

3. ActionGraph를 클릭하고 Property에서 **Raw USD Properties**의 `pipelineStage`를 `pipelineStageOnDemand`으로 설정하세요.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/5e307726-44ac-4038-b559-812b63f650e2" />

이렇게 하면 Isaac Sim physics 단계에서 ActionGraph 노드가 실행됩니다.

### Create Imu Publisher Node
이 노드는 신체 프레임 linear acceleration, angular velocity 및 orientation을 포함하는 ROS 2에 IMU 데이터를 publish합니다.<br>

1. ActionGraph를 오른쪽 클릭하고 **Open Graph**를 클릭하여 ActionGraph를 여세요.
2. 다음과 같이 ActionGraph를 구성하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/4e0fbbb2-44f7-4695-b108-f8c6300c006c" /><br>
> 
> - `On Physics Step`: 이 노드는 Isaac Sim physics steps에서 트리거되어 전체 그래프를 실행합니다.<br>
> - `ROS2 Context`: 이 노드는 ROS 2 노드에 대한 context를 생성합니다.<br>
> - `ROS2 QoS Profile`: 이 노드는 ROS 2 노드의 QoS profile을 설정합니다.<br>
> - `Isaac Read IMU Node`: 이 노드는 Isaac Sim에서 IMU 데이터를 읽습니다.<br>
> - `Isaac Read Simulation Time`: 이 노드는 Isaac Sim에서 simulation time을 읽습니다.<br>
> - `ROS2 Publish IMU`: 이 노드는 `Isaac Read IMU Node` 노드와 `Isaac Read Simulation Time` 노드를 source로 사용하여 ROS 2에 IMU 데이터를 publish합니다.<br>

3. 다음과 같이 노드를 설정하세요.
> - `Isaac Read IMU Node` 노드에서 `IMU Prim`을 `/h1/pelvis/Imu_Sensor`으로 설정하세요.
> - `Isaac Read IMU Node` 노드에서 `Read Gravity`를 체크 해제하세요. pelvis link에서 gravity vector를 읽지 않습니다.
> - `Read Simulation Time` 노드에서 `Reset on Stop`를 체크하세요. 시뮬레이션이 중지될 때 시뮬레이션 시간을 재설정합니다.

### Create Joint State Publisher and Subscriber Nodes
이 노드는 oint states를 ROS 2에 publish하며, 여기에는 joint names, positions 및 velocities가 포함되어 있으며 Isaac Sim의 joint state commands를 subscribe합니다.<br>

1. 새로운 ActionGraph를 만들고 이름을 `ROS_Joint_States`으로 변경하세요.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/074bbbdd-aa56-4fee-abce-db2c0b615b83" />

2. ActionGraph를 클릭하고 Property에서 **Raw USD Properties**의 `pipelineStage`를 `pipelineStageOnDemand`으로 설정하세요.
3. 다음과 같이 ActionGraph를 구성하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/36bcfca8-a8c0-4eaf-a53b-d10dc8528e09" /><br>
> - `On Physics Step`: 이 노드는 Isaac Sim physics steps에서 트리거되어 전체 그래프를 실행합니다.<br>
> - `ROS2 Context`: 이 노드는 ROS 2 노드에 대한 context를 생성합니다.<br>
> - `ROS2 QoS Profile`: 이 노드는 ROS 2 노드의 QoS profile을 설정합니다.<br>
> - `ROS2 Subscribe Joint State`: 이 노드는 external policy node의 joint states commands을 subscribe합니다.<br>
> - `ROS2 Publish Joint State`: 이 노드는 Isaac Sim의 현재 joint states를 ROS 2에 publish합니다.<br>
> - `Isaac Read Simulation Time`: 이 노드는 Isaac Sim에서 simulation time을 읽습니다.<br>
> - `Articulation Controller`: 이 노드는 Subscribe joint States node에서 joint state commands을 실행합니다.<br>

4. 다음과 같이 노드를 설정하세요.
> - `ROS2 Publish Joint State` 노드에서 `Target Prim`을 `/h1`으로 설정하세요.
> - `ROS2 Publish Joint State` 노드에서 `Topic Name`을 `/joint_states`으로 설정하세요.
> - `ROS2 Subscribe Joint State` 노드에서 `Topic Name`을 `/joint_command`으로 설정하세요.
> - `Articulation Controller` 노드에서 `Target Prim`을 `/h1`으로 설정하세요.
> - `Isaac Read Simulation Time` 노드에서 `Reset on Stop`을 체크하세요.

## Publish ROS Clock and Set Up Environment
이제 asset이 설정되었으니 로봇을 배치하고 물리적 설정을 구성한 후 ROS time을 publish할 시뮬레이션 시나리오를 만드세요.

### Setup Simulation Scenario
1. **File > New**를 눌러 새로운 파일을 만들고, content browser에서 **Isaac Sim/Environments/Simple_Warehouse**에서 **warehouse.usd**를 Stage로 드래그하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/19a458aa-f06b-424e-9bc5-a02de2cb62a4" />

> [!NOTE]
> **warehouse.usd**를 불러올 때 스트리밍 클라이언트가 멈출 시 스트리밍 클라이언트를 재실행하세요.

2. content browser에서 위에서 만들었던 **running_a_reinforcement_learning_policy_through_ros2_and_isaac_sim.usd**를 Stage로 드래그하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/f9a872de-70d7-4274-b041-2105d361777b" />

3. `h1`의 Z값을 `1.1`으로 설정하여 ground 위로 올라오게 합니다.

4. Stage에서 오른쪽 클릭을 하고 **Create > Physics > Physcis Scene**을 클릭하여 `Physics Scene`을 생성하세요.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/b141a1d1-1e95-4251-aeb4-031967abe3f5" /><br>
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/0d77e714-16f7-41ac-8b7a-de67007d95e8" /><br>

5. `Physics Scene`을 클릭하고 Property에서 `Time Steps Per Second`을 `200`으로 설정하세요.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/7867c500-229a-4025-b648-15f3d57965a9" />

6. `Physics Scene`을 클릭하고 Property에서 다음을 설정하세요.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/866b77ff-24de-4269-8bb0-6b5388aa7b0b" /><br>
> 
> - `Physics Scene`을 클릭하고 Property에서 `Time Steps Per Second`을 `200`으로 설정하세요.
> - `Enable GPU Dynamics`를 체크 해제하세요.
> - `Broadphase Type`를 `MBP`로 설정하세요.

### Setup ROS 2 Clock Publisher
1. 새로운 ActionGraph를 생성하고 이름을 `ROS_Clock`으로 변경하세요.
2. ActionGraph를 클릭하고 Property에서 **Raw USD Properties**의 `pipelineStage`를 `pipelineStageOnDemand`으로 설정하세요.
3. 다음과 같이 ActionGraph를 구성하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/53e7491c-ea01-40ec-b633-544c61bf00ba" /><br>
> 
> - `On Physics Step`: 이 노드는 Isaac Sim physics steps에서 트리거되어 전체 그래프를 실행합니다.<br>
> - `ROS2 Context`: 이 노드는 ROS 2 노드에 대한 context를 생성합니다.<br>
> - `ROS2 QoS Profile`: 이 노드는 ROS 2 노드의 QoS profile을 설정합니다.<br>
> - `Isaac Read Simulation Time`: 이 노드는 Isaac Sim에서 simulation time을 읽습니다.<br>
> - `ROS2 Publish Clock`: 이 노드는 ROS 2 clock을 ROS 2에 publish합니다.<br>

4. 다음과 같이 노드를 설정하세요.
> - `Isaac Read Simulation Time` 노드에서 `Reset on Stop`을 체크하세요.

## Run ROS 2 Policy
asset이 설정되면 ROS 2 policy을 실행할 수 있습니다. ROS 2 workspace을 구축하고 `setup.bash` 파일을 source합니다.<br>
<br>

1. PyTorch가 설치된 환경에서 새로운 터미널에서 다음 명령을 실행하여 `h1_fullbody_controller` ROS 2 패키지를 실행합니다:
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
```bash
ros2 launch h1_fullbody_controller h1_fullbody_controller.launch.py
```
> [!NOTE]
> 이 ROS 2 패키지는 위에 publish한 ROS message와 flat terrain locomotion policy을 사용하여 observations 및 actions을 계산합니다. command velocities가 receive되지 않으면 로봇은 가만히 서서 균형을 유지합니다. 시뮬레이션을 시작하기 전에 ROS 2 policy를 반드시 시작해야 하며, 그렇지 않으면 로봇이 넘어질 것입니다.

2. **Play**를 클릭하여 시뮬레이션을 시작하세요.
3. 새로운 터미널에서 다음 명령을 실행하여 Twist messages를 publish하세요.
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
> [running-a-reinforcement-learning-policy-through-ros2-and-isaac-sim_2.webm](https://github.com/user-attachments/assets/0c140722-a35a-41e7-8921-ed61e5800901)








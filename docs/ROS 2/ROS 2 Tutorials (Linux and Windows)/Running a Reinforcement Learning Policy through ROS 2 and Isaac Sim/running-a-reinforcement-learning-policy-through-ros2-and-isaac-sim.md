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
Set Up Robot Joint Configurations은 [Tutorial 13: Rigging a Legged Robot for Locomotion Policy](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup_tutorials/tutorial_rig_legged_robot.html#isaac-sim-app-tutorial-rig-legged-robot)의 내용을 진행합니다.

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
> `joint_pos`, `joint_vel`는 radian이기에 `Target Velocity`에 넣을 때 degree로 변환해서 넣어야 합니다.

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

joint state API 값이 재설정되지 않도록 하려면 로봇 상태를 정지 상태로 재설정하지 않도록 시뮬레이션 설정을 변경해야 합니다.
1. **Edit > Preferences**를 클릭하세요.
2. 왼쪽 카테고리에서 **Physics**를 클릭하세요.
3. **Reset Simulation on Stop**를 체크 해제하세요.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/17e78913-81e8-46cc-b2fd-cb437ecd061a" /><br>




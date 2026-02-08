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
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/f4451ce5-f89f-4a56-acbe-012ab98b176d" />

2. 

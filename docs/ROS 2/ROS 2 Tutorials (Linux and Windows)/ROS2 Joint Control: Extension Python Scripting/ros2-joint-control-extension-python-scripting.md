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
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
```bash
ros2 run isaac_tutorials ros2_publisher.py
```
> [ros2_joint_control_extension_python_scripting.webm](https://github.com/user-attachments/assets/97260aca-c1e6-4e62-a4d6-4902d955169e)


## Graph Shortcut







## Add Joint States in Extension







## Position and Velocity Control Modes










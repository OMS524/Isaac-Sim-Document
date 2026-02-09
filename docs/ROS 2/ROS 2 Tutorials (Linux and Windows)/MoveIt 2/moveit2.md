# MoveIt 2
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

## Running MoveIt 2
1. **Window > Examples > Robotics Examples**로 이동하세요.<br>**Robotics Examples** 탭을 클릭하세요.<br>**ROS2 > MoveIt > Franka MoveIt**를 누르고 우측에서 **Load Sample Scene**을 클릭하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/3d3937e9-874a-4996-8c72-0270b842a898" /><br>

2. 새로운 터미널에서 다음 명령을 실행하여 MoveIt 2을 실행합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch isaac_moveit isaac_moveit.launch.py
> ```

3. RViz2에서 MotionPlanning에서 `Planning Group`을 `hand`, `Goal State`를 `open`으로 설정합니다.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/e3f590cd-6153-48cf-831f-aa6efa68c835" /><br>

4. **Plan**을 클릭합니다.
> [moveit2_1.webm](https://github.com/user-attachments/assets/e24bad34-748e-4e1c-9679-463b7516849a)

5. **Execute**을 클릭합니다.
> [moveit2_2.webm](https://github.com/user-attachments/assets/ea6ca6c2-03a1-4f5c-aefb-37489abe4cfb)

> [!NOTE]
> 일부 시스템에서는 **Goal State**를 **close**를 선택하면 실행이 실패/중단되고, 지연 후 또는 다음 실행 시에 해당 작업이 실행됩니다.

6. `Planning Group`를 `panda_arm`, `Goal State`를 `<random_valid>`으로 설정합니다.
> <img width="501" height="425" alt="image" src="https://github.com/user-attachments/assets/6129c92e-3be9-462f-beb3-9a21a0642ecf" />

7. **Plan & Execute**을 클릭합니다.
> [moveit2_3.webm](https://github.com/user-attachments/assets/54fa3802-5a9e-45e6-afb5-d264ae38bb51)

## TroubleShooting
Rviz 창에 로봇이 있어야 할 위치에 대한 검은색 화면이 표시되는 경우 mesa driver를 업데이트할 수 있습니다. 새 터미널에서 다음 명령을 실행합니다.
> ```bash
> # update mesa driver
> sudo apt update
> sudo apt install -y software-properties-common
> sudo add-apt-repository ppa:kisak/kisak-mesa
> sudo apt install -y mesa-utils
> sudo apt -y upgrade
> ```










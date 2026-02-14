# ROS 2 Python Custom Messages
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

## Using Custom Messages with Python
Isaac Sim에서 `rclpy`를 사용하려면 패키지를 `Python3.11`로 빌드해야 합니다. 자신만의 패키지를 만들고 ROS 2 workspace로 빌드할 수 있습니다.<br>
<br>
Isaac Sim은 `Python 3.11`만 지원합니다. 빌드된 패키지는 `Python 3.11`로 빌드된 경우 Isaac Sim에서 `rclpy`와 함께 직접 사용할 수 있습니다.<br>
<br>
워크플로우를 시연하기 위해 튜토리얼에서는 [Isaac Sim ROS Workspace](https://github.com/isaac-sim/IsaacSim-ros_workspaces) 저장소의 일부인 custom_message 패키지를 사용합니다. 이 저장소에는 다음과 같은 정의의 `custom_message/msg/SampleMsg.msg` 아래에 사용자 지정 메시지가 포함되어 있습니다:
```text
std_msgs/String my_string
int64 my_num
```
<br>

Isaac Sim과 함께 사용할 수 있는 ROS 2 custom message packages를 만들려면 Isaac Sim ROS Workspace 폴더의 `humble_ws/src` 또는 `jazzy_ws/src` 아래에 패키지를 배치할 수 있습니다. 그런 다음 `./build_ros.sh`를 실행하고 작업 공간을 소싱한 후 Isaac Sim을 실행합니다. 사용자 지정 패키지 설치 트랙에 따라 ROS 설치 가이드의 단계를 완료했는지 확인합니다.

### Script Editor
> [!NOTE]
> 환경이 Container로 Isaac Sim을 실행하기 때문에 **Standalone Python Scripts** 방법으로 진행합니다.<br>
> **Script Editor**의 방법은 [Script Editor](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_custom_message_python.html#using-custom-messages-with-python)를 참고하세요.

### Standalone Python Scripts
1. 워크스페이스 원하는 곳에 `ros2_custom_message.py` 파일을 만든 후 다음 코드를 입력합니다.
```python
import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": True})

import omni
from isaacsim.core.utils.extensions import enable_extension

# enable ROS2 bridge extension
enable_extension("isaacsim.ros2.bridge")

# Make the rclpy imports
import rclpy
from custom_message.msg import SampleMsg

# Create message
sample_msg = SampleMsg()

# assign data in the string part and integer part of the message
sample_msg.my_string.data = "hello from Isaac Sim!"
sample_msg.my_num = 23

print("Message assignment completed!")
```

2. 다음 명령을 실행합니다.
- ROS2 워크스페이스안에 Dockerfile 안에 파이썬 312로 되어 있다면 아래 명령어를 실행
```bash
cd ~/IsaacSim-ros_workspaces/
git checkout IsaacSim-5.1.0
```

- `~/IsaacSim-ros_workspaces/build_ros.sh` 파일 맨 마지막에 아래 내용 추가
```bash
# After build complete, create runtime compat symlinks
mkdir -p /workspace 2>/dev/null || true
ln -sfn "${SCRIPT_DIR}/build_ws/${ROS_DISTRO}/${ROS_DISTRO}_ws" "/workspace/${ROS_DISTRO}_ws" 2>/dev/null || true
ln -sfn "${SCRIPT_DIR}/build_ws/${ROS_DISTRO}/${ROS_DISTRO}_ws" "/workspace/humble_ws" 2>/dev/null || true
ln -sfn "${SCRIPT_DIR}/build_ws/${ROS_DISTRO}/isaac_sim_ros_ws" "/workspace/build_ws" 2>/dev/null || true
```

- ROS2 워크스페이스에서 파이썬 3.11로 빌드
```bash
cd ~/IsaacSim-ros_workspaces/
./build_ros.sh
```
> [!NOTE]
> `./build_ros.sh` 실행 시 오류가 날 경우 해당 오류 패키지를 잠시 다른 위치로 이동하여 다시 빌드하고 원래 위치로 이동하는 걸 추천합니다.
> `Successfully copied 4.29GB to /home/oms/IsaacSim-ros_workspaces/build_ws/humble/humble_ws`와 같은 문구가 나오면 빌드 성공입니다.

- 컨테이너에서 소싱하기 위한 python3 경로 설정
```bash
export PATH=/isaac-sim/kit/python/bin:$PATH
```

- 컨테이너에서 소싱
```bash
source /IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local_setup.bash
source /IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/setup.bash
```

- 컨테이너에서 실행
```bash
./python.sh {path}/ros2_custom_message.py
./python.sh /IsaacSim-ros_workspaces/tutorials/ros2_tutorials/ros2_python_custom_messages/ros2_custom_message.py
```








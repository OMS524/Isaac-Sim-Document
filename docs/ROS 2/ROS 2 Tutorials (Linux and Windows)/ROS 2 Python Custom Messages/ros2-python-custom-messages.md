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
1. **Window > Script Editor**로 이동하여 다음 코드를 입력합니다.
> ```python
> import rclpy
> from custom_message.msg import SampleMsg
> 
> # Create message
> sample_msg = SampleMsg()
> 
> # assign data in the string part and integer part of the message
> sample_msg.my_string.data = "hello from Isaac Sim!"
> sample_msg.my_num = 23
> 
> print("Message assignment completed!")
> ```
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/503dea0f-483a-48fe-934c-aeccc8cdd563" />

2. **Run**을 클릭합니다.








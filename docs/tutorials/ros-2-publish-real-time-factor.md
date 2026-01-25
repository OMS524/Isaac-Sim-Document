# ROS 2 Publish Real Time Factor (RTF)
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

## Learning Objectives
이 튜토리얼에서는 Isaac Sim의 Real Time Factor(RTF)를 ROS2 Float 32 메시지로 publish 하는 방법을 설명합니다.

## Getting Started
**Extension Manager** 창에서 **Window > Extensions** 으로 이동하여 **isaacsim.ros2.bridge** Extension을 활성화합니다.

<img width="311" height="457" alt="image" src="https://github.com/user-attachments/assets/2af677f3-43ce-4c48-a84a-920b9e44cb7a" />

<img width="1199" height="737" alt="image" src="https://github.com/user-attachments/assets/6b59364b-c63e-4a3f-b657-1d40206285b6" />

## Publish RTF
RTF는 시뮬레이션이 실시간과 비교하여 얼마나 빠르거나 느린지를 나타냅니다. 이는 프레임당 RTF = simulated_elapsed_time / real_elapsed_time으로 계산됩니다. RTF가 1보다 크면 시뮬레이션 시간이 wall clock time보다 빠르게 실행됩니다. RTF가 1보다 작으면 시뮬레이션이 실시간보다 느리게 실행됩니다.

1. **Tools > Robotics > ROS 2 OmniGraphs > Generic Publisher**로 이동합니다. 매개변수 팝업 창이 나타납니다.
2. **Publish RTF as Float32**로 선택하고 확인을 클릭합니다.

<img width="321" height="168" alt="image" src="https://github.com/user-attachments/assets/311d18b1-5033-487b-8503-f1f4049100de" />

3. 새로운 Action graph는 일반 ROS2 Publisher 노드에 연결된 Isaac Real Time Factor 노드로 생성되며, 이 노드는 std_msgs/msg/Float32 ROS 메시지를 publish 하도록 설정되어 있습니다.

4. /Graph/ROS_GenericPub에서 찾은 Action Graph prim을 선택합니다. 마우스 오른쪽 버튼을 클릭하고 Open Graph를 선택합니다. 자동 생성된 그래프는 다음과 일치해야 합니다

<img width="372" height="410" alt="image" src="https://github.com/user-attachments/assets/b0887c90-a8b3-4f03-9820-80beccc0527d" />

<img width="601" height="419" alt="image" src="https://github.com/user-attachments/assets/5dbe60b7-5964-4e91-873d-5f1afe76cbb3" />

5. 시뮬레이션을 시작하려면 Play를 클릭하세요.

6. ROS2 소스 터미널에서 다음 명령을 실행하여 Isaac Sim에서 게시된 RTF 값을 확인합니다.
```bash
ros2 topic echo /topic
```
unloaded system의 경우 RTF는 1.0에 가까워야 합니다.

[ROS 2 Publish Real Time Factor (RTF)_1.webm](https://github.com/user-attachments/assets/5045f678-dece-426c-a276-fc049aa88e33)








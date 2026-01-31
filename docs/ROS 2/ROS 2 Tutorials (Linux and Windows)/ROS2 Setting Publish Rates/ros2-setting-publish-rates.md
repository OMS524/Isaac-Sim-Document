# ROS2 Setting Publish Rates
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

## Setting Publish Rates with OmniGraph
Action Graph는 모든 시뮬레이션 프레임에 체크되므로 OmniGraph 노드는 시뮬레이션 속도의 요인에 바인딩됩니다.<br>
이 튜토리얼에서는 이러한 시뮬레이션 요인에서 게시 ROS2 노드를 구성하는 방법을 설명합니다.

## Isaac Simulation Gate Node
이 섹션에서는 정의된 대로 특정 프레임 수마다 OmniGraph를 체크하는 데 사용할 수 있는 Isaac Simulation Gate 노드를 보여줍니다.<br>
이 OmniGraph 노드로 IMU publisher가 설정되어 있습니다.<br>

다음 명령어를 통해 Isaac Sim을 실행하세요.<br>
```bash
./runheadless.sh
```
다음 명령어를 통해 Isaac Sim을 Streaming 하세요.<br>
```bash
./docker/isaacsim-webrtc-streaming-client-1.1.5-linux-x64.AppImage
```

1. turtlebot simple room scene을 열어보세요.<br>Content browser에서 **Sim>Samples>ROS2>Scenario>turtlebot_tutorial.usd**를 Stage로 드래그하세요.
<img width="700" alt="image" src="https://github.com/user-attachments/assets/e551ea6b-04ea-49fa-9ce7-0731e41077af" /><br>

2. `/World/Turtlebot3_burger/base_link/imu_link`의 prim을 선택한 다음, **Create > Sensors > Imu Sensor**로 이동하여 IMU 센서를 생성합니다.<br>Imu 센서가 `imu_link` prim 아래에 생성되었는지 확인합니다.
<img width="500" alt="image" src="https://github.com/user-attachments/assets/7d3a69cc-18fc-461e-beb2-54ae058f97ba" /><br>

3. `/World/Turtlebot3_burger/base_link/imu_link`의 prim 내부에 새로운 Action Graph를 생성하고 ROS_IMU라고 명명합니다<br>(그래프의 배치는 자동 ROS 2 네임스페이스 생성에 중요합니다).<br>이를 위해 `/World/Turtlebot3_burger/base_link/imu_link`의 prim을 선택한 다음, **Window > Graph Editors > Action Graph**로 이동하여 Action Graph를 생성합니다.
<img width="500" alt="image" src="https://github.com/user-attachments/assets/858fa48f-c255-4ac4-a8a4-992a50368625" /><br>

4. simulation gate 노드를 포함한 IMU용 그래프를 다음과 같이 구성하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/ed58911a-ff70-4630-8867-5a8482b160e8" /><br>
>
> 각 노드에 대해 다음 속성을 설정합니다:<br>
> - **Isaac Simulation Gate** 노드의 Property 탭에서:
> > step을 `2`로 설정합니다.<br>
> > step 크기가 `2`라는 것은 downstream 노드가 다른 프레임마다 tick된다는 것을 의미합니다.<br>
> - **Isaac Read IMU Node** 노드의 Property 탭에서:
> > IMU Prim에 `/World/Turtlebot3_burger/base_link/imu_link/Imu_Sensor`를 추가합니다.<br>
> - **ROS2 Publish Imu** 노드의 Property 탭에서:
> > frameId를 `imu_link`로 설정합니다.<br>
> > 이는 주행거리 설정에서 생성한 TF publisher가 이미 publish하고 있는 TF 트리에 사용된 `imu_link` 프레임과 일치합니다.<br>
















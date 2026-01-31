# ROS 2 Quality of Service (QoS)
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

## Setting QoS Profile for ROS 2 OmniGraph Nodes
다음 명령어를 통해 Isaac Sim을 실행하세요.<br>
```bash
./runheadless.sh
```
다음 명령어를 통해 Isaac Sim을 Streaming 하세요.<br>
```bash
./docker/isaacsim-webrtc-streaming-client-1.1.5-linux-x64.AppImage
```

1. **Tools > Robotics > ROS 2 OmniGraphs > Generic Publisher**로 이동합니다.<br>**Generic Publisher Graph**를 위해**Publish String**를 선택하고 **OK**를 클릭합니다.
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/8957cba7-2625-4ddf-b022-69dfd19ee596" /><br>

2. 새로 생성된 Graph 프림을 prim합니다.<br>**ROS_GenericPub**을 선택하고 마우스 오른쪽 버튼을 클릭한 다음 **Open Graph**를 선택합니다.
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/09cf5f54-d1e3-40cd-966a-3e3a65cb79d0" /><br>
>
> ROS2 Publisher와 같은 모든 ROS 2 OmniGraph 노드에는 qosProfile string 입력이 포함되어 있습니다. 이 입력은 JSON string로 형식화되어 있습니다.<br>
> publishers 및 subscriptions에 대한 기본 QoS 설정에 대한 JSON string의 예는 아래에 나와 있습니다.<br>
> > ```python
> > {
> >     "history": "keepLast",
> >     "depth": 10,
> >     "reliability": "reliable",
> >     "durability": "volatile",
> >     "deadline": 0.0,
> >     "lifespan": 0.0,
> >     "liveliness": "systemDefault",
> >     "leaseDuration": 0.0
> > }
> > ```
> JSON string이 유효하려면 Depth를 양의 정수로 설정해야 하며 deadline, lifespan, leaseDuration을 float로 설정해야 합니다.<br>
> <br>
> 유효한 JSON string을 가진 모든 ROS 2 OmniGraph 노드의 qosProfile 입력을 직접 설정할 수도 있지만, ROS2 QoS Profile 노드를 사용하여 이 string을 자동으로 생성하고 출력을 여러 ROS 2 publisher 또는 subscriber 노드에 연결할 수도 있습니다.

4. Action Graph 창에서 ROS2 QoS Profile 노드를 추가하고 아래와 같이 연결합니다.<br>createProfile 입력에는 여러 개의 사전 설정된 QoS profile이 포함되어 있습니다.<br>다른 입력은 QoS 정책으로, custom QoS profil을 생성하도록 개별적으로 설정할 수 있습니다.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/4cc65af5-cf86-4a9c-837d-96d932492273" /><br>

5. **ROS2 QoS Profile**에서 createProfile을 Sensor Data로 설정한 다음 **Play**을 클릭하여 시뮬레이션을 시작합니다.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/a945ee77-234b-465f-bde3-63fef8855a6c" /><br>

> [!NOTE]
> UI가 새 값으로 업데이트되지 않으면 노드 외부를 클릭한 다음 다시 클릭해야 할 수도 있습니다.

6. 새로운 터미널에서 다음 명령을 실행하여 해당 topic에 대한 QoS settings을 가져옵니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 topic info /topic -v
> ```
> QoS 프로파일의 출력은 Isaac Sim에서 정의한 것과 일치해야 합니다.<br>
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/9e0f7aa9-4c7d-401a-8ee2-474aae531dab" /><br>

> [!NOTE]
> 기본적으로 Fast DDS(이전의 Fast RTPS)는 Depth를 저장하지 않으므로 Depth policy이 알 수 없는 것으로 나타날 수 있습니다.<br>
> Cyclone DDS를 사용하여 Isaac Sim 및 ROS2 노드를 실행하여 Depth info를 검색해 보세요.<br>
> Cyclone DDS로 전환한 후에도 Depth policy이 여전히 알 수 없는 것으로 표시된다면, 이는 하드웨어 구성과 관련이 있을 수 있습니다.<br>

### Creating Static Publishers














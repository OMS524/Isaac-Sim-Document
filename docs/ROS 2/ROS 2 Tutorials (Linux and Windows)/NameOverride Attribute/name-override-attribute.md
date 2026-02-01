# NameOverride Attribute
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

## Setting up the NameOverride Attribute
Joint State 또는 `TF` publisher를 설정할 때, 기본 이름은 ROS link name을 publish하는 데 사용됩니다.<br>
경우에 따라 prim name이 ROS stack에서 예상하는 관례와 일치하지 않을 수 있습니다.<br>
이 경우, `isaac:nameOverride` prim attribute을 사용하면 ROS를 사용하여 publish할 때 prim name을 내부적으로 재정의할 수 있습니다.<br>
<br>
계속 진행하기 전에 확장의 공동 상태 추가 섹션을 따라 장면을 설정합니다.<br>

## Adding the `isaac:nameOverride` Prim Attribute
1. 아무 joint prim이나 클릭하세요.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/c7da7767-fd90-459e-9701-89cab2d7f63f" /><br>

2. property 패널의 raw USD properties에서 **Name Override** 필드를 찾습니다.<br>이 필드가 이미 있는 경우 다음 단계를 건너뛰고 이후 다음 단계로 진행합니다. 이 필드가 없는 경우 다음 단계로 진행합니다.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/c3fae826-be5a-445d-a6eb-604d75f8a15f" /><br>

3. property 패널에서 추가를 클릭합니다. 팝업 메뉴에서 **Isaac > NameOverride**로 이동합니다. 이 property가 Prim에 적용됩니다.

4. property 패널의 **Name Override** 필드에 사용자 지정 기본 이름을 추가합니다.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/5e11f1f0-52da-478d-803c-61c8e7945a67" /><br>

5. **Play**을 클릭하고 새로운 터미널에서 다음 명령어를 실행하여 `/joint_states` topic을 에코할 때 추가한 사용자 지정 이름으로 공동 이름이 업데이트된 것을 확인할 수 있습니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 topic echo /joint_states
> ```
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/ba53b572-2b49-4334-aacb-8a4d740a6a40" /><br>
<br>
**ROS Publishers:** <br>
**ROS 2 Publish Transform Tree**와 **ROS 2 Publish Joint State** OmniGraph 노드는 주어진 prim에 대해 정의된 경우 `iaca:nameOverride` 속성에서 제공하는 이름을 자동으로 publish합니다.<br>
<br>
**ROS Subscribers:** <br>
ROS 2 Joint State Subscriber 파이프라인의 경우, 아래와 같이 **Isaac Joint Name Resolver** OmniGraph 노드를 드래그하여 파이프라인 내에 연결할 수 있습니다<br>
> <img width="750" alt="image" src="https://github.com/user-attachments/assets/9e3bd54a-866b-4334-91ab-d6265aa68c00" /><br>
<br>
**Isaac Joint Name Resolver**의 경우 Target Prim 또는 Robot Path를 `/franka`로 설정합니다.<br>
<br>
외부 ROS 2 노드에서 사용자 지정 prim 이름을 사용하여 Isaac Sim에 joint commands을 publish하면 **Isaac Joint Name Resolver** 노드가 실제 prim path를 Artulation Controller에 제공하여 명령에 따라 prim을 조작할 수 있습니다.





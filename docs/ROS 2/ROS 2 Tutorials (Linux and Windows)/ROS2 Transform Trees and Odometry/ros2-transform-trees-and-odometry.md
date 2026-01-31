# ROS2 Transform Trees and Odometry
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

## Getting Started
- **URDF Import: Turtlebot** 튜토리얼을 완료해야 합니다.
- **ROS 2 Cameras** 튜토리얼을 완료해야 합니다.

## Transform Tree Publisher
이미 ROS 2 카메라 튜토리얼을 검토하고 이미 두 대의 카메라가 무대에 있다고 가정하면, 그 카메라를 변환 트리에 추가하여 글로벌 프레임에서 카메라의 위치를 추적할 수 있도록 합시다.<br>
<br>
**ROS 2 Cameras** 튜토리얼의 [ros2_cameras.usd](/docs/ROS%202/ROS%202%20Tutorials%20(Linux%20and%20Windows)/ROS%202%20Cameras/ros2_cameras.usd)를 열어주세요.<br>

### Transform Publisher
1. 새로운 Action Graph 만들어 다음과 같이 구성합니다.<br>

<img width="500" alt="image" src="https://github.com/user-attachments/assets/ff4c18c1-d5a7-4dc1-a99f-90c9c643f59f" /><br>

2. *ROS 2 Publish Transform Tree*의 Property 탭에서 targetPrims에 `Camera_1`과 `Camera_2`를 추가합니다.<br>

<img width="500" alt="image" src="https://github.com/user-attachments/assets/b3846bae-2570-493f-a60b-9dba1acba9a3" />

3. 새로운 터미널에서 다음 명령어를 통해 두 카메라가 모두 TF 트리에 있는지 확인합니다. 뷰포트에서 카메라를 움직여 카메라의 자세가 어떻게 변하는지 확인하세요.<br>
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
```bash
ros2 topic echo /tf
```
<img width="250" alt="image" src="https://github.com/user-attachments/assets/3f4a9b91-c032-4f71-976a-6954933ee406" />

### Articulation Transforms
관절 로봇의 각 링크 변환을 얻으려면 로봇의 관절 루트를 ROS2 Publish Transform Tree 노드의 대상 Prims 필드에 추가할 수 있습니다. 관절 루트 이후의 모든 링크는 자동으로 게시됩니다.<br>

> [!IMPORTANT]
> 관절형 로봇에 대해 생성된 TF 트리가 잘못된 링크를 root link로 선택한 것을 발견하면 다음 단계를 통해 관절형 root link를 수동으로 선택합니다.<br>
> - Stage Tree의 **Raw USD Properties** 탭에서 로봇의 root prim을 선택하고 **Articulation Root** 섹션을 찾습니다. 섹션 내부 오른쪽 상단 모서리에 있는 X를 클릭하여 삭제합니다.
> - Stage Tree의 **Raw USD Properties** 탭에서 원하는 link를 선택하고 +ADD 버튼을 클릭한 다음 **Physics > Articulation Root**를 추가합니다.
> - Articulation Root를 변경한 후 파일을 저장하고 다시 로드합니다.

### Publish Relative Transforms
기본적으로 TF는 world frame을 참조합니다.<br>
Turtlebot의 `/base_link` TF가 `/World`를 기준으로 publish되어 있는지 확인할 수 있습니다.<br>
카메라와 같은 다른 것에 대한 TF를 얻으려면 부모 *parentPrim*에 해당 TF를 표시해야 합니다.<br>
*parentPrim* 필드에 *Camera_1*을 추가하고 property 변경 사이에 시뮬레이션을 중지하고 재생하면 `/base_link` TF가 이제 *Camera_1*에 상대적임을 확인할 수 있습니다.<br>

## Setting Up Odometry
로봇의 odometry를 설정하려면 odometry ROS message와 해당 transforms을 publish합니다.<br>
<br>
**URDF Import: Turtlebot** 튜토리얼의 [urdf_import_turtlebot.usd](/docs/ROS%202/ROS%202%20Tutorials%20(Linux%20and%20Windows)/URDF%20Import%3A%20Turtlebot/urdf_import_turtlebot.usd)를 열어주세요.<br>

1. 가져온 Turtlebot3 로봇의 Articulation Root가 `/World/Turtlebot3_burger`인지 확인합니다.<br>그렇지 않으면 `/World/Turtlebot3_burger/base_footprint`에서 Articulation Root를 제거하고 `/World/Turtlebot3_burger`에 추가합니다.<br>Articulation Root 섹션의 단계를 따라 Turtlebot3 로봇의 관절 루트를 변경합니다.
> (Optional Exercise)
> ROS2 Publish Transform Tree 노드의 **targetPrims** 필드에 `/World/Turtlebot3_burger`(기본값)를 추가하고, 로봇의 모든 링크(고정 또는 관절)의 TF가 `/tf` topic에 게시되는 것을 관찰합니다.

2. 새로운 Action Graph를 만들어 다음과 같이 Action Graph를 구성하세요.<br>
> <img width="750" alt="image" src="https://github.com/user-attachments/assets/dd2de0c1-a899-4acd-8a21-382cf6bcc982" /><br>
>
> - **Isaac Compute Odometry**의 Property 탭에서:
> > - chassisPrim에 `/World/turtlebot3_burger`를 추가합니다.
> > - 이 노드는 로봇의 시작 위치를 기준으로 현재 위치를 계산합니다.
> > 이 노드의 출력은 `/odom` ROS 2 topic의 publisher와 `/odom` 프레임에서 `/base_link` 프레임으로의 단일 변환을 publisher하는 TF publisher 모두에 입력됩니다.<br>
> - **ROS2 Publish Raw Transform Tree**의 Property 탭에서:
> > - childFrameId을 `base_link`으로 설정하세요.
> > - parentFrameId을 `odom`으로 설정하세요.
> > TF 트리에서 odom -> base_link 프레임을 publish합니다.
> - **ROS2 Publish Odometry**의 Property 탭에서:
> > - chassisFrameId을 `base_link`으로 설정하세요.
> > - odomFrameId을 `odom`으로 설정하세요.
> > TF 트리에서 odom -> base_link 프레임을 publish합니다.

> [!NOTE]
> **ROS2 Publish Odometry** 노드는 전체 3D 속도 정보를 publish합니다. 선형 속도와 각속도 모두 3차원(x, y, z)으로 publish되므로 로봇의 motion state를 보다 완벽하게 표현할 수 있습니다.

3. 이 시점에서 우리는 odometry 데이터를 publish하고 있으며, TF 트리는 오직 *odom -> base_link*로만 구성되어 있습니다.<br>또한 base_link 아래에 있는 관련 로봇 prim을 TF 트리에 추가하고자 합니다.<br>이를 위해, **ROS2 Publish Transform Tree** 노드를 그래프에 추가하고 위의 이전 노드들과 유사하게 Exec In, Context, Timestamp를 첨부합니다.
> <img width="750" alt="image" src="https://github.com/user-attachments/assets/a589be8b-83ae-464d-9290-7f7bf17afcf9" /><br>
>
> **ROS2 Publish Transform Tree**의 Property 탭에서:
> - parentPrim을 `/World/turtlebot3_burger/base_link`으로 설정하세요.
> - targetPrims을 다음과 같이 설정하세요.
> > - `/World/turtlebot3_burger/base_footprint`
> > - `/World/turtlebot3_burger/base_scan`
> > - `/World/turtlebot3_burger/caster_back_link`
> > - `/World/turtlebot3_burger/base_link/imu_link`
> > - `/World/turtlebot3_burger/wheel_left_link`
> > - `/World/turtlebot3_burger/wheel_right_link`

4. *odom -> base_link -> <other robot links>*로 구성된 TF 트리를 publish합니다.<br>이 다음 단계는 로봇의 실제 localization을 지정하려는 경우에만 필요합니다.<br>일반적으로 Nav2 AMCL과 같은 localization 지정용 ROS package는 global frame과 odom frame 간의 변환을 설정하는 역할을 합니다.<br>실제 localization 지정을 설정하려면 그래프에 다른 *ROS2 Publish Raw Transform Tree* 게시 노드를 추가하고 위의 이전 노드와 유사하게 Exec In, Context 및 Timestamp를 연결합니다.
> <img width="750" alt="image" src="https://github.com/user-attachments/assets/a589be8b-83ae-464d-9290-7f7bf17afcf9" /><br>
>
> **ROS2 Publish Raw Transform Tree**의 Property 탭에서:
> - childFrameId을 `odom`으로 설정하세요.
> - parentFrameId을 `world` 설정하세요.
> TF 트리에서 odom -> base_link 프레임을 publish합니다.
> - Translation 및 Rotation 필드는 (0.0, 0.0, 0.0) translation vector(XYZ)와 (1.0, 0.0, 0.0, 0.0) rotation quaternion(IJKR)의 기본값을 사용하므로 분리된 상태로 두십시오.<br>이 rotation 및 translation은 로봇의 시작 자세에 해당합니다.<br>로봇이 다른 위치에서 시작하는 경우, 해당 자세에 맞게 이러한 필드를 업데이트해야 합니다.





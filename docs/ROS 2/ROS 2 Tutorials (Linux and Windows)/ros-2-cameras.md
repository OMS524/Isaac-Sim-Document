# ROS 2 Cameras
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

## Setting Up Cameras
| Step | Screenshot | Description |
|-|-|-|
| 1 | <img src="https://github.com/user-attachments/assets/534a28c1-07c3-4b61-ba0e-572bfe966aa4"/> | Content 창에서 **Isaac Sim > Environment > Simple_Room**에서 `simple_room.usd`를 Stage 창으로 드래그 |
| 2 | <img src="https://github.com/user-attachments/assets/e011d9bf-08f6-4d9a-a3f9-fe02df035b30" width="300"/> | **Window > Viewports > Viewports 2**<br>Viewports 2 활성화 |
| 3 | <img src="https://github.com/user-attachments/assets/c2711f5f-b786-4d2d-ae8d-02b3b22bd77b" width="300"/> | **Create > Camera**<br>Camera 2대 생성 |
| 4 | <img src="https://github.com/user-attachments/assets/2b9cdcfe-f5f5-4364-9df2-5e8454eb2428" width="300"/> | Stage 창에서 생성된 카메라를 `Camera_1`, `Camera_2`로 변경 |
| 5 | <img src="https://github.com/user-attachments/assets/76616624-d713-45ed-8f4c-08de10f0871f" width="300"/> | `Camera_1`의 Translate를 `x=-3.5,y=-2.5,z=0.05` Orient를 `x=90.0,y=-50.0,z=0.0`<br>`Camera_2`의 Translate를 `x=3.5,y=-2.5,z=0.05` Orient를 `x=90.0,y=50.0,z=0.0`<br>(`Camera`의 위치와 회전은 임의의 서로 다른 위치로 하셔도 됩니다.) |
| 6 | <img src="https://github.com/user-attachments/assets/06db84f0-bcae-4875-b994-165ee4c0f1e8" width="300"/> | Viewport 상단에 Camere 버튼을 눌러<br>`Viewport 1` **Cameras > Camera_1**<br>`Viewport 2` **Cameras > Camera_2** |

## Building the Graph for an RGB Publisher
1. **Window > Action Editors > Action Graph**
2. **New Action Graph** 클릭
3. 다음과 같이 Action Graph 구성
<img width="971" height="409" alt="image" src="https://github.com/user-attachments/assets/002afdcc-1d58-46f0-a304-e16b524873c9" />

Parameter:
| Node | Input Field | Value |
|-|-|-|
| Isaac Create Render Product | cameraPrim | /World/Camera_1 |
|  | enabled | True |
| ROS 2 Camera Helper | type | rgb |
|  | topicName | rgb |
|  | frameId | turtle |

### Graph Explained
- **On Playback Tick Node**: 시뮬레이션이 "Playing"일 때 틱 생성. 이 노드로부터 틱을 수신한 노드는 시뮬레이션 단계마다 계산 기능을 실행합니다.
- **ROS 2 Context Node**: ROS 2는 미들웨어 통신에 DDS를 사용합니다. DDS는 Domain ID를 사용하여 물리적 네트워크를 공유하더라도 서로 다른 논리적 네트워크가 독립적으로 작동할 수 있도록 합니다. 같은 Domain의 ROS 2 노드는 서로 자유롭게 검색하고 메시지를 보낼 수 있는 반면, 다른 Domain의 ROS 2 노드는 그렇지 않습니다. ROS 2 컨텍스트 노드는 주어진 Domain ID로 컨텍스트를 생성합니다. 기본적으로 0으로 설정되어 있습니다. Domain ID Env Var 사용을 선택하면 현재 Isaac Sim 인스턴스를 실행한 환경에서 `ROS_DOMAIN_ID`를 가져옵니다.
- **Isaac Create Render Product**: 주어진 카메라 프림에서 렌더링된 데이터를 획득하고 그 경로를 렌더 제품 프림으로 출력하는 렌더 제품 프림 만듭니다. 명령에 따라 활성화된 필드를 확인하거나 선택 해제하여 렌더링을 활성화하거나 비활성화할 수 있습니다.
- **Isaac Run One Simulation Frame**: 이 노드는 파이프라인이 처음부터 한 번만 실행되도록 보장합니다.
- **ROS 2 Camera Helper**: publish 할 데이터 유형과 이를 publish 할 ros topic을 나타냅니다.

**Camera Helper Node**

Camera Helper Node는 사용자로부터 복잡한 후처리 네트워크를 추상화하고 있습니다.

Camera Helper Node가 연결된 상태에서 Play을 누른 후, Action Graph 창의 왼쪽 상단 모서리에 있는 아이콘을 클릭하면 Action Graph 목록에 새로운 것이 나타날 수 있습니다: `/Render/PostProcessing/SDGPipeline`.

이 그래프는 Camera Helper Node에서 자동으로 생성됩니다. 파이프라인은 렌더러로부터 관련 데이터를 가져와 처리한 다음 해당 ROS Publisher에게 보냅니다. 이 그래프는 실행 중인 세션에서만 생성됩니다. 자산의 일부로 저장되지 않으며 Stage 트리에 표시되지 않습니다.

## Verifying ROS Connection
전달되는 원시 정보를 관찰하려면 `ros2 topic echo /<topic>`을 사용합니다.

`rqt_image_view` 방법을 사용하여 깊이를 다시 시각화합니다: `ros2 run rqt_image_view rqt_image_view /depth`.


이번에 RViz2에 publish 된 이미지를 확인하려면:
1. Isaac Sim에서 Play를 눌러 시뮬레이션을 시작합니다.
2. ROS 2 소스 터미널에서 rviz2 명령을 입력하여 RViz를 엽니다.
3. Add를 눌러 By Topic 탭에서 /rgb 토픽의 Image를 추가합니다.
<img width="1199" height="880" alt="image" src="https://github.com/user-attachments/assets/5e36ed40-ba39-4048-b960-ddd8ca755698" />
<img width="1220" height="901" alt="image" src="https://github.com/user-attachments/assets/20e26496-be29-4b92-816c-6bbea6572dd1" />

## Depth and Other Perception Ground Truth Data

RGB 이미지 외에도 다음과 같은 합성 센서와 지각 정보가 모든 카메라에 제공됩니다:
- Depth
- Point Cloud
각 합성 데이터 주석기에 사용되는 단위를 관찰하려면 omni.replicator를 참조하세요.

다음 Bounding box와 Labels을 Publish 하기 전에 Isaac Sim Replicator Tutorials을 검토하여 의미적으로 주석이 달린 장면에 대해 알아보세요.
> [!NOTE]
> vision_msgs에 의존하는 BoundingBox 퍼블리셔 노드를 사용하려면 시스템에 설치되어 있는지 확인하거나 옵션 구성 및 내부 ROS 라이브러리 활성화를 시도해 보세요.
- BoundingBox 2D Tight
- BoundingBox 2D Loose
- BoundingBox 3D
- Semantic labels
- Instance Labels
각 Camera Helper Node는 하나의 유형의 데이터만 가져올 수 있습니다. Camera Helper Node의 Property 탭의 필드 유형에 대한 드롭다운 메뉴에서 노드에 할당할 `type`을 지정할 수 있습니다.
> [!NOTE]
> Camera Helper Node의 type을 지정하고 활성화한 후(즉, 시뮬레이션을 시작하고 기본 SDGPipline이 생성됨), type을 변경하고 노드를 재사용할 수 없습니다. 새 노드를 사용하거나 스테이지를 새로고침하고 수정된 유형으로 SDGPipline을 재생성할 수 있습니다.

여러 대의 카메라에 대해 여러 개의 ros topic을 게시하는 예는

Isaac Sim 콘텐츠 브라우저, **Isaac Sim>Samples>ROS2>Scenario>Turtlebot_tutorial.usd**로 이동하여 찾을 수 있습니다.




# ROS2 Setting Publish Rates
## Environment Infomation
| Item | Description |
|-|-|
| Author | 오민석 |
| Date | 2026-01-31 |
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

### Isaac Simulation Gate Node
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

<img width="300" alt="image" src="https://github.com/user-attachments/assets/7d3a69cc-18fc-461e-beb2-54ae058f97ba" /><br>

3. `/World/Turtlebot3_burger/base_link/imu_link`의 prim 내부에 새로운 Action Graph를 생성하고 이름을 ROS_IMU으로 설정합니다.(그래프의 배치는 자동 ROS 2 네임스페이스 생성에 중요합니다).<br>이를 위해 `/World/Turtlebot3_burger/base_link/imu_link`의 prim을 선택한 다음, **Window > Graph Editors > Action Graph**로 이동하여 Action Graph를 생성합니다.

<img width="300" alt="image" src="https://github.com/user-attachments/assets/858fa48f-c255-4ac4-a8a4-992a50368625" /><br>

4. simulation gate 노드를 포함한 IMU용 그래프를 다음과 같이 구성하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/ed58911a-ff70-4630-8867-5a8482b160e8" /><br>
>
> 각 노드에 대해 다음 Property를 설정합니다:<br>
> - **Isaac Simulation Gate** 노드의 Property 탭에서:
> > step을 `2`로 설정합니다.<br>
> > step 크기가 `2`라는 것은 downstream 노드가 다른 프레임마다 tick된다는 것을 의미합니다.<br>
> - **Isaac Read IMU Node** 노드의 Property 탭에서:
> > IMU Prim에 `/World/Turtlebot3_burger/base_link/imu_link/Imu_Sensor`를 추가합니다.<br>
> - **ROS2 Publish Imu** 노드의 Property 탭에서:
> > frameId를 `imu_link`로 설정합니다.<br>
> > 이는 주행거리 설정에서 생성한 TF publisher가 이미 publish하고 있는 TF 트리에 사용된 `imu_link` 프레임과 일치합니다.<br>

### Setting Publish Rates for Nodes Within SDG Pipeline
이전 섹션에서는 OmniGraph ROS2 publishing pipeline의 Isaac Simulation Gate 노드에 추가했습니다.<br>
Camera 및 RTX Lidar sensor의 경우 SDG pipeline 내에서 자동으로 구성됩니다.<br>
<br>
각 publisher의 publish rate를 수정하려면 각 ROS2 Helper 노드 내의 **frameSkipCount** 매개변수를 수정할 수 있습니다.<br>
1. `/World/Turtlebot3_burger/base_scan/ROS_LidarRTX`에 있는 Lidar Action Graph를 엽니다.<br>Ros2RTXLidarHelper 노드(`/World/Turtlebot3_burger/base_scan/ROS_LidarRTX/LaserScanPublish`)를 선택하고 Property에서 `frameSkipCount` 값을 `11`로 설정합니다.
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/d7c9caee-1b5a-4c2a-9875-c5dc1c0b18eb" /><br>
> 
> 이렇게 하면 publish 사이에 11프레임을 건너뛰고 SDG 파이프라인 내에 연결된 Isaac Simulation Gate 노드의 step 속성을 자동으로 11값으로 설정할 수 있습니다.<br>
> 11프레임을 건너뛰는 것은 매 12프레임마다 publish하는 것과 같습니다.<br>

2. 이 튜토리얼에서는 point cloud를 publish할 필요가 없으므로, point cloud에 대해 Ros2RTXLidarHelper 노드(`/World/Turtlebot3_burger/base_scan/ROS_LidarRTX/PointCloudPublish`)를 선택하고 활성화된 `enabled`를 비활성화하세요.
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/c27c947b-eec0-4158-b4ca-79ed1a797c1b" /><br>

3. `/World/ActionGraph_camera`에 있는 Camera Action Graph를 엽니다.<br>`/World/ActionGraph_camera/isaac_create_render_product_01`를 선택하여 활성화된 `enabled`을 비활성화하여 두 번째 카메라 렌더 제품을 비활성화합니다.
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/8b08634a-82d7-4490-92fc-fdcb0393165b" /><br>

4. RGB images를 위해 /World/ActionGraph_camera/ros2_camera_helper에서 `frameSkipCount` 값을 `3`으로 설정합니다.
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/c388f94b-a5b6-43e8-9483-e2234cb49a4d" /><br>
> 
> 이렇게 하면  publish 사이에 3프레임을 건너뛰고 SDG 파이프라인 내에 연결된 Isaac Simulation Gate 노드의 step 속성을 자동으로 4값으로 설정할 수 있습니다.<br>
> 3프레임을 건너뛰는 것은 매 4프레임마다 publish하는 것과 같습니다.<br>

5. 이 튜토리얼에서는 Camera1의 Depth Image를 publish할 필요가 없습니다.<br>`/World/ActionGraph_camera/ros2_camera_helper_02`에서 `enabled`을 비활성화하여 Depth Image에 대한 camera helper를 비활성화합니다.
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/42e27a71-bdf9-4cf8-8e18-c179a0dee758" /><br>

6. camera info에 대한 Camera Info Helper 노드 설정의 속성 패널인 `/World/ActionGraph_camera/ros2_camera_info_helper`에서 `frameSkipCount` 값을 `5`로 설정합니다.
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/aee978f1-3d56-4beb-a0bc-93466b9d9f0b" /><br>
> 
> 이렇게 하면 publish 사이에 5프레임을 건너뛰고 SDG 파이프라인 내에 연결된 Isaac Simulation Gate 노드의 step 속성을 자동으로 6값으로 설정할 수 있습니다.<br>
> 5프레임을 건너뛰는 것은 매 6프레임마다 publish하는 것과 같습니다.<br>

### Setting Simulation Frame Rates
특정 노드를 다양한 속도로 체크하도록 ActionGraphs를 구성했습니다.<br>
모든 ActionGraphs는 시뮬레이션 속도에 대해 정의된 최대 프레임 속도로 제한되므로 Python interface를 사용하여 이 시뮬레이션 프레임 속도를 수정할 수 있습니다.<br>
<br>
1. 스크립트 편집기에서 파이썬 코드를 실행하여 시뮬레이션 속도를 설정합니다. **Window > Script Editor**로 이동하여 script editor를 엽니다.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/5ce33a19-d094-4807-ba82-5bec97f10463" /><br>
> 시뮬레이션 속도를 설정하는 두 가지 방법이 있습니다:<br>
> <br>
> - carb 설정 변경. scene 재생 후 아래 스크립트를 실행합니다.<br>이 방법은 시뮬레이션 타임라인 실행률을 설정하는 것을 목표로 합니다.<br>이는 OnPlayBackTick 노드의 시간에 영향을 미칩니다.
> > ```python
> > # Change the carb settings. This is not persistent between when stopping and replaying
> > 
> > import carb
> > physics_rate = 60 # fps
> > carb_settings = carb.settings.get_settings()
> > carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
> > carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(physics_rate))
> > carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(physics_rate))
> > ```
> - SetTimeCodesPerSecond 및 set_target_framerate 변경.<br>이 방법은 물리학 실행 속도를 설정하는 것을 목표로 합니다.<br>이는 IsaacReadSimulationTime 노드의 시간에 영향을 미칩니다.
> > 초당 시간 코드는 장면이 재생되기 전에 한 번만 설정할 수 있습니다. 이 값을 변경하려면 먼저 장면을 다시 로드하세요.
> > ```python
> > # This must be called after a stage is loaded. Timeline must be stopped when setting SetTimeCodesPerSecond and set_target_framerate. This is persistent between stopping and replaying:
> > import omni
> > physics_rate = 60 # fps
> > 
> > timeline = omni.timeline.get_timeline_interface()
> > stage = omni.usd.get_context().get_stage()
> > timeline.stop()
> > 
> > stage.SetTimeCodesPerSecond(physics_rate)
> > timeline.set_target_framerate(physics_rate)
> > 
> > timeline.play()
> > ```

2. script editor에서 snippets(`Run(Ctrl+Enter)`)을 실행하고 시뮬레이션 속도에 미치는 영향을 확인합니다.<br>viewport show/hide menu **(eye) > Heads Up Display > FPS**로 이동하여 FPS 디스플레이를 활성화할 수 있습니다.
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/b6ae993c-f4d9-4163-b7c4-a471518e7f6f" /><br>
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/f52cb73b-bd55-4b20-ac32-4b31fba8161d" /><br>
> 
> physics_rate를 다른 값으로 수정하고 FPS 수치를 확인해 보세요.

### Checking ROS 2 Publish Rate
1. 시뮬레이션에서 **Play**를 누르세요.
2. 다음 명령어를 사용하여 각 ROS topic의 publish rate를 확인합니다:
> ```bash
> ros2 topic hz /topic_name
> ```
> <br>
> `/topic_name`은 아래에 나열된 각 sensor topic으로 대체됩니다.<br>
> <br>
> ublish rates가 추정됩니다.<br>
> 고성능 기계에서는 최대 FPS가 이전 섹션에서 설정한 물리 속도(기본값 60Hz)에 더 가깝습니다.<br>
3. 각 sensor topic에 대해 그들의 rates는 최대 시뮬레이션 FPS의 요소입니다.<br>(이전에 정의한 실행 단계에 따라 다름)
- **/clock**: 시뮬레이션 FPS(기본값 약 60Hz)와 동일한 속도로 게시합니다.
- **/imu**: sim_fps/2 (~30Hz) rate로 publish
- **/scan**: sim_fps/12 (~5Hz) rate로 publish
- **/camera_1/rgb/image_raw**: sim_fps/4 (~15Hz)로 publish
- **/camera_1/rgb/camera_info**: sim_fps/6 (~10Hz)에서 publish
이 튜토리얼의 모든 단계가 포함된 파일은<br>
Isaac Sim에서 Content browser에서 **Isaac Sim>Samples>ROS2>Scenario>Turtlebot_tutorial_multi_sensor_publish_rates.usd**를 클릭하여 열 수 있습니다.<br>
파일을 연 후에는 Setting Simulation Frame Rates 단계를 실행하여 target simulation rate를 설정하는 것을 잊지 마세요.<br>
> [!NOTE]
> `/camera_1/rgb/image_raw` topic이 예상보다 느린 rate로 publish되는 것을 관찰하면 각 이미지 메시지의 크기가 커서 네트워크 트래픽이나 DDS 대기열 관리에 병목 현상이 발생하기 때문일 수 있습니다.<br>
> publish rate를 개선하려면 렌더 제품 해상도의 차원을 줄이는 것이 좋습니다.<br>
> 이 작업은 Image Publisher `/World/ActionGraph_camera/isaac_create_render_product`에 연결된 render product 노드로 이동하여 scene을 재생하기 전에 dimensions을 수정하여 수행할 수 있습니다.

### Troubleshooting
target simulation frame rate와 많이 다른 publisg rate를 관찰하면 다음을 시도해 보세요:<br>
1. factory settings에서 Isaac Sim을 실행하여 지속적인 simulation frame rate settings을 지우세요:
> ```bash
> ./isaac-sim.sh --reset-user
> ```

2. 컴퓨터의 CPU 사용량을 확인하여 병목 현상을 식별합니다.<br>Isaac Sim의 사용량이 엄청나게 많다면 Fabric을 활성화한 상태에서 실행해 보세요:
> ```bash
> ./isaac-sim.fabric.sh --reset-user
> ```

> [!IMPORTANT]
> 위의 명령어는 실험적인 것이며 아이작 심의 모든 기능이 지원되는 것은 아닙니다.<br>
> 하지만 전반적인 성능이 더 좋을 수도 있습니다.<br>
> Fabric을 처음 실행할 때는 --reset-user flag만 사용하면 됩니다.<br>



# RTX Lidar Sensors
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

## Adding a RTX Lidar ROS 2 Bridge
1. **Create > Sensors > RTX Lidar > NVIDIA > Example Rotary 2D**

2. 생성된 Example Rotary 2D의 Prim을 */World/turtlebot3_burger/base_scan*으로 드래그하세요.<br>그 후 Example Rotary 2D의 Property 탭에서 위치를 `(0,0,0)`으로 설정합니다.

3. **Create > Sensors > RTX Lidar > NVIDIA > Example Rotary**

4. 생성된 Example Rotary의 Prim을 */World/turtlebot3_burger/base_scan*으로 드래그하세요.<br>그 후 Example Rotary의 Property 탭에서 위치를 `(0,0,0)`으로 설정합니다.

5. **Window > Graph Editors > Action Graph**에서 New Action Graph를 클릭합니다.<br>생성된 Action Graph를 *//World/turtlebot3_burger/base_scan*로 이동합니다.<br>다음과 같이 Action Graph를 구성합니다.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/1f1e749d-4cfa-4baf-a74b-2f9268d6faf7" /><br>
> **On Playback Tick**: 재생을 누른 후 다른 모든 노드를 트리거하는 노드입니다.<br>
> **ROS2 Context Node**: ROS2는 미들웨어 통신에 DDS를 사용합니다. DDS는 도메인 ID를 사용하여 물리적 네트워크를 공유하더라도 서로 다른 논리적 네트워크가 독립적으로 작동할 수 있도록 합니다. 동일한 도메인의 ROS 2 노드는 서로 자유롭게 검색하고 메시지를 보낼 수 있는 반면, 다른 도메인의 ROS 2 노드는 그렇지 않습니다. ROS2 컨텍스트 노드는 주어진 도메인 ID로 컨텍스트를 생성합니다. 기본적으로 0으로 설정되어 있습니다. Use Domain ID Env Var을 선택하면 현재 Isaac Sim 인스턴스를 실행한 환경에서 `ROS_DOMAIN_ID`를 가져옵니다.<br>
> **Isaac Run One Simulation Frame**: 성능을 향상시키기 위해 렌더링 제품 생성 파이프라인을 처음부터 한 번 실행하는 노드입니다.<br>
> **Isaac Create Render Product**: Property 탭에서 cameraPrim에 Add Target을 눌러 생성된 `Example Rotary 2D`를 선택합니다.<br>
> 다른 **Isaac Create Render Product**에서 Property 탭에서 cameraPrim에 Add Target을 눌러 생성된 `Example Rotary`를 선택합니다.<br>
> **ROS2 RTX Lidar Helper**: 이 노드는 RTX Lidar에서 레이저 스캔 메시지의 Publush를 처리합니다. input의 render product는 Isaac Create Render Product의 출력에서 얻어집니다. frameId를 `base_scan`으로 설정합니다.<br>
> 다른 **ROS2 RTX Lidar Helper**에서 input에서 type을 `point_cloud`으로 변경합니다. topicName을 `point_cloud`으로 변경합니다. 이 노드는 RTX Lidar에서 포인트 클라우드를 Publush하는 작업을 처리합니다. input의 render product는 두 번째 Isaac Create Render Product의 출력에서 얻어집니다. frameId를 `base_scan`으로 설정합니다.

> [!NOTE]
> **ROS2 RTX Lidar Helper**에서 type이 laser_scan으로 설정된 경우, RTX Lidar가 전체 스캔을 생성할 때만 LaserScan 메시지가 게시됩니다. rotary Lidar의 경우, 이는 360도 전체 회전이며, solid state Lidar 의 경우 프로필에 설정된 대로 라이다의 전체 방위각입니다. 라이다 회전 속도와 시간 스텝 크기에 따라 전체 회전 스캔을 완료하는 데 여러 프레임이 필요할 수 있습니다. 즉, 스텝 크기 1/60s를 렌더링할 때 회전 속도가 10Hz인 회전 라이다는 전체 스캔을 완료하는 데 6 프레임이 소요되므로, LaserScan 메시지는 6 프레임마다 한 번씩 게시됩니다. 고체 상태 라이다는 단일 프레임으로 전체 스캔을 완료하므로, 해당 LaserScan 메시지는 모든 프레임에 publish됩니다.
> 
> PointCloud 메시지는 **ROS2 RTX Lidar Helper**의 전체 스캔 publish 설정 값에 따라 매 프레임 또는 전체 스캔이 누적된 후에 게시됩니다.

## RViz visualization
1. Isaac Sim에서 **Play**를 눌러 시뮬레이션을 시작합니다.
2. 새로운 터미널에서 다음 명령어를 입력하여 토픽을 확인합니다 `Rviz2`를 실행합니다
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
```bash
ros2 topic list
```
3. `Rviz2`를 실행합니다.
```bash
rviz2
```
5. Rviz2에서 Fixed Frame을 `base_scan`으로 설정합니다.<br>해당 예제는 TF 구성이 안되어 있기 때문에 직접 입력해야 합니다.










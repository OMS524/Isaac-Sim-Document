# Driving TurtleBot using ROS 2 Messages
## Environment Infomation
| Item | Description |
|-|-|
| Author | 오민석 |
| Date | 2026-01-24 |
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

## 필수 조건
1. 앞서 예제 URDF Import: Turtlebot가 되어야 한다.

## Putting It Together
### 그래프 구축
1. Window-Graph Editors-Action Graph를 선택
2. New Action Graph 선택
<img width="1236" height="283" alt="image" src="https://github.com/user-attachments/assets/d3fb3959-6dda-4ba7-9bc6-a6c7cbbc81a3" />
3. 아래 그래프와 동일한 그래프 그리기
<img width="1801" height="711" alt="image" src="https://github.com/user-attachments/assets/9a3ed158-db93-4213-8287-1e792e10c330" />


### 그래프 설명
**On Playback Tick Node**

시뮬레이션이 "재생 중"일 때 Tick을 생성합니다.
이 노드에서 Tick을 수신하는 노드는 시뮬레이션 단계마다 연산 기능을 실행합니다.

**ROS2 Context Node**

ROS2는 미들웨어 통신에 DDS(Domain Distributed System)을 사용합니다.
DDS는 Domain ID를 이용하여 물리적 네트워크를 공유하더라도 서로 다른 논리 네트워크가 독립적으로 작동할 수 있도록 합니다.
동일한 도메인에 있는 ROS2 노드는 서로를 자유롭게 검색하고 메시지를 주고받을 수 있지만, 다른 도메인에 있는 ROS2 노드는 그렇지 않습니다.
ROS2 Context Node는 지정된 도메인 ID를 사용하여 컨텍스트를 생성합니다.
기본값은 0입니다.
'useDomainIDEnvVar'을 체크하면 현재 Isaac Sim 인스턴스를 실행한 환경에서 ROS_DOMAIN_ID를 가져옵니다.

ROS2 Context Node의 Property 탭에서 useDomainIDEnvVar를 체크하세요.

<img width="514" height="116" alt="image" src="https://github.com/user-attachments/assets/d2c54f2d-16e0-47a0-b6be-645345759905" />


**ROS2 Subscribe Twist Node**

트위스트 메시지를 구독합니다.

ROS2 Subscribe Twist Node의 Property 탭에서 topicName에 /cmd_vel을 입력하세요.

<img width="514" height="201" alt="image" src="https://github.com/user-attachments/assets/1828712a-fe61-4f59-9f8c-a4f763674cc1" />


**Scale To/From Stage Unit Node**

자산 또는 입력을 스테이지 단위로 변환합니다.

**Break 3-Vector Node**

Twist 구독자 노드의 출력은 선형 속도와 각속도, 두 값 모두 3차원 벡터입니다.
하지만 미분 제어기 노드의 입력은 전진 속도와 z축 회전 속도만 받으므로, 배열을 분해하여 해당 요소들을 추출한 후 미분 제어기 노드에 입력해야 합니다.

**Differential Controller Node**

이 노드는 원하는 차량 속도를 입력받아 로봇의 바퀴 속도를 계산합니다.
이를 위해서는 바퀴 반지름과 바퀴 간 거리가 필요합니다. 또한 선택적으로 속도 제한 매개변수를 입력받아 바퀴 속도를 제한할 수 있습니다.
Differential Controller Node의 Property 탭에서 다음 표를 참고해서 maxAngularSpeed, maxLinearSpeed, wheelDistance, wheelRadius를 입력하세요.
| Field | Value |
|-|-|
| Max Angular Speed | 1.0 |
| Max Linear Speed | 0.22 |
| Wheel Distance | 0.16 |
| Wheel Radius | 0.025 |
<img width="516" height="362" alt="image" src="https://github.com/user-attachments/assets/05b77cf1-c8ab-43ea-ba6e-5668913aa00c" />

**Articulation Controller Node**

이 노드는 대상 로봇에 할당되며, 움직여야 할 관절의 이름이나 인덱스를 입력받아 위치 명령 , 속도 명령 또는 힘 명령 에 따라 해당 관절을 움직입니다.
Articulation Controller Node는 On Playback Tick Node에 의해 Tick 됩니다.
따라서 새로운 Twist 메세지가 도착하지 않으면 이전에 수신한 명령을 계속 실행합니다.

turtlebot3_burger 하위의 base_footprint의 Property 탭에서 확인해보았을 때 Articulation Root API가 있을 경우
base_footprint의 Articulation Root API를 삭제 후, turtlebot3_burger에 Articulation Root API를 추가하세요.

<img width="506" height="196" alt="image" src="https://github.com/user-attachments/assets/2176b775-6bd9-4e8e-a3aa-ca1a638ad5e1" />

<img width="538" height="464" alt="image" src="https://github.com/user-attachments/assets/109eb1d8-1f66-498b-8f45-60ea4202da51" />

Articulation Controller Node의 Property 탭에서 'Add Target...'을 클릭하여 turtlebot3_burger을 추가합니다.
turtlebot3_burger은 Articulation Root API가 적용되어 있어야 합니다.

<img width="514" height="296" alt="image" src="https://github.com/user-attachments/assets/cf437cb6-944a-4299-95c7-d7de11878c6c" />

각 Constant Tokoen Node의 Property 탭에서 Value에 wheel_left_joint, wheel_right_joint를 입력하세요.

<img width="515" height="98" alt="image" src="https://github.com/user-attachments/assets/667a8461-45be-4138-bd1b-c15ca590ed57" />
<img width="515" height="98" alt="image" src="https://github.com/user-attachments/assets/03e0781a-6049-4741-a595-3ee1092b13c1" />


### ROS 연결 확인
1. Isaac Sim 재생 버튼 클릭
2. 별도의 일반 터미널에서 다음 명령어를 실행하여 /cmd_vel, /parameter_events, /rosout의 토픽 확인
```bash
ros2 topic list
```
<img width="236" height="72" alt="image" src="https://github.com/user-attachments/assets/4394cc70-4c57-4d44-8435-88704a89bf58" />

3. Isaac Sim과 ROS 2 연동
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

4. 다음 명령어를 통해 로봇을 앞으로 움직이거나 멈추는 토픽 게시
앞으로 가기
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{'linear': {'x': 0.2, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}"
```

멈추기
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}"
```
[Driving TurtleBot using ROS 2 Messages_1.webm](https://github.com/user-attachments/assets/cd3af97f-639b-4cad-b565-5f41905b6ce3)


5. 다음 명령어를 통해 teleop_twist_keyboard를 이용하여 터틀봇을 더 쉽게 이동
```bash
sudo apt-get install ros-$ROS_DISTRO-teleop-twist-keyboard
```
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
[Driving TurtleBot using ROS 2 Messages_2.webm](https://github.com/user-attachments/assets/e71c6432-b12b-4b39-9fa0-bd3e2a6857a1)







[Screencast from 01-24-2026 07:48:58 PM.webm](https://github.com/user-attachments/assets/4f04764b-d911-4b1b-b3ab-b2a5e52911b8)# Driving TurtleBot using ROS 2 Messages
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


**ROS2 Context Node**


**ROS2 Subscribe Twist Node**


**Scale To/From Stage Unit Node**


**Break 3-Vector Node**


**Differential Controller Node**
Differential Controller
<img width="516" height="362" alt="image" src="https://github.com/user-attachments/assets/05b77cf1-c8ab-43ea-ba6e-5668913aa00c" />



**Articulation Controller Node**
Articulation Controller
<img width="515" height="295" alt="image" src="https://github.com/user-attachments/assets/b41975a3-8330-4baf-a512-332202819c68" />

Constant Tokoen
<img width="515" height="98" alt="image" src="https://github.com/user-attachments/assets/667a8461-45be-4138-bd1b-c15ca590ed57" />
<img width="515" height="98" alt="image" src="https://github.com/user-attachments/assets/03e0781a-6049-4741-a595-3ee1092b13c1" />


### ROS 연결 확인
1. Isaac Sim 재생 버튼 클릭
2. 별도의 일반 터미널에서 다음 명령어를 실행하여 /cmd_vel, /parameter_events, /rosout의 토픽 확인
```bash
ros2 topic list
```
<img width="236" height="72" alt="image" src="https://github.com/user-attachments/assets/4394cc70-4c57-4d44-8435-88704a89bf58" />

3. 다음 명령어를 통해 로봇을 앞으로 움직이는 토픽 게시
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{'linear': {'x': 0.2, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}"
```

4. 다음 명령어를 통해 로봇을 멈추는 토픽 게시
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}"
```


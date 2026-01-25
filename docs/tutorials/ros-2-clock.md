# ROS 2 Clock
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

## Simulation Time and Clock
외부 ROS 2 노드가 시뮬레이션 시간과 동기화되는 경우 일반적으로 클록 토픽이 사용됩니다.
RViz2와 같은 많은 ROS 2 노드는 매개변수 use_sim_time을 사용하며,
True로 설정하면 노드가 /clock 토픽에 가입하고 게시된 시뮬레이션 시간과 동기화를 시작하도록 표시합니다.

이 매개변수를 ROS 2 실행 파일에서 설정하거나 새 ROS 2 소스 터미널에서 다음 명령을 사용하여 매개변수를 설정할 수 있습니다:
```bash
ros2 param set /node_name use_sim_time true
```

현재 실행 중인 노드로 /node_name을 교체해야 합니다.
터미널을 사용하여 설정하는 경우 매개변수를 설정하기 전에 노드가 이미 먼저 실행 중이어야 합니다.

## Running ROS 2 Clock Publisher
1. Window > Graph Editors > Action Graph로 가서 Action graph를 생성하세요.
2. 다음 사진과 같이 Action graph에 OmniGraph를 추가하세요.
<img width="678" height="512" alt="image" src="https://github.com/user-attachments/assets/c4dba303-0a9b-456d-9210-25f65b68100c" />
- On Playback Tick: 모든 시뮬레이션 프레임에서 다른 그래프 노드를 실행합니다.
- ROS 2 Context: 주어진 도메인 ID 또는 ROS_DOMAIN_ID 환경 변수를 사용하여 컨텍스트를 생성합니다.
- Isaac Read Simulation Time: 현재 시뮬레이션 시간을 가져옵니다. 참고: 기본적으로 시뮬레이션 시간은 단조롭게 증가하므로 시뮬레이션이 중지되었다가 다시 재생되더라도 시간은 계속 증가합니다. 이는 주로 시뮬레이션이 초기화될 때 시간이 뒤로 이동할 때 발생할 수 있는 문제를 방지하기 위한 것입니다. 시뮬레이션이 초기화될 때마다 시계를 0에서 시작하도록 하려면 resetOnStop을 True로 설정할 수 있습니다.
- ROS 2 Publish Clock: /clock 토픽에 시뮬레이션 시간을 publish 합니다.


# Add Noise to Camera
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

## Running the Example
1. sample script를 실행하세요.
```bash
./python.sh standalone_examples/api/isaacsim.ros2.bridge/camera_noise.py
```
위 명령어를 Docker에서 실행하게 되면 오류 몇 가지가 발생하여 순차적으로 오류 해결법을 제시한다.
<br>
**ROS2 Bridge startup failed**
<img width="924" height="415" alt="image" src="https://github.com/user-attachments/assets/ff630cfe-f6df-4078-8638-35a775f3b20e" /><br>
아래 명령어를 Docker 내에서 실행한다.
```bash
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib
```

**Augmentation cannot run, script nodes are disabled**
<img width="924" height="35" alt="image" src="https://github.com/user-attachments/assets/b446d80c-6041-4530-b721-46308650a035" /><br>
기존 실행 명령어에서<br>
`--/app/omni.graph.scriptnode/opt_in=true`<br>
`--/app/omni.graph.scriptnode/enable_opt_in=false`<br>
위 2가지를 추가하여 실행한다.
```bash
./python.sh standalone_examples/api/isaacsim.ros2.bridge/camera_noise.py \
  --/app/omni.graph.scriptnode/opt_in=true \
  --/app/omni.graph.scriptnode/enable_opt_in=false
```

| 실행 된 터미널 로그 | ROS 토픽 확인 |
|:-:|:-:|
| <img src="https://github.com/user-attachments/assets/869104a1-03c2-46b9-9d70-f91debec637a" width="300"/> | <img src="https://github.com/user-attachments/assets/10d64ebe-9546-4b82-9d61-c8324a41115f" width="300"/> |
> [!NOTE]
> 실행 시 `rclpy loaded` 로그가 출력된 뒤, 잠시 대기하면 다음과 같은 실행 로그가 나타난다.<br>
> `Module omni.replicator.core.ogn.python.impl.nodes.OgnAugment 843efe5 load on device 'cuda:0' took 0.81 ms  (cached)`<br><br>
> 해당 로그가 출력되면 노드가 정상적으로 실행된 상태이며,<br>
> 이후 `ros2 topic list` 명령을 통해 `/rgb_augmented` 토픽이 생성된 것을 확인할 수 있다.

2. 새로운 터미널에서 rviz2 실행
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
```bash
rviz2
```

3. Add를 눌러 By Topic 탭에서 `/rgb_augmented` 토픽의 Image를 추가합니다.
> <img src="https://github.com/user-attachments/assets/7c0879d1-f91a-49db-80df-bbaec65cc85e" width="300"/> <img src="https://github.com/user-attachments/assets/ee8a5503-fc29-43f8-9af1-a9068aedb45b" width="300"/>

## Code Explained




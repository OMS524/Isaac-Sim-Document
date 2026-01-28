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
> 실행하면 `rclpy loaded`에서 약간 기다려야 다음과 같이 실행된 로그가 나타난다.<br>
> `Module omni.replicator.core.ogn.python.impl.nodes.OgnAugment 843efe5 load on device 'cuda:0' took 0.81 ms  (cached)`<br>
> 실행된 로그를 확인한 후 `ros2 topic list`를 통해 `/rgb_augmented`를 확인할 수 있다.

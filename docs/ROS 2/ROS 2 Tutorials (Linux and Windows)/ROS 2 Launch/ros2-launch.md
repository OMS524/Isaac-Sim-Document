# ROS 2 Launch
## Environment Infomation
| Item | Description |
|-|-|
| Author | 오민석 |
| Date | 2026-02-15 |
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

## Launching Isaac Sim with ROS 2
`isaacsim` 패키지에는 Isaac Sim을 실행하기 위한 스크립트와 ROS 2 실행 파일이 포함되어 있습니다.<br>
<br>
`run_isaacsim.launch.py` 이라는 실행 파일은 `isaacsim` 패키지의 실행 폴더에 포함되어 있습니다.<br>
<br>
launch parameters는 아래에 정의되어 있습니다:
> - **version**: 사용할 Isaac Sim의 버전을 지정합니다. 지정된 버전에 대해 기본 설치 루트 폴더에서 Isaac Sim이 실행됩니다. 최신 버전의 Isaac Sim을 사용하려면 비워 둡니다.<br>
> [**default_value** = "5.1.0"]
> 
> - **install_path**: Isaac Sim이 기본값이 아닌 위치에 설치된 경우 Isaac Sim 앱 셀렉터에 표시되는 패키지 경로인 Isaac Sim 설치 루트 폴더에 대한 특정 경로를 제공합니다. (정의된 경우 "version" parameter는 무시됩니다.)<br>
> [**default_value** = "]
> 
> - **use_internal_libs**: Isaac Sim과 함께 제공된 내부 ROS 라이브러리를 사용하려면 true로 설정합니다.<br>
> [**default_value** = "true"]
> 
> - **dds_type**: 특정 dds 유형으로 Isaac Sim을 실행하려면 "fastdds" 또는 "cyclonedds"로 설정합니다.<br>
> [**default_value** = "fastdds"]
> 
> - **gui**: 표준 gui 모드에서 Isaac Sim을 시작할 때 USD 파일의 경로를 제공하여 파일을 엽니다. 비어 있으면 표준 gui 모드에서 Isaac Sim이 empty stage를 엽니다.<br>
> [**default_value** = "]
> 
> - **standalone**: Python 파일의 경로를 제공하여 파일을 열고 독립 실행형 워크플로에서 Isaac Sim을 시작합니다. 빈 상태로 두면 표준 GUI 모드에서 Isaac Sim이 empty stage를 엽니다.<br>
> [**default_value** = "]
> 
> - **play_sim_on_start**: 활성화되면 scene이 로드된 후 Isaac Sim이 play을 시작합니다. (표준 GUI 모드에서만 적용됩니다.)<br>
> [**default_value** = "false"]
> 
> - **ros_distro**: 사용할 ROS 버전을 제공합니다. Humble만 지원됩니다.<br>
> [**default_value** = "humble"]
> 
> - **ros_installation_path**: ROS 설치 경로의 쉼표로 구분된 목록입니다. ROS가 기본 설정이 아닌 위치(/opt/ros/ 아래에 없음)에 설치된 경우, ROS 설치를 위한 기본 설정.bash 파일의 경로를 제공합니다. (/path/to/custom/ros/install/setup.bash). 마찬가지로 작업 공간 설치를 위한 local_setup.bash 파일에도 경로를 추가합니다. (/path/to/custom_ros_workspace/install/local_setup.bash)<br>
> [**default_value** = ""]
> 
> - **headless**: WebRTC를 사용하여 headless 모드에서 Isaac Sim을 실행하려면 "webrtc"로 설정합니다. 비어 있으면 표준 GUI 모드에서 Isaac Sim이 실행됩니다. 이 parameter는 "standalone" parameter에 의해 재정의될 수 있습니다.<br>
> [**default_value** = ""]
> 
> - **custom_args**: 실행 시간 동안 isaac-sim.sh 에 전달하고자 하는 커스텀 Isaac Sim args를 추가합니다.<br>
> [**default_value** = ""]
> 
> - **exclude_install_path**: LD_LIBRARY_PATH, PYTHONPATH 및 PATH 환경 변수에서 제외할 설치 경로 목록입니다. <br>(/path/to/custom_ros_workspace/install/).
> [**default_value** = ""]










```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
```bash
ros2 launch isaacsim run_isaacsim.launch.py
```








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

> [!IMPORTANT]
> 해당 튜토리얼은 ROS 2에서 로컬에 설치된 Isaac Sim을 실행하는 예제입니다.<br>
> 본 환경에서는 Isaac Sim은 컨테이너에서 실행하고, ROS 2는 로컬에서 실행하고 있으므로 튜토리얼과 실행 구조가 다릅니다.<br>
> 따라서 본 튜토리얼은 Isaac Sim을 ROS 2 launch로 실행할 수 있다는 예시로 이해하시기 바랍니다.<br>

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
> [**default_value** = ""]
> 
> - **use_internal_libs**: Isaac Sim과 함께 제공된 내부 ROS 라이브러리를 사용하려면 true로 설정합니다.<br>
> [**default_value** = "true"]
> 
> - **dds_type**: 특정 dds 유형으로 Isaac Sim을 실행하려면 "fastdds" 또는 "cyclonedds"로 설정합니다.<br>
> [**default_value** = "fastdds"]
> 
> - **gui**: 표준 gui 모드에서 Isaac Sim을 시작할 때 USD 파일의 경로를 제공하여 파일을 엽니다. 비어 있으면 표준 gui 모드에서 Isaac Sim이 empty stage를 엽니다.<br>
> [**default_value** = ""]
> 
> - **standalone**: Python 파일의 경로를 제공하여 파일을 열고 독립 실행형 워크플로에서 Isaac Sim을 시작합니다. 빈 상태로 두면 표준 GUI 모드에서 Isaac Sim이 empty stage를 엽니다.<br>
> [**default_value** = ""]
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
> - **exclude_install_path**: LD_LIBRARY_PATH, PYTHONPATH 및 PATH 환경 변수에서 제외할 설치 경로 목록입니다. (/path/to/custom_ros_workspace/install/).<br>
> [**default_value** = ""]

이제 ROS 2 launch에서 Isaac Sim을 실행하는 주요 예제를 살펴보겠습니다. 다음 예제 실행 전에 이전 프로세스를 종료해야 합니다.

- 기본 설정에서 Isaac Sim을 실행하려면 아래 명령을 실행합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch isaacsim run_isaacsim.launch.py
> ```

- 작업 공간에서 사용자 지정 ROS 패키지로 Isaac Sim을 실행하려면 아래 명령을 실행합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch isaacsim run_isaacsim.launch.py exclude_install_path:=/home/user/IsaacSim-ros_workspaces/humble_ws/install ros_installation_path:=/home/user/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local_setup.bash
> ```

- 다음으로 USD 파일이 열려 있는 상태에서 아이작 심을 실행하고 즉시 플레이를 시작하겠습니다. 아래 명령을 실행하세요.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch isaacsim run_isaacsim.launch.py gui:=https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd play_sim_on_start:=true
> ```

- 이제 독립 실행형 워크플로를 사용하여 Isaac Sim을 실행해 보겠습니다. 아래 명령을 실행하세요.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch isaacsim run_isaacsim.launch.py standalone:=$HOME/isaacsim/standalone_examples/api/isaacsim.ros2.bridge/moveit.py
> ```

## Launch Isaac Sim with Nav2
Isaac Sim launch 파일은 다른 ROS 2 워크플로우에서 Isaac Sim launching을 통합하기 위해 다른 launching 파일에 포함될 수 있습니다.<br>
<br>
여기서 우리는 [Nav2 example](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#isaac-sim-app-tutorial-ros2-navigation)와 [isaac_ros_navigation_goal ROS 2 package](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html#isaac-sim-app-tutorial-ros2-nav-goals)를 사용하여 Isaac Sim을 실행하는 것을 시연할 것입니다.<br>
<br>
예제 실행 파일 `carter_navigation/launch/carter_navigation_isaacsim.launch.py`은 `carter_navigation` 패키지에서 찾을 수 있습니다.<br>
<br>
이 시나리오에서는 실행 파일이 Isaac Sim에서 console 출력을 기다리도록 구성되어 있습니다: "Stage loaded and simulation is playing." 이 메시지는 GUI 모드에서 모든 장면을 로드하는 데 사용되는 `open_isaacsim_stage.py` 스크립트에서 출력됩니다. 이는 `Isaacsim` 패키지의 스크립트 폴더에서 찾을 수 있습니다.<br>

1. 아래 명령을 사용하여 통합 실행 파일을 실행합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch carter_navigation carter_navigation_isaacsim.launch.py
> ```
> 장면이 로드될 때까지 잠시 기다립니다. 창고 내비게이션 장면이 Isaac Sim에 자동으로 로드되면 RViz2가 로봇의 센서 데이터를 자동으로 표시하기 시작하고 로봇이 탐색할 수 있는 자동 goals가 생성됩니다.

iw_hub robot navigation scene과 `iw_hub_navigation` 패키지를 사용하여 동일한 워크플로를 실행할 수 있습니다. 다음 명령을 사용하여 통합 실행 파일을 실행합니다:
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```
> ```bash
> ros2 launch iw_hub_navigation iw_hub_navigation_isaacsim.launch.py
> ```



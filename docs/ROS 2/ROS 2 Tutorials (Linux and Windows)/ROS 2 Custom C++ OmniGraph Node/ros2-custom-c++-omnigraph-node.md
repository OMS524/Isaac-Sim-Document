# ROS 2 Custom C++ OmniGraph Node
## Environment Infomation
| Item | Description |
|-|-|
| Author | 오민석 |
| Date | 2026-02-14 |
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

## Building a Custom Message Package
Isaac Sim에서 사용자 지정 메시지를 사용하려면 ROS 2를 사용하여 사용자 지정 메시지 패키지를 빌드해야 합니다. 공식 ROS 2 문서에 있는 [사용자 지정 msg 및 srv 파일을 생성](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)을 참조하여 진행합니다.
> 1. 패키지 생성
> > ```bash
> > cd ~/IsaacSim-ros_workspaces/humble_ws/src/
> > ros2 pkg create --build-type ament_cmake --license Apache-2.0 tutorial_interfaces
> > ```
> 
> 2. 디렉토리 생성
> > ```bash
> > cd ~/IsaacSim-ros_workspaces/humble_ws/src/tutorial_interfaces/
> > mkdir msg srv
> > ```
> 
> 3. 다음 파일을 생성합니다.
> > - `tutorial_interfaces/msg` 경로에 `Num.msg`라는 이름으로 파일 생성 후 아래 내용 입력
> > ```text
> > int64 num
> > ```
> > - `tutorial_interfaces/msg` 경로에 `Sphere.msg`라는 이름으로 파일 생성 후 아래 내용 입력
> > ```text
> > geometry_msgs/Point center
> > float64 radius
> > ```
> > - `tutorial_interfaces/srv` 경로에 `AddThreeInts.srv`라는 이름으로 파일 생성 후 아래 내용 입력
> > ```text
> > int64 a
> > int64 b
> > int64 c
> > ---
> > int64 sum
> > ```
> 
> 4. 빌드
> > ```bash
> > cd ~/IsaacSim-ros_workspaces/humble_ws/src/
> > colcon build --packages-select tutorial_interfaces
> > ```
> 
> 5. 소싱
> > ```bash
> > source install/setup.bash
> > ```

## Setting Up Kit Extension C++ Template
1. Omniverse Kit Extension Template C++ 설치
> ```bash
> cd ~
> git clone https://github.com/NVIDIA-Omniverse/kit-extension-template-cpp.git
> cd ~/kit-extension-template-cpp/
> git checkout release/107.3.0
> ```

2. 빌드
> ```bash
> ./build.sh
> ```

3. 테스트
> ```bash
> cd ~/kit-extension-template-cpp/
> ./_build/linux-x86_64/release/omni.app.kit.dev.sh
> ```

4. 이 튜토리얼에 사용될 샘플 사용자 지정 확장 프로그램을 다운로드하세요.
> [Custom ROS 2 OmniGraph Node Extension (Humble)](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/_downloads/5418eff6891a41d71ac8b5f687bf1cbd/omni.example.cpp.omnigraph_node_ros.zip)

5. 다운받은 zip파일을 `kit-extension-template-cpp/source/extensions` 경로에 압축 해체합니다.

6. `kit-extension-template-cpp/deps` 경로에 `kit-sdk-deps.packman.xml` 파일의 맨 끝에 `</project>` 위에 다음 내용을 추가합니다.
> ```text
> <dependency name="system_ros" linkPath="../_build/target-deps/system_ros" tags="${config}">
>     <source path="/opt/ros/humble" />
> </dependency>
> 
> <dependency name="additional_ros_workspace" linkPath="../_build/target-deps/additional_ros" tags="${config}">
>     <source path="/home/user/IsaacSim-ros_workspaces/humble_ws/install/tutorial_interfaces" />
> </dependency>
> ```
> **source path**에서 `user`의 이름은 개인 컴퓨터 환경에 맞춰서 넣어주세요

7. 빌드
> ```bash
> ./build.sh
> ```

## Adding the Extension to Isaac Sim
1. 기존 컨테이너를 삭제하고 `kit-extension-template-cpp`을 마운트하여 다시 생성합니다.
> ```bash
> docker stop isaac-sim
> ```
> ```bash
> docker rm isaac-sim
> ```
> ```bash
> docker run --name isaac-sim \
>   --entrypoint bash -it --gpus all \
>   --network=host \
>   -u 1234:1234 \
>   -e ACCEPT_EULA=Y \
>   -e PRIVACY_CONSENT=Y \
>   -e ROS_DISTRO=humble \
>   -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
>   -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib \
>   -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
>   -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
>   -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
>   -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
>   -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
>   -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
>   -v ~/IsaacSim-ros_workspaces:/IsaacSim-ros_workspaces:rw \
>   -v ~/kit-extension-template-cpp:/kit-extension-template-cpp:rw \
>   nvcr.io/nvidia/isaac-sim:5.1.0
> ```

2. 컨테이너에서 다음 명령을 실행합니다.
> ```bash
> export PATH=/isaac-sim/kit/python/bin:$PATH
> ```
> ```bash
> source /IsaacSim-ros_workspaces/humble_ws/install/local_setup.bash
> ```

3. Isaac Sim 실행합니다.
> ```bash
> ./runheadless.sh
> ```

4. Isaac Sim에서 **Window > Extensions**로 이동하여 `Custom ROS2 OGN Example Extension`을 활성화합니다.
> - **Setting** 클릭
> > <img width="1000" alt="image" src="https://github.com/user-attachments/assets/48409f49-b783-45ce-a047-062c8f49992b" />
> - **Extension Search Paths**에 `kit-extension-template-cpp/_build/linux-x86_64/release/exts` 경로 추가
> > <img width="1000" alt="image" src="https://github.com/user-attachments/assets/ef0375c6-7608-4c98-bd25-6ae5c789c07c" />
> - **Refresh** 클릭
> > <img width="1000" alt="image" src="https://github.com/user-attachments/assets/ec7aba3c-d9cf-4074-adb8-794ef079d580" />
> - 검색창에 `Custom ROS2 OGN Example Extension` 입력 후 **THIRD PARTY** 클릭하여 `Custom ROS2 OGN Example Extension` extension 활성화
> > <img width="1000" alt="image" src="https://github.com/user-attachments/assets/52ce0adb-b572-4714-9aa3-42184a932dae" />

5. **Window > Graph Editors > Action Graph**로 이동하여 Action Graph를 생성 후 다음과 같이 구성합니다.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/f246f6bd-fa61-4c9b-96d5-7a06d5f20d76" /><br>
>
> **Custom Python ROS 2 Node** 노드는 커스텀 노드입니다.<br>
> **To String** 노드는 사용자 지정 노드의 출력을 문자열로 변환합니다.<br>
> **Print Text** 노드는 인쇄하여 뷰포트 또는 터미널에 대한 사용자 지정 노드의 출력을 문자열로 표시합니다. 속성 패널에서 `To Screen`을 체크하면 뷰포트에 텍스트가 출력됩니다.<br>
> `To Screen`을 체크합니다.




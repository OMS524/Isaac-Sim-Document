# URDF Import: Turtlebot
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
1. ROS2 설치, ROS2 사용 가능, ROS2 확장 기능 활성화, 필요 환경 변수 설정
2. ROS 워크스페이스 대한 이해
3. xacro 설치
```bash
sudo apt install ros-humble-xacro
```

## TurtleBot URDF 가져오기
### turtlebot3 깃 클론
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/src
git clone -b $ROS_DISTRO https://github.com/ROBOTIS-GIT/turtlebot3.git turtlebot3
```

### Turtlebot3 Burger의 URDF 파일을 전처리하여 네임스페이스 인자 값을 수동으로 제거하고 tb3_burger_processed.urdf에 저장
```bash
cd turtlebot3/turtlebot3_description/urdf
namespace=""
xacro ./turtlebot3_burger.urdf "namespace:=${namespace:+$namespace/}" > tb3_burger_processed.urdf
```

### URDF 파일 접근 권한 부여
URDF 파일을 불러올려면 그 URDF 파일의 접근 권한이 있어야 한다.
```bash
docker run --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
    -e "PRIVACY_CONSENT=Y" \
    -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
    -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
    -v ~/IsaacSim-ros_workspaces:/IsaacSim-ros_workspaces:rw \
    -u 1234:1234 \
    nvcr.io/nvidia/isaac-sim:5.1.0
```
위 Docker 실행 명령어에서 -u 1234:1234는 user의 uid, gid를 의미한다.
해당 uid, gid를 이용하여 다음 명령어를 통해 소유권을 가지게 할 수 있다.
sudo chown -R <container_uid>:<container_gid> ~/IsaacSim-ros_workspaces
```bash
sudo chown -R 1234:1234 ~/IsaacSim-ros_workspaces/humble_ws
```

또한, 위 Docker 실행 명령어를 보면
-v ~/IsaacSim-ros_workspaces:/IsaacSim-ros_workspaces:rw를 추가하여
Isaac Sim에서 URDF 파일을 가져올 수 있게 워크스페이스를 마운트 해주었고, rw는 읽기/쓰기 권한을 의미한다.

### Isaac Sim 실행
```bash
docker run --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
    -e "PRIVACY_CONSENT=Y" \
    -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
    -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
    -v ~/IsaacSim-ros_workspaces:/IsaacSim-ros_workspaces:rw \
    -u 1234:1234 \
    nvcr.io/nvidia/isaac-sim:5.1.0
./runheadless.sh -v
```

### simple_room.usd 불러오기
Content 브라우저에서 Isaac Sim-Environments-Simple_Room에 simple_room.usd를 Stage 브라우저로 드래그
<img width="1850" height="1053" alt="image" src="https://github.com/user-attachments/assets/a0b383e1-dce4-4b23-8598-855e800a45d4" />

### URDF 불러오기
File-Import 에서 워크스페이스에서 turtlebot3/turtlebot3_description/urdf/tb3_burger_processed.urdf를 선택
우측 프롬프트 창에서 Option 부분 설정 몇 가지를 진행
  1. Model 섹션에서 Referenced Model 설정
  2. Links 섹션에서 Moveable Base 설정
  3. Joints & Drives 섹션에서 wheel_left_joint, wheel_right_joint의 Target을 Velocity로 설정
위 설정을 진행한 후 Import
<img width="920" height="555" alt="image" src="https://github.com/user-attachments/assets/0e74590b-9597-4389-bf97-70cd189b624d" />

Import를 하면 다음과 같은 팝업이 나타난다.
해당 팝업 내용은 URDF를 USD로 변환해서 해당 경로에 저장하겠다는 내용이다.
Yes를 클릭한다.

<img width="464" height="105" alt="image" src="https://github.com/user-attachments/assets/c06cfda0-82e5-48e3-b5f7-fd1dc5a4a277" />

다음과 같이 URDF를 불러온 것을 확인할 수 있다.
<img width="1850" height="1053" alt="image" src="https://github.com/user-attachments/assets/fd9674c9-26af-4b9b-9159-1b62474e60fe" />






















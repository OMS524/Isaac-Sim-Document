# Commands
## Isaac Sim
**Terminal 1**
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
```bash
./runheadless.sh -v
```
**Terminal 2**
```bash
~/docker/isaacsim-webrtc-streaming-client-1.1.5-linux-x64.AppImage
```

## ROS 2
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

<br>
<br>
<br>
<br>
<br>

## Test
컨테이너 생성
```bash
docker run --name isaac-sim \
  --entrypoint bash -it --gpus all \
  --network=host \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT=Y \
  -e ROS_DISTRO=humble \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib \
  -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
  -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
  -v ~/IsaacSim-ros_workspaces:/IsaacSim-ros_workspaces:rw \
  -u $(id -u):$(id -g) \
  nvcr.io/nvidia/isaac-sim:5.1.0
```

`-u $(id -u):$(id -g)`로 컨테이너의 id를 `host`의 id와 맞춰서 실행 시 컨테이너는 `host`의 폴더 권한을 얻지만 컨테이너 안에 `root(GID:0)`가 권한을 가지고 있는 `/isaac-sim`의 권한이 없어 접근을 못하기 때문에 권한을 부여 해줘야 한다.<br>
(컨테이너 생성 시 최초 1회 실행)
```bash
docker exec -it -u root isaac-sim bash
chmod a+rx /isaac-sim
```


```bash
./python.sh /IsaacSim-ros_workspaces/tutorial/src/publishing_camera_data.py \
  --/app/omni.graph.scriptnode/opt_in=true \
  --/app/omni.graph.scriptnode/enable_opt_in=false    
```
```bash
sudo chown -R $(id -u):$(id -g) ~/IsaacSim-ros_workspaces
```

### Docker
컨테이너 확인
```bash
docker ps -a
```

컨테이너 삭제
```bash
docker rm isaac-sim
```

컨테이너 실행
```bash
docker start isaac-sim
```

컨테이너 종료
```bash
docker stop isaac-sim
```

실행 중인 컨테이너 들어가기
```bash
docker exec -it isaac-sim bash
```



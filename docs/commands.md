# Commands
## Docker Commands
### Container 생성
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

### Container 확인 및 삭제
Container 확인
```bash
docker ps -a
```
Container 삭제
```bash
docker rm isaac-sim
```

### Container 실행 및 종료
Container 실행
```bash
docker start isaac-sim
```
Container 종료
```bash
docker stop isaac-sim
```

### Container 진입
실행 중인 Container 진입
```bash
docker exec -it isaac-sim bash
```

### 실행 명령어
시각화 가능한 Isaac Sim 실행
```bash
./runheadless.sh -v
```
Python 파일 실행
```bash
./python.sh /IsaacSim-ros_workspaces/tutorial/src/publishing_camera_data.py \
  --/app/omni.graph.scriptnode/opt_in=true \
  --/app/omni.graph.scriptnode/enable_opt_in=false    
```

## ROS 2 Commands
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

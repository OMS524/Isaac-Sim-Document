# Commands
## Isaac Sim
### Terminal 1
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
```bash
~/docker/isaacsim-webrtc-streaming-client-1.1.5-linux-x64.AppImage
```

## ROS 2
### Terminal 2
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

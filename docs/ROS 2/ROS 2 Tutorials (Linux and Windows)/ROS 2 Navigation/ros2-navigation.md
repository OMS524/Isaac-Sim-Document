# ROS 2 Navigation
## Environment Infomation
| Item | Description |
|-|-|
| Author | 오민석 |
| Date | 2026-02-09 |
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

## Nav2 Setup
이 시나리오에서 Nav2에 publish되는 topic와 message 유형은 다음과 같습니다:
| ROS2 Topic | ROS2 Message Type |
|-|-|
| /tf | tf2_msgs/TFMessage |
| /odom | nav_msgs/Odometry |
| /map | nav_msgs/OccupancyGrid |
| /point_cloud | sensor_msgs/PointCloud |
| /scan | sensor_msgs/LaserScan (published by an external pointcloud_to_laserscan node) |

### Occupancy Map
이 시나리오에서는 occupancy map가 필요합니다. 이 샘플의 경우 NVIDIA Isaac Sim 내의 [Occupancy Map Generator extension](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/digital_twin/ext_isaacsim_asset_generator_occupancy_map.html#ext-isaacsim-asset-generator-occupancy-map)을 사용하여 창고 환경의 occupancy map를 생성하고 있습니다.<br>
<br>

1. **Window > Examples > Robotics Examples**로 이동하세요.<br>**Robotics Examples** 탭을 클릭하세요.<br>
**ROS2 > Navigation > Nova Carter**를 누르고 우측에서 **Load Sample Scene**을 클릭하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/5dea56d2-de9e-4599-896a-14e4a8dc1f91" /><br>
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/04346da6-8b05-46e8-8a4d-cd43380244c8" /><br>

> [!NOTE]
> Scene을 불러올 때 스트리밍 클라이언트가 멈출 시 스트리밍 클라이언트를 재실행하세요.

2. viewport에서 왼쪽 상단에 **Camera**를 클릭합니다.<br>**Top**을 클릭합니다.
> <img width="200" alt="image" src="https://github.com/user-attachments/assets/ad8ebc7e-1bac-48bd-b9bc-53411b0e5770" /><br>
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/66eb5a43-4b6a-44eb-958a-2e1f57ebfa4b" /><br>

3. **Tools > Robotics > Occupancy Map**을 클릭합니다.
> <img width="400" alt="image" src="https://github.com/user-attachments/assets/2516cb17-c758-4687-a17e-0cec9dadd219" /><br>

4. Occupancy Map extension에서 `Origin`이 `X: 0.0, Y: 0.0, Z: 0.0`으로 설정합니다.<br>`Lower bound`의 경우 `Z: 0.1`으로 설정합니다.<br>`Upper Bound`의 경우 ``Z: 0.62`으로 설정합니다.<br>upper bound Z distance는 ground에 대한 Lidar onboard Nova Carter의 vertical distance와 일치하도록 0.62미터로 설정되어 있다는 점을 명심하세요.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/9b285411-fe13-4c55-99de-c5acda8815e7" /><br>

5. Stage에서 **warehouse_with_forklifts**을 클릭합니다.<br>Occupancy Map extension에서 **BOUND SELECTION**을 클릭합니다.
> <img width="1000" alt="image" src="https://github.com/user-attachments/assets/8b578b0e-9092-4836-891c-56eff09efed4" /><br>

































# RTX Lidar Sensors
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

## Adding a RTX Lidar ROS 2 Bridge
1. **Create > Sensors > RTX Lidar > NVIDIA > Example Rotary 2D**



2. 생성된 Example Rotary 2D의 Prim을 */World/turtlebot3_burger/base_scan*으로 드래그하세요.<br>그 후 Example Rotary 2D의 Property 탭에서 위치를 `(0,0,0)`으로 설정합니다.



3. **Create > Sensors > RTX Lidar > NVIDIA > Example Rotary**


4. 생성된 Example Rotary의 Prim을 */World/turtlebot3_burger/base_scan*으로 드래그하세요.<br>그 후 Example Rotary의 Property 탭에서 위치를 `(0,0,0)`으로 설정합니다.

5. **Window > Graph Editors > Action Graph**에서 New Action Graph를 클릭합니다.<br>생성된 Action Graph를 *//World/turtlebot3_burger/base_scan*로 이동합니다.<br>다음과 같이 Action Graph를 구성합니다.


> <img width="500" alt="image" src="https://github.com/user-attachments/assets/1f1e749d-4cfa-4baf-a74b-2f9268d6faf7" />















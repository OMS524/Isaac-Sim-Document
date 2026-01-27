# ROS 2 Cameras
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

## Setting Up Cameras
| Step | Screenshot | Description |
|------|-----------|--------------------|
| 1 | <img src="https://github.com/user-attachments/assets/e011d9bf-08f6-4d9a-a3f9-fe02df035b30" width="300"/> | **Window > Viewports > Viewports 2** Viewports 2 활성화 |
| 2 | <img src="https://github.com/user-attachments/assets/c2711f5f-b786-4d2d-ae8d-02b3b22bd77b" width="300"/> | **Create > Camera** Camera 2대 생성 |
| 3 | <img src="https://github.com/user-attachments/assets/2b9cdcfe-f5f5-4364-9df2-5e8454eb2428" width="300"/> | Stage 창에서 생성된 카메라를 `Camera_1`, `Camera_2`로 변경 |
| 4 | <img src="https://github.com/user-attachments/assets/76616624-d713-45ed-8f4c-08de10f0871f" width="300"/> | `Camera_1`의 Translate를 `x=-3.5,y=-2.5,z=0.05` Orient를 `x=90.0,y=-50.0,z=0.0` `Camera_2`의 Translate를 `x=3.5,y=-2.5,z=0.05` Orient를 `x=90.0,y=50.0,z=0.0` |





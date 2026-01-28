# Isaac Sim Document
이 문서는 NVIDIA Isaac Sim 설치부터 기본 실행, ROS2 연동, 튜토리얼까지 실습 중심으로 정리된 한글 가이드입니다.

원본 공식 Isaac Sim Documentation을 바탕으로 누구나 따라 설치하고 실행할 수 있도록 구성되었습니다.

https://docs.isaacsim.omniverse.nvidia.com/5.1.0/index.html

## 개요
**NVIDIA Isaac Sim**은 로봇 개발과 시뮬레이션을 위한 GPU 기반 물리 엔진 플랫폼입니다. 로봇 모델 로드, 센서 시뮬레이션, ROS2 연동, 강화학습 환경까지 지원합니다.

## 시스템 요구사항
| Element | Minimum Spec | Good | Ideal |
|-|-|-|-|
| OS | Ubuntu 22.04/24.04, Windows 10/11 | Ubuntu 22.04/24.04, Windows 10/11 | Ubuntu 22.04/24.04, Windows 10/11 |
| CPU | Intel Core i7 (7th Generation), AMD Ryzen 5 | Intel Core i7 (9th Generation), AMD Ryzen 7 | Intel Core i9, X-series or higher, AMD Ryzen 9, Threadripper or higher |
| Cores | 4 | 8 | 16 |
| RAM | 32GB | 64GB | 64GB |
| Storage | 50GB SSD | 500GB SSD | 1TB NVMe SSD |
| GPU | GeForce RTX 4080 | GeForce RTX 5080 | RTX PRO 6000 Blackwell |
| VRAM | 16GB | 16GB | 48GB |
| Driver | Linux: 580.65.06, Windows: 580.88 | Linux: 580.65.06, Windows: 580.88 | Linux: 580.65.06, Windows: 580.88 |

## Tutorials
튜토리얼의 순서는 다음과 같습니다.
### Getting Started with Importing and Controlling
  - [URDF Import: Turtlebot](/docs/ROS%202/ROS%202%20Tutorials%20%28Linux%20and%20Windows%29/urdf-import-turtlebot.md)
  - [Driving TurtleBot using ROS 2 Messages](/docs/ROS%202/ROS%202%20Tutorials%20%28Linux%20and%20Windows%29/driving-tutlebot-using-ros-2-messages.md)
### Timing
  - [ROS 2 Clock](/docs/ROS%202/ROS%202%20Tutorials%20%28Linux%20and%20Windows%29/ros-2-clock.md)
  - [ROS 2 Publish Real Time Factor (RTF)](/docs/ROS%202/ROS%202%20Tutorials%20%28Linux%20and%20Windows%29/ros-2-publish-real-time-factor.md)

### Sensors and Control
  - [ROS 2 Cameras](/docs/ROS%202/ROS%202%20Tutorials%20%28Linux%20and%20Windows%29/ROS%202%20Cameras/ros-2-cameras.md)



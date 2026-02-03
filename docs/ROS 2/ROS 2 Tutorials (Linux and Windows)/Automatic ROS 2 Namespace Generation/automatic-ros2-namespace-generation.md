# Automatic ROS 2 Namespace Generation
## Environment Infomation
| Item | Description |
|-|-|
| Author | 오민석 |
| Date | 2026-02-03 |
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

## ROS 2 Namespaces
ROS 2에서 namespaces를 관리하는 것은 다중 로봇 시뮬레이션에서 각 로봇의 topic를 고유하게 식별할 수 있도록 하는 데 매우 중요합니다.<br>
현재 OmniGraph 내에서 ROS publisher, subscriber 및 서비스를 위한 namespaces를 설정하는 방법은 크게 두 가지가 있습니다.<br>
1. nodeNamespace 필드의 namespaces 필드를 수동으로 설정합니다.
> <img width="736" height="218" alt="image" src="https://github.com/user-attachments/assets/cbc90cda-6e98-4743-a719-b5e21675275b" /><br>
2. (Recommended) 모든 Isaac Sim ROS OmniGraph 노드의 namespaces를 자동으로 생성하도록 asset을 구성합니다.<br>이 튜토리얼은 Isaac Sim에서 namespaces를 설정하는 과정을 안내하여 효율적인 topic 관리와 충돌 방지를 가능하게 합니다.

### Configuring the Asset

#### asdasdsa




























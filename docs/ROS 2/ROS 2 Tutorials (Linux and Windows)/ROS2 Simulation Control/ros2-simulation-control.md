# ROS2 Simulation Control
## Environment Infomation
| Item | Description |
|-|-|
| Author | 오민석 |
| Date | 2026-02-15 |
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

## Getting Started
- Simulation Interfaces package를 설치합니다
> ```bash
> sudo apt install ros-humble-simulation-interfaces
> ```

## Overview
ROS 2 imulation Control extension은 ROS 2 Simulation Interfaces를 사용하여 Isaac Sim 기능을 제어합니다. 이 extension은 extension 가능하도록 설계되어 있어 성능 오버헤드 없이 여러 서비스와 작업을 동시에 실행할 수 있습니다.<br>
<br>
이 extension 기능은 다음을 포함하여 ROS 2 services and actions을 통해 Isaac Sim에 대한 포괄적인 제어를 제공합니다:
- **Simulation State Control**: Play, pause, stop, and step through simulations
- **Entity Management**: Spawn, delete, and manipulate simulation entities (prims)
- **World Management**: Load, unload, and query simulation worlds (USD files)
- **State Querying**: entities, simulation state 및 사용 가능한 resources에 대한 정보 가져오기
<br>
이를 통해 Simulation Interfaces 패키지를 사용하여 Isaac Sim을 프로그래밍 방식으로 또는 ROS 2 command line interface를 통해 제어하여 자동화된 테스트와 같은 워크플로우를 지원할 수 있습니다. 이 페이지에서는 Isaac Sim에서 지원하는 Simulation Interfaces의 모든 services and actions에 대한 예제 명령어를 나열하고 제공합니다.

### Enabling the Extension
1. Isaac Sim 실행합니다.
2. **Window > Extensions**으로 이동하여 `isaacsim.ros2.sim_control`을 활성화합니다.
> <img width="500" alt="image" src="https://github.com/user-attachments/assets/a735143a-ea2e-458f-b44a-4d05c44ff5b7" />

## Available Services and Actions
Extension 프로그램은 다음과 같은 ROS 2 services를 제공합니다:
- `/get_simulator_features`: Isaac Sim implementation에서 지원되는 기능을 나열합니다
- `/set_simulation_state`: 시뮬레이션을 특정 상태로 설정합니다(stopped/playing/paused/quitting)
- `/get_simulation_state`: 현재 시뮬레이션 상태 가져오기
- `/get_entities`: 시뮬레이션에서 모든 entities(prims) 목록 가져오기
- `/get_entity_info`: 특정 entities에 대한 자세한 정보를 가져옵니다(현재 OBJECT category type 반환)
- `/get_entity_state`: 특정 entities의 pose, twist, acceleration를 가져옵니다
- `/get_entitys_states`: 필터링을 통해 여러 entities의 상태(pose, twist, acceleration)를 가져옵니다
- `/delete_entity`: 시뮬레이션에서 특정 entities(prim)를 삭제합니다
- `/spawn_entity`: 지정된 위치에서 시뮬레이션에 새 entities 생성
- `/reset_simulation`: 시뮬레이션 환경을 초기 상태로 재설정합니다
- `/set_entity_state`: 시뮬레이션에서 특정 entities의 상태(pose, twist)를 설정합니다
- `/step_simulation`: 시뮬레이션을 특정 프레임 수로 진행합니다
- `/load_world`: 시뮬레이션에 world 또는 환경 파일 로드
- `/unload_world`: 현재 world를 언로드하고 empty stage를 만듭니다
- `/get_current_world`: 현재 로드된 world에 대한 정보를 가져옵니다
그리고 다음 ROS 2 동작들:
- `/simulate_steps`: progress feedback을 사용하여 시뮬레이션 stepping 하는 action

## Using the ROS 2 Simulation Control Services
이 섹션에서는 사용 가능한 각 service를 사용하는 방법에 대해 자세히 설명합니다.<br>
이후 나오는 예제 명령어들은 다음 명령을 실행한 ROS 2 소싱된 터미널에서 실행해야 합니다.
> ```bash
> cd ~/IsaacSim-ros_workspaces/humble_ws/
> export FASTRTPS_DEFAULT_PROFILES_FILE=/home/oms/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
> source /opt/ros/humble/setup.bash
> source install/local_setup.bash
> ```

### GetSimulatorFeatures Service
GetSimulatorFeatures Service는 Simulation_interfaces에서 Isaac Sim이 지원하는 services와 actions의 하위 집합을 나열합니다.
> ```bash
> ros2 service call /get_simulator_features simulation_interfaces/srv/GetSimulatorFeatures
> ```
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/d505c797-0c65-4de8-8ba5-14463a59b9eb" />

### SetSimulationState Service
SetSimulationState Service는 SimulationState.msg(STATE_STOPED, STATE_PLAYING, STATE_PAUSED, STATE_QUITING)에 정의된 에넘에 해당하는 시뮬레이션의 글로벌 상태(stopped/playing/paused/quitting료)를 업데이트합니다.
> - To set simulation state to playing:
> ```bash
> ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{state: {state: 1}}"
> ```
> - To set simulation state to paused:
> ```bash
> ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{state: {state: 2}}"
> ```
> - To set simulation state to stopped:
> ```bash
> ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{state: {state: 0}}"
> ```
> - To quit the simulator:
> ```bash
> ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{state: {state: 3}}"
> ```
> [ros2_simulation_control_1.webm](https://github.com/user-attachments/assets/230211df-4847-4439-8aa0-3e3dde734d78)

### GetSimulationState Service
GetSimulationState Service는 SimulationState.msg(STATE_STOPED, STATE_PLAYING, STATE_PASUED, STATE_QUITING)에 정의된 에넘에 해당하는 전체 시뮬레이션의 현재 상태(stopped/playing/paused/quitting)를 검색합니다.
> ```bash
> ros2 service call /get_simulation_state simulation_interfaces/srv/GetSimulationState
> ```
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/94b610dd-fb38-49ad-85c2-a98e41088989" />

Returns state 0 for stopped, 1 for playing, 2 for paused

### GetEntities Service
GetEntities Service는 선택적 필터링(재귀 패턴 사용)을 통해 시뮬레이션에 존재하는 모든 엔티티의 목록을 가져옵니다.
> - Get all entities in the simulation:
> ```bash
> ros2 service call /get_entities simulation_interfaces/srv/GetEntities "{filters: {filter: ''}}"
> ```
> - Get entities with full paths or partial paths. In this case filter for prims containing `camera` in the path:
> ```bash
> ros2 service call /get_entities simulation_interfaces/srv/GetEntities "{filters: {filter: 'camera'}}"
> ```
> - Get entities with paths starting with `/World`:
> ```bash
> ros2 service call /get_entities simulation_interfaces/srv/GetEntities "{filters: {filter: '^/World'}}"
> ```
> - Get entities with paths ending with `mesh`:
> ```bash
> ros2 service call /get_entities simulation_interfaces/srv/GetEntities "{filters: {filter: 'mesh$'}}"
> ```
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/ffc74901-18d7-42a7-b55c-358bbea7ef52" />

### GetEntityInfo Service
GetEntityInfo Service는 특정 엔티티의 유형 및 속성과 같은 세부 정보를 제공합니다.
> ```bash
> ros2 service call /get_entity_info simulation_interfaces/srv/GetEntityInfo "{entity: '/World/robot'}"
> ```
> [ros2_simulation_control_11.webm](https://github.com/user-attachments/assets/eb0e01d8-cce4-40db-8856-529656191c0f)

### GetEntityState Service
GetEntityState Service는 주어진 기준 프레임에 대해 특정 엔티티의 pose, twist, acceleration를 제공합니다. 현재는 world 프레임만 지원됩니다.
> ```bash
> ros2 service call /get_entity_state simulation_interfaces/srv/GetEntityState "{entity: '/World/robot'}"
> ```
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/05b6a0f8-8398-436b-824e-f2291ceddbae" />

- RigidBodyAPI를 사용하는 엔티티의 경우 자세와 속도가 모두 반환됩니다
- RigidBodyAPI가 없는 엔티티의 경우, 속도가 0인 포즈만 반환됩니다
- Acceleration 값은 항상 0으로 보고됩니다(현재 API에서 제공되지 않음)
- 엔티티 상태를 성공적으로 검색하면 `RESULT_OK`를 반환합니다
- 엔티티가 존재하지 않는 경우 `RESULT_NOT_FOUND`를 반환합니다
- 엔티티 상태를 검색하는 동안 오류가 발생하면 `RESULT_OPERATION_FAILED`를 반환합니다

### GetEntitiesStates Service
GetEntitiesStates Service는 시뮬레이션에서 여러 엔티티의 월드 프레임에서 상태(pose, twist, acceleration)를 가져옵니다.
> - Get states for all entities in the simulation:
> ```bash
> ros2 service call /get_entities_states simulation_interfaces/srv/GetEntitiesStates "{filters: {filter: ''}}"
> ```
> - Get states for entities containing ‘robot’ in their path:
> ```bash
> ros2 service call /get_entities_states simulation_interfaces/srv/GetEntitiesStates "{filters: {filter: 'robot'}}"
> ```
> - Get states for entities with paths starting with ‘/World’:
> ```bash
> ros2 service call /get_entities_states simulation_interfaces/srv/GetEntitiesStates "{filters: {filter: '^/World'}}"
> ```
> [ros2_simulation_control_12.webm](https://github.com/user-attachments/assets/e10f5882-a937-4281-b1c7-5d9c945b74a5)

- GetEntities와 GetEntityState 서비스의 기능을 결합합니다
- 리제그스 패턴 매칭을 사용하여 엔티티를 먼저 필터링합니다
- 필터링된 각 엔티티의 상태를 검색합니다
- 엔티티 경로 및 해당 상태 목록을 반환합니다
- RigidBodyAPI를 사용하는 엔티티의 경우 pose와 velocities가 모두 반환됩니다
- RigidBodyAPI가 없는 엔티티의 경우, velocities가 0인 포즈만 반환됩니다
- Acceleration 값은 항상 0으로 보고됩니다(현재 API에서 제공되지 않음)
- 이 service를 사용하는 것이 여러 엔티티에 상태가 필요할 때 여러 GetEntityState 호출을 하는 것보다 더 효율적입니다
- 엔티티 상태를 성공적으로 검색하면 `RESULT_OK`를 반환합니다
- 필터링 또는 검색 상태에서 오류가 발생하면 `RESULT_OPERATION_FAILED`를 반환합니다

### DeleteEntity Service
DeleteEntity Service는 시뮬레이션에서 지정된 엔티티를 삭제합니다.
> ```bash
> ros2 service call /delete_entity simulation_interfaces/srv/DeleteEntity "{entity: '/World/robot'}"
> ```
> <img width="300" alt="image" src="https://github.com/user-attachments/assets/9a8eb892-7a81-444f-a931-987b75410ef0" />

- 엔티티가 성공적으로 삭제되면 서비스는 `RESULT_OK`를 반환합니다
- 엔티티가 보호되어 삭제할 수 없는 경우 `RESULT_OPERATION_FAILED`를 반환합니다
- 삭제를 시도하기 전에 prim_utils.is_prim_no_delete()를 사용하여 prim을 삭제할 수 있는지 확인합니다

### SpawnEntity Service
SponEntity Service는 지정된 위치에서 시뮬레이션에 새로운 엔티티를 생성합니다.
> - Basic entity spawn with default position:
> ```bash
> ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'MyEntity', allow_renaming: false, uri: '/path/to/model.usd'}"
> ```
> - Spawn with specific position and orientation:
> ```bash
> ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'PositionedEntity', allow_renaming: false, uri: '/path/to/model.usd', initial_pose: {pose: {position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}}"
> ```
> - Empty Xform creation (no URI):
> ```bash
> ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'EmptyXform', allow_renaming: false, uri: ''}"
> ```
> - With auto-renaming enabled:
> ```bash
> ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'AutoRenamedEntity', allow_renaming: true, uri: '/path/to/model.usd'}"
> ```
> - With namespace specified:
> ```bash
> ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'NamespacedEntity', allow_renaming: false, uri: '/path/to/model.usd', entity_namespace: 'robot1'}"
> ```
> [ros2_simulation_control_2.webm](https://github.com/user-attachments/assets/2f308005-0375-43ec-a8b1-dc3feaf6b0e3)

- URI가 제공되면, 주어진 기본 경로에서 USD 파일을 참조로 로드합니다
- URI가 제공되지 않으면 주어진 기본 경로에 Xform을 생성합니다
- 생성된 모든 프림은 추적을 위해 simulationInterfacesSpawned 속성으로 표시됩니다
- 엔티티가 성공적으로 생성되었으면 `RESULT_OK`를 반환합니다
- 엔티티 이름이 이미 존재하고 allow_renaming이 거짓인 경우 `NAME_NOT_UNIQUE (101)`을 반환합니다
- 엔티티 이름이 비어 있고 allow_renaming이 거짓인 경우 `NAME_INVALID (102)`를 반환합니다
- USD 파일을 parse 또는 load하지 못한 경우 `RESOURCE_PARSE_ERROR (106)`을 반환합니다

### ResetSimulation Service
ResetSimulation Service는 시뮬레이션 환경을 초기 상태로 재설정합니다.
> ```bash
> ros2 service call /reset_simulation simulation_interfaces/srv/ResetSimulation
> ```
> [ros2_simulation_control_3.webm](https://github.com/user-attachments/assets/49daf4a8-f0ac-4f53-b284-c2828e26513f)

- 시뮬레이션 timeline을 중지합니다
- simulationInterfacesSpawned 속성을 사용하여 모든 프림을 찾아서 제거합니다
- 다중 패스를 사용하여 생성된 모든 엔티티가 제거되도록 보장합니다
- 시뮬레이션 timeline을 재시작합니다
- 재설정에 성공하면 `RESULT_OK`를 반환합니다
- 오류 재설정 시뮬레이션에서 `RESULT_OPERATION_FAILED`를 반환합니다

### SetEntityState Service
SetEntityState Service는 시뮬레이션에서 특정 엔티티의 상태(pose, twist)를 설정합니다. 현재 world 프레임에서는 변환만 허용됩니다.
> - Set only position and orientation:
> ```bash
> ros2 service call /set_entity_state simulation_interfaces/srv/SetEntityState "{
>   entity: '/World/Cube',
>   state: {
>     header: {frame_id: 'world'},
>     pose: {
>       position: {x: 1.0, y: 2.0, z: 3.0},
>       orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
>     },
>     twist: {
>       linear: {x: 0.0, y: 0.0, z: 0.0},
>       angular: {x: 0.0, y: 0.0, z: 0.0}
>     }
>   }
> }"
> ```
> - Set position, orientation and velocity (for entities with rigid body physics):
> ```bash
> ros2 service call /set_entity_state simulation_interfaces/srv/SetEntityState "{
>   entity: '/World/RigidBody',
>   state: {
>     header: {frame_id: 'world'},
>     pose: {
>       position: {x: 1.0, y: 2.0, z: 3.0},
>       orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
>     },
>     twist: {
>       linear: {x: 0.5, y: 0.0, z: 0.0},
>       angular: {x: 0.0, y: 0.0, z: 0.1}
>     }
>   }
> }"
> ```
> [ros2_simulation_control_4.webm](https://github.com/user-attachments/assets/b3d71260-43d7-4db4-b1f3-804944a74149)

- position와 orientation은 항상 모든 엔티티에 대해 업데이트됩니다
- Velocities는 RigidBodyAPI를 가진 엔티티에만 설정됩니다
- non-rigid bodies의 경우 position와 orientation만 설정됩니다(velocity 설정은 무시됩니다)
- Acceleration 설정은 현재 지원되지 않으며 무시됩니다
- 엔티티 상태를 성공적으로 설정하면 `RESULT_OK`를 반환합니다
- 엔티티가 존재하지 않는 경우 `RESULT_NOT_FOUND`를 반환합니다
- 오류 설정 엔티티 상태인 경우 `RESULT_OPERATION_FAILED`를 반환합니다

### StepSimulation Service
StepSimulation Service는 유한한 수의 단계를 시뮬레이션하고 일시 정지 상태로 돌아갑니다.
> - Step the simulation by 1 frame (note: will use 2 steps internally):
> ```bash
> ros2 service call /step_simulation simulation_interfaces/srv/StepSimulation "{steps: 1}"
> ```
> - Step the simulation by 10 frames:
> ```bash
> ros2 service call /step_simulation simulation_interfaces/srv/StepSimulation "{steps: 10}"
> ```
> - Step the simulation by 100 frames:
> ```bash
> ros2 service call /step_simulation simulation_interfaces/srv/StepSimulation "{steps: 100}"
> ```
> [ros2_simulation_control_5.webm](https://github.com/user-attachments/assets/b7b0af89-2b53-48ed-98c3-3a59731c2ee7)

- stepping을 수행하려면 시뮬레이션이 일시 중지된 상태여야 합니다
- 모든 step가 완료될 때까지 service 호출이 차단됩니다
- step이 완료되면 시뮬레이션은 자동으로 일시 중지 상태로 돌아갑니다
- step이 성공적으로 완료되면 `RESULT_OK`를 반환합니다
- service 호출 시 시뮬레이션이 일시 중지되지 않으면 `RESULT_INCORRECT_STATE`를 반환합니다
- 스텝 중 오류가 발생하면 `RESULT_OPERATION_FAILED`를 반환합니다

### LoadWorld Service
LoadWorld Service는 시뮬레이션에 world 또는 environment 파일을 로드하여 현재 scene을 지우고 시뮬레이션을 중지 상태로 설정합니다. 현재 USD 형식 worlds를 지원합니다.
> - Load a world from a USD file:
> ```bash
> ros2 service call /load_world simulation_interfaces/srv/LoadWorld "{uri: '/path/to/world.usd'}"
> ```
> - Load a sample world with Isaac Sim sample environments:
> ```bash
> ros2 service call /load_world simulation_interfaces/srv/LoadWorld "{uri: 'https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Environments/Simple_Room/simple_room.usd'}"
> ```
> - Load a sample world with a ROS2 scenario:
> ```bash
> ros2 service call /load_world simulation_interfaces/srv/LoadWorld "{uri: 'https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Samples/ROS2/Scenario/carter_warehouse_apriltags_worker.usd'}"
> ```
> [ros2_simulation_control_6.webm](https://github.com/user-attachments/assets/3d27ece4-e2b4-4df6-bbe0-90ef00d08243)

- USD 파일(.usda, .usda, .usdc, .usdz)만 지원됩니다
- world를 로드하기 전에 시뮬레이션을 중지하거나 일시 중지해야 합니다(재생되지 않음)
- world를 로드하면 현재 scene이 지워지고 USD 파일에서 새로운 stage가 생성됩니다
- 주어진 path를 직접 찾을 수 없는 경우, Isaac Sim은 자동으로 default assetroot path로 접두사를 붙이려고 합니다
- 월드가 성공적으로 로드되었으면 `RESULT_OK`를 반환합니다
- 파일 형식이 지원되지 않는 경우 `UNSUPPORTED_FORMAT`을 반환합니다
- USD 파일을 parse 또는 load할 수 없는 경우 `RESOURCE_PARSE_ERROR`를 반환합니다
- service 호출 시 시뮬레이션이 재생 중일 때 `RESULT_OPERATION_FAILED`를 반환합니다

### UnloadWorld Service
UnloadWorld Service는 현재 world를 시뮬레이션에서 unload하여 현재 scene을 지우고 새로운 empty stage를 생성합니다. 이전에 생성된 엔티티는 모두 제거됩니다.
> ```bash
> ros2 service call /unload_world simulation_interfaces/srv/UnloadWorld
> ```
> [ros2_simulation_control_7.webm](https://github.com/user-attachments/assets/1ae3fc7e-4044-42ae-8c7a-45595a493744)

- 시뮬레이션은 월드를 unload하기 전에 중지하거나 일시 중지해야 합니다(재생되지 않음)
- 현재 세계를 unload한 후 새로운 empty stage를 생성합니다
- world가 성공적으로 unload되었으면 `RESULT_OK`를 반환합니다
- 현재 load된 world가 없는 경우 `NO_WORLD_LOADED`를 반환합니다
- service 호출 시 시뮬레이션이 재생 중일 때 `RESULT_OPERATION_FAILED`를 반환합니다

### GetCurrentWorld Service
GetCurrentWorld Service는 현재 로드된 world에 대한 URI, name, format 등의 정보를 반환합니다.
> ```bash
> ros2 service call /get_current_world simulation_interfaces/srv/GetCurrentWorld
> ```
> [ros2_simulation_control_8.webm](https://github.com/user-attachments/assets/dea0ecd5-7b0a-4edf-b13d-86ff57e55a6b)

- 파일에서 world가 로드된 경우 URI와 name을 포함한 world 정보를 반환합니다
- 메모리에서 생성된 world(new stage)의 경우, 빈 URI와 "untitled_world"를 이름으로 반환합니다
- 성공하면 월드 정보와 함께 `RESULT_OK`를 반환합니다
- 현재 로드된 월드가 없는 경우 `NO_WORLD_LOADED`를 반환합니다

### GetAvailableWorlds Service
GetAvailableWorlds Service는 시뮬레이션에 로드할 수 있는 사용 가능한 world 파일 목록을 반환합니다. TagsFilter 기반 필터링을 지원하여 기본 Isaac Sim 경로에서 USD 월드 파일을 검색합니다.
> - Get all default available worlds:
> ```bash
> ros2 service call /get_available_worlds simulation_interfaces/srv/GetAvailableWorlds
> ```
> - Get worlds with tag filtering (search for default worlds with specific tags in filename):
> ```bash
> ros2 service call /get_available_worlds simulation_interfaces/srv/GetAvailableWorlds "{filter: {tags: ['warehouse', 'carter']}, continue_on_error: true}"
> ```
> - Search additional custom paths:
> ```bash
> ros2 service call /get_available_worlds simulation_interfaces/srv/GetAvailableWorlds "{additional_sources: ['/custom/worlds/path'], continue_on_error: true}"
> ```
> - Offline-only search with additional local sources:
> ```bash
> ros2 service call /get_available_worlds simulation_interfaces/srv/GetAvailableWorlds "{additional_sources: ['/home/user/custom_worlds', '/opt/isaac_worlds'], offline_only: true, continue_on_error: true}"
> ```
> [ros2_simulation_control_9.webm](https://github.com/user-attachments/assets/071fc0f2-66fd-4e46-b649-13556f29a7c8)

- 기본 Isaac Sim 경로 검색: `/Isaac/Environments` 및 `/Isaac/Samples/ROS2/Scenario`
- 태그 매칭을 위해 FILTER_MODE_ANY(기본값) 또는 FILTER_MODE_ALL로 TagsFilter를 지원합니다
- `additional_sources`에 지정된 추가 사용자 지정 경로를 검색할 수 있습니다
- `offline_only: true`: 로컬 파일 시스템 경로만 검색하는 경우 true
- 일부 경로가 실패하더라도 계속 검색하려면 `continue_on_error: true`를 설정합니다
- 사용 가능한 월드 목록과 함께 `RESULT_OK`를 반환합니다
- 기본 자산 경로에 액세스할 수 없고 추가 소스가 제공되지 않는 경우 `DEFAULT_SOURCES_FAILED`을 반환합니다

## Using the ROS 2 Simulation Control Actions
### SimulateSteps Action
SimulateSteps Action은 유한한 수의 steps를 시뮬레이션하고 각 step이 끝난 후 피드백과 함께 일시 정지 상태로 돌아갑니다.
> - Basic usage - Step the simulation by 10 frames:
> ```bash
> ros2 action send_goal /simulate_steps simulation_interfaces/action/SimulateSteps "{steps: 10}"
> ```
> - With feedback - Step the simulation by 20 frames and show feedback:
> ```bash
> ros2 action send_goal /simulate_steps simulation_interfaces/action/SimulateSteps "{steps: 20}" --feedback
> ```
> [ros2_simulation_control_10.webm](https://github.com/user-attachments/assets/11ea9cfd-08cf-4651-ba00-6c856416fda1)

- stepping을 수행하려면 시뮬레이션이 일시 중지된 상태여야 합니다
- step이 완료되면 시뮬레이션이 일시 중지된 상태로 돌아갑니다
- 각 step이 완료되고 남은 step을 표시한 후 피드백을 받게 됩니다
- 실행 중에 action을 취소할 수 있습니다

## Technical Details
extension 기능은 `omni.timeline` interface를 사용하여 시뮬레이션 상태를 제어하고 표준 서비스를 통해 깨끗한 ROS 2 interface를 제공합니다. 구현에는 다음이 포함됩니다:
- 단일 노드를 통해 모든 ROS 2 services를 처리하는 singleton `ROS2ServiceManager`
- Isaac Sim’s timeline과 interface하는 `SimulationControl` class
- Action Graph interface와 독립적으로 ROS 2 spinning을 위한 Thread-safe implementation




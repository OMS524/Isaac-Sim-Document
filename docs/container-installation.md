# Container Installation
## Environment Infomation
| Item | Description |
|-|-|
| Author | 오민석 |
| Date | 2026-01-24 |
| OS | Ubuntu 22.04 |
| GPU | NVIDIA RTX 6000 Ada Generation |
| Driver Version | 580.126.09 |
| CUDA Version | 13.0 |

## 필수 조건
Isaac Sim Installation 과정에서 IOMMU 비활성화 부분이 완료되어 있어야 합니다.

## 컨테이너 설치
### 1. Docker 설치
```bash
sudo apt update
sudo apt install -y curl 
```
Docker installation using the convenience script
```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```
Post-install steps for Docker
```bash
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```
Verify Docker
```bash
docker run hello-world
```

### 2. NVIDIA Container Toolkit을 설치
Configure the repository
```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
    && \
    sudo apt-get update
```
Install the NVIDIA Container Toolkit packages
```bash
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```
Configure the container runtime
```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```
Verify NVIDIA Container Toolkit
```bash
docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
```


## 컨테이너 배포
### 1. GPU 드라이버 버전 확인
```bash
nvidia-smi
```

### 2. Pull the Isaac Sim Container
```bash
docker pull nvcr.io/nvidia/isaac-sim:5.1.0
```

### 3. host에 캐시된 볼륨 마운트를 생성
```bash
mkdir -p ~/docker/isaac-sim/cache/main/ov
mkdir -p ~/docker/isaac-sim/cache/main/warp
mkdir -p ~/docker/isaac-sim/cache/computecache
mkdir -p ~/docker/isaac-sim/config
mkdir -p ~/docker/isaac-sim/data/documents
mkdir -p ~/docker/isaac-sim/data/Kit
mkdir -p ~/docker/isaac-sim/logs
mkdir -p ~/docker/isaac-sim/pkg
sudo chown -R 1234:1234 ~/docker/isaac-sim
```

### 4. 대화형 Bash 세션을 사용하여 Isaac Sim Container를 실행
```bash
docker run --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
    -e "PRIVACY_CONSENT=Y" \
    -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
    -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
    -u 1234:1234 \
    nvcr.io/nvidia/isaac-sim:5.1.0
```

### 5. 사용 중인 시스템이 Isaac Sim과 호환되는지 확인
```bash
./isaac-sim.compatibility_check.sh --/app/quitAfter=10 --no-window
```

### 6. 라이브 스트리밍 모드로 Isaac Sim을 시작
```bash
./runheadless.sh -v
```

### 7. Isaac Sim WebRTC Streaming Client 설치 및 실행
https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html#isaac-sim-latest-release
<img width="781" height="607" alt="image" src="https://github.com/user-attachments/assets/a17c6764-ce54-46ae-99b7-e250f6683268" />
| Name | Version | Release Date | Links |
|------|---------|--------------|-------|
| Isaac Sim WebRTC Streaming Client | 1.1.5 | October 2025 | [Linux (x86_64)](https://download.isaacsim.omniverse.nvidia.com/isaacsim-webrtc-streaming-client-1.1.5-linux-x64.AppImage) |

다운 받은 isaacsim-webrtc-streaming-client-1.1.5-linux-x64.AppImage 파일을
~/docker 경로로 이동 후

libfuse2 설치
```bash
sudo apt update
sudo apt install libfuse2
```

권한 부여 및 실행
```bash
cd ~/docker
chmod +x isaacsim-webrtc-streaming-client-1.1.5-linux-x64.AppImage
./isaacsim-webrtc-streaming-client-1.1.5-linux-x64.AppImage
```

실행하면 다음과 같은 화면이 나온다.
Isaac Sim의 로컬 인스턴스에 연결하려면 Server 입력칸에 기본 IP 주소인 127.0.0.1을 입력 후
Connect를 누르면 다음과 같이 Isaac Sim 인터페이스가 나타난다.

(Isaac Sim 실행 터미널에서 `Isaac Sim Full Streaming App is loaded.`라는 문구가 나온 후 Connect를 진행할 것)
| Connect Interface | Streaming Interface |
|-|-|
| <img src="https://github.com/user-attachments/assets/23f0d4bc-bcbf-465e-b2d9-8ff43d907958" width="300"/> | <img src="https://github.com/user-attachments/assets/26919689-4653-4ac6-ab3f-ffe6d12a3afe" width="300"/> |

## 실행 명령어
```bash
docker run --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
    -e "PRIVACY_CONSENT=Y" \
    -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
    -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
    -u 1234:1234 \
    nvcr.io/nvidia/isaac-sim:5.1.0
./runheadless.sh -v
```
```bash
~/docker/isaacsim-webrtc-streaming-client-1.1.5-linux-x64.AppImage
```





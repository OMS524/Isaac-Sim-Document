# Container Installation
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

### 7. Isaac Sim WebRTC Streaming Client 설치













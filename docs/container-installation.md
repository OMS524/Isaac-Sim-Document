# Container Installation
## 컨테이너 설치
### 1. Docker 설치
```bash
$ sudo apt update
$ sudo apt install -y curl 

# Docker installation using the convenience script
$ curl -fsSL https://get.docker.com -o get-docker.sh
$ sudo sh get-docker.sh

# Post-install steps for Docker
$ sudo groupadd docker
$ sudo usermod -aG docker $USER
$ newgrp docker

# Verify Docker
$ docker run hello-world
```
### 2. NVIDIA Container Toolkit을 설치
```bash
# Configure the repository
$ curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
    && \
    sudo apt-get update

# Install the NVIDIA Container Toolkit packages
$ sudo apt-get install -y nvidia-container-toolkit
$ sudo systemctl restart docker

# Configure the container runtime
$ sudo nvidia-ctk runtime configure --runtime=docker
$ sudo systemctl restart docker

# Verify NVIDIA Container Toolkit
$ docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
```









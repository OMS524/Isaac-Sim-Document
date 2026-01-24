# Isaac-Sim-Document
Isaac Sim에서의 설치 및 튜토리얼 내용

## 설치
### 1. Isaac Sim zip 파일 다운로드
https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html
<img width="781" height="607" alt="image" src="https://github.com/user-attachments/assets/a17c6764-ce54-46ae-99b7-e250f6683268" />

| Name | Version | Release Date | Links |
|------|---------|--------------|-------|
| Isaac Sim | 5.1.0 | October 2025 | [Linux (x86_64)](https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip) |



### 2. 디렉토리 생성 및 압축 풀기
디렉토리 생성
```bash
mkdir -p ~/isaac-sim
unzip ~/Downloads/isaac-sim-standalone-5.1.0-linux-x86_64.zip -d ~/isaac-sim
```
압축 풀기
```bash
./post_install.sh
./isaac-sim.selector.sh
```
위 명령어를 실행하면 아래와 같은 오류 팝업이 나온다.

<img width="1487" height="213" alt="image" src="https://github.com/user-attachments/assets/3b8b92ed-c6ad-4311-9fb2-5542f2e1cb79" />
<img width="502" height="527" alt="image" src="https://github.com/user-attachments/assets/e802b28f-bcd7-46b5-88a5-bb0739004a82" />

IOMMU가 켜져 있으면, Linux에서 CUDA랑 NVIDIA 드라이버가 GPU 간 메모리 복사를 제대로 못 해서 오류, 깨짐, 크래스가 날 수 있다
라는 오류 팝업이 뜨고 Isaac Sim App Selector가 실행된다.
따라서, BIOS에서 IOMMU를 비활성화를 해야 한다.

메인보드가 SUPERMICRO 같은 경우 설정 방법은 다음과 같다.
| Step | Screenshot | Path / Description |
|------|-----------|--------------------|
| 1 | <img src="https://github.com/user-attachments/assets/ccf9846d-4826-45a8-a159-2c752d53f3e8" width="300"/> | BIOS 진입 후 **Advanced** 탭 선택 |
| 2 | <img src="https://github.com/user-attachments/assets/b9878d0e-f853-4af7-bb61-2eee4f96945a" width="300"/> | **Chipset Configuration** 진입 |
| 3 | <img src="https://github.com/user-attachments/assets/bbbe24b5-dab3-41f1-8a59-d9d0bc659b43" width="300"/> | **North Bridge** 선택 |
| 4 | <img src="https://github.com/user-attachments/assets/257466e0-e4d2-4ceb-99f1-a3a3ef1231e0" width="300"/> | **IIO Configuration** 진입 |
| 5 | <img src="https://github.com/user-attachments/assets/fe7a84c1-be5f-4fbf-9edf-edf948dd5d79" width="300"/> | **Intel VT for Directed I/O (VT-d)** → `Disabled` |









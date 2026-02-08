# Isaac Sim Workstation Installation
## Environment Infomation
| Item | Description |
|-|-|
| Author | 오민석 |
| Date | 2026-01-24 |
| OS | Ubuntu 22.04 |
| GPU | NVIDIA RTX 6000 Ada Generation |
| Driver Version | 580.126.09 |
| CUDA Version | 13.0 |

## 1. Isaac Sim zip 파일 다운로드
https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html
<img width="781" height="607" alt="image" src="https://github.com/user-attachments/assets/a17c6764-ce54-46ae-99b7-e250f6683268" />

| Name | Version | Release Date | Links |
|------|---------|--------------|-------|
| Isaac Sim | 5.1.0 | October 2025 | [Linux (x86_64)](https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip) |



## 2. 디렉토리 생성 및 압축 풀기
디렉토리 생성
```bash
mkdir -p ~/isaac-sim
```
압축 풀기
```bash
unzip ~/Downloads/isaac-sim-standalone-5.1.0-linux-x86_64.zip -d ~/isaac-sim
```



## 3. 실행
```bash
cd ~/isaac-sim
./post_install.sh
./isaac-sim.selector.sh
```
위 명령어를 실행하면
IOMMU가 켜져 있으면, Linux에서 CUDA랑 NVIDIA 드라이버가 GPU 간 메모리 복사를 제대로 못 해서 오류, 깨짐, 크래스가 날 수 있다
라는 내용의 오류 팝업이 나온다.

<img width="1487" height="213" alt="image" src="https://github.com/user-attachments/assets/3b8b92ed-c6ad-4311-9fb2-5542f2e1cb79" />

따라서, BIOS에서 IOMMU를 비활성화를 해야 한다.

메인보드가 SUPERMICRO 같은 경우 설정 방법은 다음과 같다.

### IOMMU Disable
> [!NOTE]
> IOMMU 비활성화 방법은 메인보드 제품에 따라 본 내용에 나오는 비활성화 방법과 다를 수 있습니다.

**Intel VT for Directed I/O (VT-d) Disabled**
| Step | Screenshot | Path / Description |
|:-:|:-:|-|
| 1 | <img src="https://github.com/user-attachments/assets/ccf9846d-4826-45a8-a159-2c752d53f3e8" width="300"/> | BIOS 진입 후 **Advanced** 탭 선택 후<br>**Chipset Configuration** 진입 |
| 2 | <img src="https://github.com/user-attachments/assets/b9878d0e-f853-4af7-bb61-2eee4f96945a" width="300"/> | **North Bridge** 진입 |
| 3 | <img src="https://github.com/user-attachments/assets/bbbe24b5-dab3-41f1-8a59-d9d0bc659b43" width="300"/> | **IIO Configuration** 진입 |
| 4 | <img src="https://github.com/user-attachments/assets/257466e0-e4d2-4ceb-99f1-a3a3ef1231e0" width="300"/> | **Intel VT for Directed I/O (VT-d)** 진입 |
| 5 | <img src="https://github.com/user-attachments/assets/fe7a84c1-be5f-4fbf-9edf-edf948dd5d79" width="300"/> | **Intel VT for Directed I/O (VT-d)** → `Disabled` |

**Intel VT for Directed I/O (VT-d)** 우측 설명을 보면
**To disable VT-d, X2APIC must also be disabled.** 라고 나와있다.<br>
즉, VT-d를 Disable 할려면 **X2APIC**도 Disabled 해줘야 한다.<br>
**X2APIC**를 Disable 하지 않고 **Intel VT for Directed I/O (VT-d)** 만 Disabled 하게 되면
저장하고 나와도 **Intel VT for Directed I/O (VT-d)** 는 다시 Enabled 가 된다.<br>

### **X2APIC** Disabled
| Step | Screenshot | Path / Description |
|:-:|:-:|-|
| 1 | <img src="https://github.com/user-attachments/assets/ccf9846d-4826-45a8-a159-2c752d53f3e8" width="300"/> | BIOS 진입 후 **Advanced** 탭 선택 후<br>**CPU Configuration** 진입 |
| 2 | <img src="https://github.com/user-attachments/assets/0a77c151-59ee-4772-8f5a-d1a3f0c107b7" width="300"/> | **Extended APIC** → `Disabled` |

IOMMU 비활성화 확인 명령어
```bash
sudo dmesg | grep -e DMAR -e IOMMU
ls /sys/kernel/iommu_groups/
```
위 명령어를 실행하면 다음 사진과 같이 아무런 출력이 없으면 비활성화 된 것을 확인할 수 있다.
<img width="786" height="173" alt="image" src="https://github.com/user-attachments/assets/72e10ef9-71ba-4600-9800-0b1093c18c49" />

Isaac Sim을 실행하면 아래와 같이 **Isaac Sim App Selector** 창이 나타난다.
여기서 **Isaac Sim**을 선택한 후 **START** 버튼을 누르면,
두 번째 이미지와 같이 기본 빈 Stage가 열린 Isaac Sim 메인 화면이 실행된다.

| Isaac Sim App Selector | Isaac Sim Main Window (New Stage) |
|-|-|
| <img src="https://github.com/user-attachments/assets/e802b28f-bcd7-46b5-88a5-bb0739004a82" width="300"/> | <img src="https://github.com/user-attachments/assets/a0d83112-e15a-475e-90f8-4cd6c661a495" width="300"/> |

## 실행 명령어
```bash
~/isaac-sim/isaac-sim.selector.sh
```





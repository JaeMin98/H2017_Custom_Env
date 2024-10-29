

# 🤖 H2017 로봇팔 강화학습 환경 설정

본 가이드는 H2017 로봇팔 제어를 위한 강화학습 환경을 단계별로 설정하는 방법을 제공합니다. 이 환경은 [두산 로보틱스 GitHub](https://github.com/doosan-robotics/doosan-robot)에서 제공하는 URDF를 기반으로 하며, end-effector와 base의 collision 및 visual 모델은 자체 제작된 모델입니다. 기존 DSR 에뮬레이터(dsr_emulator)의 Docker 실행 불편함과 배속 조절 불가 문제를 개선한 환경을 제공합니다.

# 💻 운영체제 설치

운영체제 설치는 [이 가이드](https://blog.naver.com/jm_0820/223001100698)를 참고하여 진행하십시오.

# 🛠️ ROS 설치

ROS Noetic 설치는 [ROS Noetic 설치 가이드](http://wiki.ros.org/noetic/Installation/Ubuntu)를 참고하여 진행합니다.

# 🦾 MoveIt 설치

MoveIt 관련 패키지를 설치하려면 다음 명령어를 사용하세요:

```bash
sudo apt install ros-noetic-moveit -y
sudo apt-get install ros-noetic-joint-trajectory-controller -y
sudo apt-get install ros-noetic-effort-controllers -y
sudo apt-get install ros-noetic-rosbridge-server -y
```

# 📁 ROS 작업공간 설정

ROS 작업공간 설정 단계는 아래와 같습니다:

1. ROS 환경 불러오기:

    ```bash
    source /opt/ros/noetic/setup.sh
    ```

2. 작업공간 생성 및 초기화:

    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    ```

3. 컴파일 및 환경 설정:

    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

# 📁 ROS 패키지 생성

ROS 패키지 생성을 위해 아래 명령어를 사용합니다:

```bash
cd ~/catkin_ws/src
catkin_create_pkg my_package
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

# ⚙️ 선택 옵션

## 📅 시스템 업데이트

```bash
sudo apt-get update
sudo apt-get upgrade
```

## ⌨️ 한국어 키보드 설정

[한국어 키보드 설정 가이드](https://shanepark.tistory.com/231)를 참고하십시오.

## 🐍 pip 설치

```bash
sudo apt-get install python3-pip -y
```

## 💻 추가 프로그램 설치

- [GitHub Desktop](https://gist.github.com/berkorbay/6feda478a00b0432d13f1fc0a50467f1)
- [TeamViewer](https://www.teamviewer.com/ko/download/linux/)
- [VSCode](https://code.visualstudio.com/download)

```bash
sudo apt install barrier -y  # KVM 스위치 소프트웨어
sudo apt-get install terminator -y  # 편리한 터미널
```

# 🎨 그래픽 드라이버 및 CUDA 설치

## 🚮 기존 그래픽 드라이버 제거

```bash
sudo apt --purge remove *nvidia*
sudo apt-get autoremove
sudo apt-get autoclean
sudo rm -rf /usr/local/cuda*
```

## 1️⃣ 그래픽 드라이버 설치

1. 드라이버 확인 및 설치:

    ```bash
    ubuntu-drivers devices
    sudo apt-get install nvidia-driver-<버전번호>
    sudo apt-get install dkms nvidia-modprobe -y
    sudo apt-get update
    sudo apt-get upgrade
    sudo reboot now
    ```

2. 설치 확인:

    ```bash
    nvidia-smi
    ```

## 2️⃣ CUDA 설치 (권장 버전: 11.8 또는 12.1)

CUDA 설치를 위해 [NVIDIA CUDA Toolkit 아카이브](https://developer.nvidia.com/cuda-toolkit-archive)를 참고하여 runfile을 다운로드하고 실행 권한을 부여한 후 설치를 진행합니다.

```bash
chmod 777 <runfile>
nvcc -V  # 설치 확인
```

## 3️⃣ cuDNN 설치

1. [cuDNN 아카이브](https://developer.nvidia.com/rdp/cudnn-archive)에서 deb 파일을 다운로드 후 설치합니다.
2. 필요한 경우 아래 명령어로 CUDA 및 cuDNN 소스 목록을 삭제합니다.

    ```bash
    sudo rm /etc/apt/sources.list.d/cuda*
    sudo rm /etc/apt/sources.list.d/cudnn*
    ```

# 🔥 PyTorch 설치 및 CUDA 확인

다음 Python 코드를 실행하여 CUDA와 cuDNN 설정을 확인하세요:

```python
import torch
print(torch.cuda.is_available())
if torch.cuda.is_available():
    print(torch.cuda.current_device())
    print(torch.cuda.get_device_name(torch.cuda.current_device()))
print(torch.backends.cudnn.enabled)
print(torch.backends.cudnn.version())
```

# 🦾 H2017 ROS 패키지 생성

## 1️⃣ Link attacher 설치 (Pick&Place 용도)

```bash
cd ~/catkin_ws/src
git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git -b melodic-devel
cd ..
catkin_make
source devel/setup.bash
sudo apt install ros-noetic-gazebo-plugins
```

## 2️⃣ Custom URDF 다운로드

H2017 URDF 패키지를 설치하려면 아래 명령어를 사용합니다:

```bash
cd ~/catkin_ws/src
git clone https://github.com/JaeMin98/h2017_URDF
cd ..
catkin_make
source devel/setup.bash
```

## 3️⃣ Moveit setup assistant
```bash
roslaunch moveit_setup_assistant setup_assistant.launch
```
[영상 가이드](https://www.youtube.com/watch?v=gC_CYeNccQk)

## 4️⃣ Custom world 설정
```bash
sudo mv ~/catkin_ws/src/h2017_URDF/worlds/objects/*.dae /usr/share/gazebo-11/models/
ls /usr/share/gazebo-11/models
```
world_name의 default 값을 변경 >> "$(find h2017_URDF)/worlds/custom_world.world"
```bash
gedit ~/catkin_ws/src/h2017_w_gripper1/launch/demo_gazebo.launch
gedit ~/catkin_ws/src/h2017_w_gripper1/launch/gazebo.launch
```

## 5️⃣ Launch
```
roslaunch h2017_w_gripper1 demo_gazebo.launch
```


### 🔴 controller error 발생 시 
<plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so"/> 중복 제거
```
gedit ~/catkin_ws/src/h2017_w_gripper1/config/gazebo_h2017.urdf

```

### 🔴 Link attacher 비활성화를 원할 시
world 파일에서 <plugin name="link_attacher_plugin" filename="libgazebo_ros_link_attacher.so" /> 제거
urdf 파일에서 <plugin name="ros_link_attacher_plugin" filename="libgazebo_ros_link_attacher.so"/> 제거


# 🤖 H2017 robotic arm custom environment setup
![title_image](https://github.com/user-attachments/assets/9e745252-7a92-4869-88d4-d8001fb0bb89)
This guide provides a step-by-step setup for the reinforcement learning environment to control the H2017 robotic arm. This environment is based on the URDF provided by [Doosan Robotics GitHub](https://github.com/doosan-robotics/doosan-robot), and it uses a custom-made model for collision and visual models of the end-effector and base. It offers an improved environment that addresses the inconvenience of running Docker in the existing DSR emulator (dsr_emulator) and the inability to control the playback speed.

# 💻 Operating System Installation

Please refer to [this guide](https://blog.naver.com/jm_0820/223001100698) to install the operating system.

# 🛠️ ROS Installation

Install ROS Noetic by following the instructions in the [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu).

# 🦾 MoveIt Installation

To install MoveIt-related packages, use the following commands:

```bash
sudo apt install ros-noetic-moveit -y
sudo apt-get install ros-noetic-joint-trajectory-controller -y
sudo apt-get install ros-noetic-effort-controllers -y
sudo apt-get install ros-noetic-rosbridge-server -y
```

# 📁 Setting up the ROS Workspace

Follow these steps to set up the ROS workspace:

1. Load the ROS environment:

    ```bash
    source /opt/ros/noetic/setup.sh
    ```

2. Create and initialize the workspace:

    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    ```

3. Compile and configure the environment:

    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

# 📁 Creating a ROS Package

To create a ROS package, use the following commands:

```bash
cd ~/catkin_ws/src
catkin_create_pkg my_package
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

# ⚙️ Optional Settings

## 📅 System Update

```bash
sudo apt-get update
sudo apt-get upgrade
```

## ⌨️ Setting up Korean Keyboard

Refer to [this guide](https://shanepark.tistory.com/231) for setting up the Korean keyboard.

## 🐍 Installing pip

```bash
sudo apt-get install python3-pip -y
```

## 💻 Installing Additional Programs

- [GitHub Desktop](https://gist.github.com/berkorbay/6feda478a00b0432d13f1fc0a50467f1)
- [TeamViewer](https://www.teamviewer.com/ko/download/linux/)
- [VSCode](https://code.visualstudio.com/download)

```bash
sudo apt install barrier -y  # KVM switch software
sudo apt-get install terminator -y  # Convenient terminal
```

# 🎨 Graphics Driver and CUDA Installation

## 🚮 Removing Existing Graphics Drivers

```bash
sudo apt --purge remove *nvidia*
sudo apt-get autoremove
sudo apt-get autoclean
sudo rm -rf /usr/local/cuda*
```

## 1️⃣ Installing Graphics Driver

1. Verify and install the driver:

    ```bash
    ubuntu-drivers devices
    sudo apt-get install nvidia-driver-<version number>
    sudo apt-get install dkms nvidia-modprobe -y
    sudo apt-get update
    sudo apt-get upgrade
    sudo reboot now
    ```

2. Verify the installation:

    ```bash
    nvidia-smi
    ```

## 2️⃣ Installing CUDA (Recommended Versions: 11.8 or 12.1)

Refer to the [NVIDIA CUDA Toolkit Archive](https://developer.nvidia.com/cuda-toolkit-archive) to download the runfile, grant execute permissions, and install CUDA.

```bash
chmod 777 <runfile>
nvcc -V  # Verify installation
```

## 3️⃣ Installing cuDNN

1. Download the deb file from the [cuDNN Archive](https://developer.nvidia.com/rdp/cudnn-archive) and install it.
2. If needed, use the following commands to remove CUDA and cuDNN source lists.

    ```bash
    sudo rm /etc/apt/sources.list.d/cuda*
    sudo rm /etc/apt/sources.list.d/cudnn*
    ```

# 🔥 Verifying PyTorch and CUDA Installation

Run the following Python code to verify CUDA and cuDNN settings:

```python
import torch
print(torch.cuda.is_available())
if torch.cuda.is_available():
    print(torch.cuda.current_device())
    print(torch.cuda.get_device_name(torch.cuda.current_device()))
print(torch.backends.cudnn.enabled)
print(torch.backends.cudnn.version())
```

# 🦾 Creating the H2017 ROS Package

## 1️⃣ Installing the Link Attacher (For Pick&Place)

```bash
cd ~/catkin_ws/src
git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git -b melodic-devel
cd ..
catkin_make
source devel/setup.bash
sudo apt install ros-noetic-gazebo-plugins
```

## 2️⃣ Downloading the Custom URDF

To install the H2017 URDF package, use the following commands:

```bash
cd ~/catkin_ws/src
git clone https://github.com/JaeMin98/h2017_URDF
cd ..
catkin_make
source devel/setup.bash
```

## 3️⃣ MoveIt Setup Assistant

```bash
roslaunch moveit_setup_assistant setup_assistant.launch
```
[Video Guide](https://www.youtube.com/watch?v=gC_CYeNccQk)

## 4️⃣ Setting up a Custom World

```bash
sudo mv ~/catkin_ws/src/h2017_URDF/worlds/objects/*.dae /usr/share/gazebo-11/models/
ls /usr/share/gazebo-11/models
```
```bash
gedit ~/catkin_ws/src/h2017_w_gripper1/launch/demo_gazebo.launch # Change the default value of world_name >> "$(find h2017_URDF)/worlds/custom_world.world"
gedit ~/catkin_ws/src/h2017_w_gripper1/launch/gazebo.launch # Change the default value of world_name >> "$(find h2017_URDF)/worlds/custom_world.world"
```

## 5️⃣ Launch

```bash
roslaunch h2017_w_gripper1 demo_gazebo.launch
```

### 🔴 If a Controller Error Occurs

```bash
gedit ~/catkin_ws/src/h2017_w_gripper1/config/gazebo_h2017.urdf  # Remove any duplicate gazebo_ros_control plugins
```

### 🔴 To Disable the Link Attacher

```bash
Remove <plugin name="link_attacher_plugin" filename="libgazebo_ros_link_attacher.so" /> from the world file
Remove <plugin name="ros_link_attacher_plugin" filename="libgazebo_ros_link_attacher.so"/> from the urdf file
```

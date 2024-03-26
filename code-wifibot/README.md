# WifiBot + Joyustick demo
This package holds the two required packages to run a demo with a WiFiBot and a
logitech F710 joystick.

# Requirements:
* ROS noetic (base is enough)
* OpenCV
* Dependencies:
```bash
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    curl \
    vim \
    git \
    cmake \
    make \
    gcc \
    gdb \
    g++ \
    libopencv-dev \
    python3-opencv \
    python3-pip \
    python3-ipdb \
    ipython3

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt-get install ros-noetic-ros-base -y --no-install-recommends

sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    ros-noetic-joy \
    ros-noetic-tf \
    ros-noetic-camera-info-manager \
    ros-noetic-image-transport \
    ros-noetic-image-transport-plugins \
    v4l-utils
```

Create a directory in HOME called catkin_ws, and run this commands:
```
cd
mkdir catkin_ws
cd catkin_ws
git clone https://Joako1991@bitbucket.org/Joako1991/code-wifibot.git src
source /opt/ros/noetic/setup.bash
rospack profile
catkin_make
```

In order to enable the Raspberry pi camera, execute these two lines:
```bash
echo "start_x=1" | sudo tee -a /boot/firmware/config.txt
echo "gpu_mem=128" | sudo tee -a /boot/firmware/config.txt
```

Then, to run the demo on startup, run this command:

```bash
crontab -e
```

If you want to use the USB camera, copy the following line at the end:

```bash
@reboot /home/ubuntu/catkin_ws/src/launch_demo.sh
```

In the previous line, you need to change ubuntu by your username.
Finally, in the launch_demo.sh, change the IP addresses by the RPI4 IP addresses,
so you can connect remotely to the robot. You should put both, CLIENT and SERVER,
the same address, the one for the RPI4.

In the local computer, execute the following lines:
```bash
ROS_SERVER_IP="xxx.xxx.xxx.xxx"
ROS_CLIENT_IP="yyy.yyy.yyy.yyy"
CUSTOM_ROS_URI="http://${ROS_SERVER_IP}:11311"
export ROS_MASTER_URI="${CUSTOM_ROS_URI}"
export ROS_HOSTNAME=${ROS_CLIENT_IP}
export ROS_IP=${ROS_CLIENT_IP}
```

Change xxx.xxx.xxx.xxx by the RPI4 IP address, and the yyy.yyy.yyy.yyy by
the computer address


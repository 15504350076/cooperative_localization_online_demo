ROS2 on Ubuntu22
================
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-ros-base
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc


Build and test
==============
cmake -B build
cmake --build build
./build/SysMsg/SysMsgTest
./build/SysMsg/SysMsgThreadsTest


Update ROS1 codes
=================
Override the related files in Common/SysMsg with  SysMsg/Include/*, SysMsg/Src/* and SysMsg/Test/*
Update the Common/SysMsg/CMakeLists.txt according to the example in SysMsg/CMakeLists.txt here

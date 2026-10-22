On Ubuntu
=========
sudo apt install libsdbus-c++-dev


Build and test
==============
cmake -B build
cmake --build build
./build/SysMsg/SysMsgTest
./build/SysMsg/SysMsgThreadsTest


Update ROS codes
================
cp SysMsg/Include/SysMsg.h <drone_dir>/Common/SysMsg/Include/
cp SysMsg/Src/SysMsg.cpp <drone_dir>/Common/SysMsg/Src/
cp SysMsg/Test/*.cpp <drone_dir>/Common/SysMsg/Test/
Update the Common/SysMsg/CMakeLists.txt according to the example in SysMsg/CMakeLists.txt here

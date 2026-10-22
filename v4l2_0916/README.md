# 概要
本模块是采集mipi摄像头并编码成一个个jpeg图片，并将采集时间戳做为jpeg图片文件名
# 开发和编译环境
本项目使用windows vscode加remote development扩展插件，cmake编译脚本。  
项目运行在RK3588 Ubuntu22.04主机

# 编译全部
`cmake -B build`  
`cmake --build build`  
# 运行
cd Install/Bin
./v4l2_to_ffmpeg /dev/video55 /run/jpegs 3840 2160 30
其中/dev/video55应该是安装的mipi摄像头的设备节点
/run/jpegs是存放jpeg图片的目录，在ram中，文件以上电后采集的图片微妙时间命名
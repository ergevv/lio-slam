基于ros的工程运行：
	1、
	mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/src 
	catkin_init_workspace 
	catkin_init_workspace

	catkin_init_workspace 是Catkin工具的一部分，用于在一个已存在的空目录（通常就是 src 目录）中初始化一个Catkin工作空间。当在命令行中执行这个命令时，它会在当前目录（即 ~/catkin_ws/src）下自动生成一个 CMakeLists.txt 文件。
	这个 CMakeLists.txt 文件是一个CMake配置文件，它告诉Catkin和CMake如何构建、编译和安装该工作空间中的所有ROS包。通过这个初始化步骤，你就为添加ROS包并对其进行编译和管理准备好了环境。
	2、修改CMakeLists.txt
	3、catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes  //输出编译信息
	4、运行前加载环境变量：source	./devel/setup.bash
	5、运行.launch文件：roslaunch lvi_sam(包名) run.launch  （对应.launch文件，无需路径，.launch文件是写死位置的）


    
vscode调试添加环境：
1、echo ~/桌面/开源工程/VINS-mono-with-drt/devel/setup.bash >> ~/.bashrc
2、source ~/.bashrc
3、重启vscode
# dancer_car
The control of four wheel car, mechanical arm and camera in rviz and gazebo

将文件放入catkin_ws/src，终端中输入
```
cd ~/catkin_ws/src
catkin_make
source devel/setup.bash
```

rviz中运行
```
roslaunch car_model display.launch
```

gazebo中运行
```
roslaunch car_model gazebo.launch
```

control启动
```
roslaunch car_control robot_control.launch
```

手动输入指令
```
rostopic pub -1 /car_model/joint_column_link1_controller/command std_msgs/Float64 "data: 1.5"
rostopic pub -1 /car_model/joint_column_link2_controller/command std_msgs/Float64 "data: 1.5"
rostopic pub -1 /car_model/joint_column_link3_controller/command std_msgs/Float64 "data: 1.5"
```

rqt发布指令
```
rosrun rqt_gui rqt_gui
```
在Topics中选择Message Publisher，
添加/car_model/joint_column_link1_controller/command
设置frequency为100Hz，设置data为sin(i/200)*0.2+0.2
添加/car_model/joint_column_link2_controller/command
设置frequency为100Hz，设置data为sin(i/100)*0.4
添加/car_model/joint_column_link3_controller/command
设置frequency为100Hz，设置data为sin(i/50)
在前面的框中打钩，可见小车运动。

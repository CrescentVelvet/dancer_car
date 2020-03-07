# **dancer_car**
The control of four wheel car, mechanical arm and camera in rviz and gazebo
# 使用指南
## 环境配置
安装Ubuntu18.04，ros-melodic，gazebo
将文件放入catkin_ws/src，终端中输入
```
cd ~/catkin_ws/src
catkin_make
source devel/setup.bash
```
## rviz中运行（查看模型）
```
roslaunch car_model display.launch
```
Add RobotModule和camera，修改Fixed Frame为world_link，可以在gui上控制关节转动
## gazebo中运行（加载场景地图）
将car_model中worlds中的car_world.jpg复制进/usr/share/gazebo-9/media/material/textures中，

将car_model中worlds中的gazebo.material复制进/usr/share/gazebo-9/media/material/scripts中。
## gazebo中运行（键盘控制小车）
```
roslaunch car_model gazebo.launch
```
View Axis和Interia.gazebo运行时在rviz中camera的ImageTopic设置为camera

可以看见相机图像。

控制前进后退与转弯:

   u            i            o

   j            k            l

   m            <            >


控制加速减速与角度:

q/a : increase/decrease only linear  speed by 10%

w/s : increase/decrease only angular speed by 10%


控制三根杆子运动：

   r            t

   f            g

   v            b

## control启动（控制吊杆运动）（无用）
```
roslaunch car_control robot_control.launch
```

### 可以手动输入指令（无用）

```
rostopic pub -1 /car_model/joint_column_link1_controller/command std_msgs/Float64 "data: 1.5"
rostopic pub -1 /car_model/joint_column_link2_controller/command std_msgs/Float64 "data: 1.5"
rostopic pub -1 /car_model/joint_column_link3_controller/command std_msgs/Float64 "data: 1.5"
```
输入指令为转动关节角度的弧度值[-PI,PI]
### 也可以使用rqt发布指令（无用）
```
rosrun rqt_gui rqt_gui
```
在Topics中选择Message Publisher，

添加/car_model/joint_column_link1_controller/command

设置frequency为100Hz，设置data为sin(i/100)

添加/car_model/joint_column_link2_controller/command

设置frequency为100Hz，设置data为sin(i/100)*3

添加/car_model/joint_column_link3_controller/command

设置frequency为100Hz，设置data为sin(i/50)

在前面的框中打钩，控制杆子转动

***
# 错误提示
## catkin_make编译时
### 找不到effort_controllers
```
sudo apt-get install ros-melodic-effort-controllers
```
### 找不到joint_state_publisher
```
sudo apt-get install ros-melodic-joint-state-publisher-gui
```
### 找不到gazebo
CMakeLists.txt和package.xml中删去gazebo相关内容
## rviz运行时
### 没有gui控制关节
display.launch中<gui default="False" />修改为True
### 警告'state_publisher' executable is deprecated
display.launch中<type="state_publisher" />修改为robot_state_publisher
### 警告'use_gui' parameter was specified
忽略
## gazebo运行时
### 找不到gazebo/spawn_model
gazebo.launch中pkg="gazebo"修改为gazebo_ros
### 找不到gazebo_world
gazebo.launch中file="$(find gazebo_world)修改为gazebo_ros
### 找不到pr2-controller-manager
```
sudo apt-get install ros-melodic-pr2-controller-manager
```
### 运行gazebo黑屏
```
killall gzserver
```
### 找不到robot_description
gazebo.launch中中添加

使用urdf：<param name="robot_description" textfile="$(find car_model)/robots/car_model.URDF"/>

使用xacro：<param name="robot_description" command="$(find xacro)/xacro --inorder '$(find car_model)/robots/car_model.xacro'" />

### 警告KDL does not support a root link with an inertia
给root link添加一个不具有inertia的parent
### 警告Deprecated syntax
car_model.xacro中的transmission的EffortJointInterface前面加上hardware_interface/
### 找不到controller_manager
```
sudo apt-get install ros-melodic-gazebo-ros-control
```
### 找不到effort_controllers
```
sudo apt-get install ros-melodic-effort-controllers
```
### 找不到fake_localization
```
sudo apt-get install ros-melodic-fake-localization
```
### 警告new node registered with same name
节点名称冲突，删去gazebo.launch中的<include file="$(find pr2_controller_manager)/controller_manager.launch" />
### 警告找不到api.ignitionfuel.org或报错Error in REST request
home/.ignition/fuel/config.yaml中的URL行的api.ignitionfuel.org修改为api.ignitionrobotics.org
### 报错节点冲突new node registered with same name
gazebo.launch中删去<include file="$(find pr2_controller_manager)/controller_manager.launch" />
### 小车各个零部件固定在原点一动不动，或者小车狂舞乱飞无法无天
质量、摩擦力设置不当（可能过小）
PID参数设置有误，car_control.yaml中修改
### 报错cmd /opt/ros/melodic/lib/gazebo_ros/spawn_model
gazebo.launch中$(find car_model)/robots/car_model.URDF-urdf -model car_model"中间加个空格
### 新建gazebo添加模型
```
roslaunch gazebo_ros empty_world.launch
```
使用urdf
```
rosrun gazebo_ros spawn_model -file $(find car_model)/robots/car_model.URDF -urdf -z 1 -model car_model.URDF
```
使用xacro
```
rosrun gazebo_ros spawn_model -file $(find car_model)/robots/car_model.xacro -xacro -z 1 -model car_model.xacro
```
### 找不到teleop_twist_keyboard
```
sudo apt-get install ros-melodic-teleop-twist-keyboard
rosstack profile
rospack profile
```
现已加入car_keyboard
***
# 附加信息
## SolidWorks生成urdf模型
### 插件安装
github上有solidworks2urdf插件，但是原作者删除了exe版本，需要自己编译

CSDN上可以下载exe版本

安装在SolidWorks安装文件夹内即可
### 转轴配置
对于装配体，选择file导出到urdf，在PropertyManager建立link和子link，选择对应的零部件

在FeatureManager，选择Insert ReferenceGeometry，选择Axis轴，圆柱体中心线，重命名
### 坐标系
在FeatureManager，选择Insert ReferenceGeometry，选择Point点，圆形中点

与转轴一起选中，插入Coordinate System，点击x轴前面的箭头调整方向，重命名

在Preview and Export里把每个link对应的转轴与坐标系选择好，输出为文件夹
## 检查urdf文件
check_urdf <urdf>：检查urdf有无语法错误
  
urdf_to_graphiz <urdf>：生成gv和pdf两个图片文件展示urdf里的父子关系
  
gz sdf -p <urdf>：urdf转换为sdf文件
## 仿真模型
```
catkin_create_pkg car_model joint_state_publisher robot_state_publisher rviz gazebo_plugins gazebo_ros gazebo_ros_control xacro gazebo car_control
```
编写xacro文件和launch文件，link与joint，以及collision、visual、inertial
## 控制模型
```
catkin_create_pkg car_control controller_manager joint_state_controller robot_state_publisher rqt_gui effort_controllers rviz gazebo_ros gazebo_plugins gazebo_ros_control xacro gazebo
```
编写yaml文件和launch文件，gazebo reference、transmission、gazebo plugin
## 运行检查
运行rviz和gazebo和control，
```
rosrun rqt_graph rqt_graph
rostopic pub -1 /car_model/joint_column_link1_controller/command std_msgs/Float64 "data: 1.5"
```
刷新，显示node状态关系图
```
rostopic list -v
rostopic echo <topic>
rostopic type <topic>
```
列出所有node订阅和发布topic的信息

查看node在指定topic上发布的数据及类型
```
rosrun rqt_console rqt_console
```
查看node输出信息

## 控制指令
sin(i/rate*speed)*diff + offset

rate是频率，speed是速度系数，diff是上下界之差，offset是上界减去diff

# UAM
*for unmanned aerial manipulation*  
*Cooperative Intelligence of Unmanned System, SJTU*

## Set Up
`sudo apt-get install ros-kinetic-trac-ik`  


## Version Control
### V1.0 Enable basic control on PC and control for real Arm
#### Usage:
`$ roslaunch uam_arm AutoTRACIK.launch`  
*if RVIZ can not display right, change the Fixed Frame to "base_link"*   
to drive the real Arm:  
`$ sudo chmod 777 /dev/ttyUSB0` 
`$ rosrun uam_arm ftservo`
then open the DC power for Arm  
#### Frame of Code
* 获取URDF模型  
> 该版本采用的是机械臂放置在的方式，采用的是*uav2.urdf*的模型  
* 通过摇杆发布期望位置
> 该版本的摇杆部分利用*arm_joy.launch*运行，该launch文件
中，调用了*turtle_joy*的摇杆控制（采用了ROS的标准库）。
并在launch文件中对joy的按键进行定义  
运行了*Joystick_node.cpp*，发布了"arm/cmd_pose"的消息，
该消息发布的即为机械臂末端的位置和姿态信息    
 * 通过Trac-IK进行运动学逆解求解
 > 这一部分体现在*TRACIKsolver.cpp*中，通过订阅上述摇杆发
 出的位置指令，进行运动学逆解求解。由于实际测试发现存在奇异点
 导致关节转角过大，因此对机械臂的前3个关节进行了限位判断。

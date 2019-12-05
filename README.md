# UAM
*for unmanned aerial manipulation*  
*Cooperative Intelligence of Unmanned System, SJTU*

## Set Up
`sudo apt-get install ros-kinetic-trac-ik`  

## Version Control
### V1.0 Enable basic control on PC
xxx  
### V2.0 Enable stability of End-Effector
*Using fake UAV pose*
#### Usage:
`$ roslaunch uam_arm AutoTRACIK.launch`  
*if first try not ok for NOT SOLVE, just try again*  
#### Frame of Code
* 获取URDF模型  
> 该版本采用的是无人机挂机械臂的模式，和在桌面上使用采用了
不同的URDF模型，此处采用的是*uam_urdf.urdf*的模型  
* 通过摇杆发布期望位置
> 该版本的摇杆部分利用*arm_joy.launch*运行，该launch文件
中，调用了*turtle_joy*的摇杆控制（采用了ROS的标准库）。
并在launch文件中对joy的按键进行定义  
运行了*Joystick_node.cpp*，发布了"arm/cmd_pose"的消息，
该消息发布的即为机械臂末端的位置和姿态信息  
* 通过订阅无人机位置进行增稳控制
> 本版本仅为仿真演示版本，原理为：录制一段无人机定点飞行的log，
以实际频率读取log并发布出来，传给逆解程序。  
通过*uav_pose.py*读取csv文件数据，注意！此python文件中包含
csv文件的绝对路径，需进行修改。该python发布PoseStamped格式的
 *uav/uav_pose*消息，包含无人机的位置和姿态。  
 利用*drone_tf_broadcaster.cpp*对无人机数据进行处理，得到
 了无人机在全局坐标系中的位置和姿态。通过*sendTransform*,将这一
 变换关系体现在“uav_Link“相对于“world”的变换关系并发布出来。  
 * 通过Trac-IK进行运动学逆解求解
 > 这一部分体现在*TRACIKsolver.cpp*中，重点是坐标变换的关系要明确。  
 对于机械臂的末端增稳控制，有以下的公式：
 F_world_end = F_world_arm · F_uav_base · F_base_end  
 其中，F_A_B表示的是B在A下的Frame表示。  
 >> * 外部通过摇杆输入的指令转化为了 F_world_end 的Frame表示；  
 >> * 测得的无人机的运动转化为了 F_world_arm 的Frame表示；  
 >> * 基座相对于无人机的坐标变换（固定值，由安装决定）由 F_uav_base 的Frame表示；    
 >> * 而机械臂的逆解，求解的是 F_base_end 的关系，即在逆解算法中，
 输入是 F_base_end 的变换关系，求得的输出是机械臂各个关节的转角。
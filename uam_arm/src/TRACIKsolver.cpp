#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <trac_ik/trac_ik.hpp>
#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

#define PI (3.1415926535897932346f)

KDL::Vector EE_xyz = KDL::Vector(0.18, 0.0, -0.28);
KDL::Rotation EE_rpy = KDL::Rotation::RPY(0.0, 1.5708, 0.0);

KDL::Vector UAV_xyz = KDL::Vector(0, 0, 0);
KDL::Rotation UAV_rot = KDL::Rotation::Quaternion(0, 0, 0, 1);

void mode_decoder(int mode_index);

bool joint_pub_flag = true;
bool reset_flag = false;
bool start_flag = false;
bool absolute_flag = false;
bool virtual_record_enable = true;


//外部遥操作输入
bool pose_Init = true;
geometry_msgs::Pose last_pose;
void EE_pose_cb(const geometry_msgs::Pose &pose)
{

    KDL::Rotation Temp_rpy = KDL::Rotation::RPY(0.0, 0.0, 0.0);

    EE_xyz = KDL::Vector(pose.position.x, pose.position.y, pose.position.z);

    if(pose_Init || reset_flag){    //首次运算或复位状态下，将末端姿态直接输出
        EE_rpy = KDL::Rotation::RPY(pose.orientation.x, pose.orientation.y, pose.orientation.z);
        pose_Init = false;
    } else{
        Temp_rpy = KDL::Rotation::RPY(pose.orientation.x - last_pose.orientation.x, pose.orientation.y - last_pose.orientation.y, pose.orientation.z - last_pose.orientation.z);
        if(absolute_flag){
            EE_rpy = Temp_rpy * EE_rpy; //相对世界坐标系旋转
        } else{
            EE_rpy = EE_rpy * Temp_rpy; //相对末端坐标系旋转
        }
    }

    mode_decoder(pose.orientation.w);   //运行模式解析

    last_pose = pose;
}

//无人机位置输入
void uam_pose_cb(const geometry_msgs::PoseStamped &msg)
{
    UAV_xyz = KDL::Vector(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    UAV_rot = KDL::Rotation::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
}


int main(int argc, char** argv)
{
    //ros
    ros::init(argc, argv, "trac_ik_solver");
    ros::NodeHandle nh;

    ros::Subscriber EE_pose_sub = nh.subscribe("/arm/cmd_pose", 10, EE_pose_cb);
    ros::Subscriber Base_pose_sub = nh.subscribe("/uam_base/pose", 10, uam_pose_cb);

    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

    ros::Rate loop (50);


    //Trac_IK
    TRAC_IK::TRAC_IK tracik_solver("base_link", "end_link", "/robot_description", 0.005, 1E-5, TRAC_IK::Distance);
    KDL::Chain chain;
    KDL::JntArray ll, ul; //lower joint limits, upper joint limits

    bool valid = tracik_solver.getKDLChain(chain);
    if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
        return 0;
    }

    valid = tracik_solver.getKDLLimits(ll,ul);
    if (!valid) {
        ROS_ERROR("There were no valid KDL joint limits found");
        return 0;
    }
    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());

    // Create Nominal chain configuration midway between all joint limits
    KDL::JntArray nominal(chain.getNrOfJoints());
    for (uint j=0; j<nominal.data.size(); j++) {
        nominal(j) = (ll(j)+ul(j))/2.0;
    }
    ROS_INFO("Using %d joints", chain.getNrOfJoints());

    unsigned int nj = chain.getNrOfJoints();
    sensor_msgs::JointState joint_state;
    unsigned int count = 0;

    KDL::Frame F_uav_base  = KDL::Frame(KDL::Rotation::RPY(3.1416, 0, 0), KDL::Vector(0, 0, -0.06));
    KDL::Frame F_base_EE_init = F_uav_base.Inverse() * KDL::Frame(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(0.18, 0, -0.30));
    KDL::Frame F_world_uav, F_world_virtual, F_virtual_EE, F_base_EE;

    while(ros::ok()){
        KDL::Frame cartpos_init;
        KDL::JntArray result;

        F_world_uav = KDL::Frame(UAV_rot, UAV_xyz);
        F_virtual_EE = KDL::Frame(EE_rpy, EE_xyz);

        if(start_flag){
            if (virtual_record_enable){     //记录启动增稳控制时无人机所处的位置
                F_world_virtual = F_world_uav;
                virtual_record_enable = false;
            }
            F_base_EE   = F_uav_base.Inverse() * F_world_uav.Inverse() * F_world_virtual * F_virtual_EE;
        }
        else{
            F_base_EE   = F_base_EE_init;
        }

        int kinematics_status;
        kinematics_status = tracik_solver.CartToJnt(nominal,F_base_EE,result);

        if(kinematics_status >= 0)
        {
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(nj);
            joint_state.position.resize(nj);
            for(unsigned int i=0;i<nj;i++)
            {
                std::stringstream ss;
                ss<<i;
                joint_state.name[i] = "revolute_joint_" + ss.str();
                joint_state.position[i] = result(i);
            }
            if(count < 10)
            {
                cartpos_init = F_base_EE; //todo 目前这里保留的是最早的关节求解关系，后期可以直接采用关节转角
            }

            //计算两次转角的差，避免大范围的摆动
            KDL::JntArray delta;
            KDL::Subtract(result, nominal, delta);
            for(unsigned int i=0;i<nj-3;i++)
            {
                if(abs(delta(i)) > 0.314 && count > 100 && !reset_flag)     //count 避免启动瞬间的大位移无法求解现象
                {
                    ROS_WARN_STREAM("Rapid Rotation, unsafe for Arm! ");
                    joint_pub_flag = false;
                    break;
                }
            }

            if(joint_pub_flag)
            {
                joint_state_pub.publish(joint_state);
                nominal = result;
            }
            joint_pub_flag = true;

        }
        else{ROS_ERROR_STREAM("Can NOT Solve!");}

        ros::spinOnce();
        loop.sleep();
        count ++;
    }
    return 0;
}

void mode_decoder(int mode_index){
//    std::cout<<"mode_index:"<<mode_index<<std::endl;
    //模式判断部分
    switch (mode_index)
    {
        case (0):{    //未操作时，不执行reset操作，且不对其他进行控制
            reset_flag = false;
            break;
        }
        case (1):{    //按下 X 按钮，reset复位，同时解除增稳控制，并启动下次记录虚拟坐标系的flag
            reset_flag = true;
            start_flag = false;
            virtual_record_enable = true;
            break;
        }
        case (2):{    //按下 O 按钮，开始增稳控制，此时会确定虚拟坐标系相对世界坐标系的位姿，并以此进行增稳控制
            start_flag = true;
            break;
        }
        case (4):{    //按下 ^ 按钮，末端将相对虚拟坐标系进行旋转控制
            absolute_flag = true;
            break;
        }
        default:{   //同时按下以上三个按钮的任意2个or3个，系统将复位至最初上电状态
            reset_flag = true;
            start_flag = false;
            virtual_record_enable = true;
            absolute_flag = false;
            break;
        }
    }
}
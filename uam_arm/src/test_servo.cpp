#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>

#include <dynamic_reconfigure/server.h>
#include <uam_arm/uam_Config.h>

uam_arm::uam_Config debug_joint_state;

void joint_state_cb(const uam_arm::uam_Config &config, uint32_t level)
{
    debug_joint_state = config;
}





int main(int argc, char** argv)
{
    //ros
    ros::init(argc, argv, "test_servo");
    ros::NodeHandle n;

    dynamic_reconfigure::Server<uam_arm::uam_Config> server;
    dynamic_reconfigure::Server<uam_arm::uam_Config>::CallbackType ff;
    ff = boost::bind(&joint_state_cb, _1, _2);
    server.setCallback(ff);

    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Rate loop (50);

    sensor_msgs::JointState joint_state;

    while(ros::ok()){
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(7);
        joint_state.position.resize(7);
        for(int i=0;i<7;i++)
        {
            std::stringstream ss;
            ss<<i;
            joint_state.name[i] = "debug_joint_" + ss.str();

        }
        joint_state.position[0] = debug_joint_state.servo_1;
        joint_state.position[1] = debug_joint_state.servo_2;
        joint_state.position[2] = debug_joint_state.servo_3;
        joint_state.position[3] = debug_joint_state.servo_4;
        joint_state.position[4] = debug_joint_state.servo_5;
        joint_state.position[5] = debug_joint_state.servo_6;
        joint_state.position[6] = debug_joint_state.Key;


        joint_state_pub.publish(joint_state);

        ros::spinOnce();
        loop.sleep();
    }
    return 0;

}

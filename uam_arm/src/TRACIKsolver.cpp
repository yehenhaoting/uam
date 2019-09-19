#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolver.hpp>

#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <trac_ik/trac_ik.hpp>

#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/String.h"
#include <sstream>
#include <string>

KDL::Vector EE_xyz = KDL::Vector(0.18, 0.0, 0.28);
KDL::Rotation EE_rpy = KDL::Rotation::RPY(0.0, 1.5708, 0.0);
bool joint_pub_flag = true;
bool reset_flag = false;


void pose_cb(const geometry_msgs::Pose& pose)
{

    EE_xyz = KDL::Vector(pose.position.x, pose.position.y, pose.position.z);
    EE_rpy = KDL::Rotation::RPY(pose.orientation.x, pose.orientation.y, pose.orientation.z);
    reset_flag = bool(pose.orientation.w);

}


int main(int argc, char** argv)
{
    //ros
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Subscriber EEpose_sub = n.subscribe("/arm/cmd_pose", 10, pose_cb);

    ros::Rate loop (50);


    //Trac_IK
    TRAC_IK::TRAC_IK tracik_solver("base_link", "end_link", "/robot_description", 0.005, 1E-5, TRAC_IK::Speed); //TRAC_IK::SolveType  Distance
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

    double eps=1E-3;
    int maxiter=1000;
    double eps_joints=1E-15;

    sensor_msgs::JointState joint_state;
    unsigned int count = 0;
    while(ros::ok()){

        KDL::Frame cartpos = KDL::Frame(EE_rpy, EE_xyz);
        KDL::Frame cartpos_init;
        KDL::JntArray result;

        int kinematics_status;
        kinematics_status = tracik_solver.CartToJnt(nominal,cartpos,result);

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
                cartpos_init = cartpos;
            }
            
            KDL::JntArray delta;
            KDL::Subtract(result, nominal, delta);
            for(unsigned int i=0;i<nj-3;i++)
            {
                if(abs(delta(i)) > 0.314 && count > 1 && !reset_flag)
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

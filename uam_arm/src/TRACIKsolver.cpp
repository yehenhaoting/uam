#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
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

using namespace KDL;
using namespace std;

Vector EE_xyz = Vector(0.18, 0.0, 0.28);
Rotation EE_rpy = Rotation::RPY(0.0, 1.5708, 0.0);

void pose_cb(const geometry_msgs::Pose& pose)
{
    EE_xyz = Vector(pose.position.x, pose.position.y, pose.position.z);
    EE_rpy = Rotation::RPY(pose.orientation.x, pose.orientation.y, pose.orientation.z);

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

    KDL::JntArray nominal(chain.getNrOfJoints()); // Create Nominal chain configuration midway between all joint limits
    unsigned int nj = chain.getNrOfJoints();
    for (uint j=0; j<nominal.data.size(); j++) {
        nominal(j) = (ll(j)+ul(j))/2.0;
    } 

    //KDL
    Tree my_tree;
    kdl_parser::treeFromParam("/robot_description",my_tree);

    Chain kdl_chain1;
    Chain kdl_chain2;
    Chain kdl_chain3;
    Chain kdl_chain4;
    Chain kdl_chain5;
    Chain kdl_chain6;

    my_tree.getChain("base_link","link1",kdl_chain1);
    my_tree.getChain("base_link","link2",kdl_chain2);
    my_tree.getChain("base_link","link3",kdl_chain3);
    my_tree.getChain("base_link","link4",kdl_chain4);
    my_tree.getChain("base_link","link5",kdl_chain5);
    my_tree.getChain("base_link","link6",kdl_chain6);

    double eps=1E-5;
    int maxiter=1000;
    double eps_joints=1E-15;

    ChainFkSolverPos_recursive kdl_fksolver1 = ChainFkSolverPos_recursive(kdl_chain1);
    ChainFkSolverPos_recursive kdl_fksolver2 = ChainFkSolverPos_recursive(kdl_chain2);
    ChainFkSolverPos_recursive kdl_fksolver3 = ChainFkSolverPos_recursive(kdl_chain3);
    ChainFkSolverPos_recursive kdl_fksolver4 = ChainFkSolverPos_recursive(kdl_chain4);
    ChainFkSolverPos_recursive kdl_fksolver5 = ChainFkSolverPos_recursive(kdl_chain5);
    ChainFkSolverPos_recursive kdl_fksolver6 = ChainFkSolverPos_recursive(kdl_chain6);

    unsigned int nj1 = kdl_chain1.getNrOfJoints();
    unsigned int nj2 = kdl_chain2.getNrOfJoints();
    unsigned int nj3 = kdl_chain3.getNrOfJoints();
    unsigned int nj4 = kdl_chain4.getNrOfJoints();
    unsigned int nj5 = kdl_chain5.getNrOfJoints();
    unsigned int nj6 = kdl_chain6.getNrOfJoints();

    JntArray kdl_jointpositions1 = JntArray(nj1);
    JntArray kdl_jointpositions2 = JntArray(nj2);
    JntArray kdl_jointpositions3 = JntArray(nj3);
    JntArray kdl_jointpositions4 = JntArray(nj4);
    JntArray kdl_jointpositions5 = JntArray(nj5);
    JntArray kdl_jointpositions6 = JntArray(nj6);

    Frame kdl_cartpos1;
    Frame kdl_cartpos2;
    Frame kdl_cartpos3;
    Frame kdl_cartpos4;
    Frame kdl_cartpos5;
    Frame kdl_cartpos6;

    sensor_msgs::JointState joint_state;
    int count=0;
    while(ros::ok()){
        KDL::Frame cartpos = Frame(EE_rpy, EE_xyz);
        KDL::JntArray jointpositions;

        int kinematics_status;
        kinematics_status = tracik_solver.CartToJnt(nominal,cartpos,jointpositions);

        if(kinematics_status >= 0)
        {
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(nj);
            joint_state.position.resize(nj);
            for(int i=0;i<nj;i++)
            {
                stringstream ss;
                ss<<i;
                joint_state.name[i] = "revolute_joint_" + ss.str();
                joint_state.position[i] = jointpositions(i);
                nominal(i)=jointpositions(i);
            }
            for(unsigned int i=0;i<nj1;i++){
                kdl_jointpositions1(i)=jointpositions(i);
            }
            for(unsigned int i=0;i<nj2;i++){
                kdl_jointpositions2(i)=jointpositions(i);
            }
            for(unsigned int i=0;i<nj3;i++){
                kdl_jointpositions3(i)=jointpositions(i);
            }
            for(unsigned int i=0;i<nj4;i++){
                kdl_jointpositions4(i)=jointpositions(i);
            }
            for(unsigned int i=0;i<nj5;i++){
                kdl_jointpositions5(i)=jointpositions(i);
            }
            for(unsigned int i=0;i<nj6;i++){
                kdl_jointpositions6(i)=jointpositions(i);
            }

            kdl_fksolver1.JntToCart( kdl_jointpositions1,kdl_cartpos1);
            kdl_fksolver2.JntToCart( kdl_jointpositions2,kdl_cartpos2);
            kdl_fksolver3.JntToCart( kdl_jointpositions3,kdl_cartpos3);
            kdl_fksolver4.JntToCart( kdl_jointpositions4,kdl_cartpos4);
            kdl_fksolver5.JntToCart( kdl_jointpositions5,kdl_cartpos5);
            kdl_fksolver6.JntToCart( kdl_jointpositions6,kdl_cartpos6);
//            if(kdl_cartpos6.p.z()<-0.035&&kdl_cartpos5.p.z()<-0.035&&kdl_cartpos4.p.z()<-0.035&&kdl_cartpos3.p.z()<-0.035&&kdl_cartpos2.p.z()<-0.035)
//            {
//                joint_state_pub.publish(joint_state);
//            }
            joint_state_pub.publish(joint_state);

        }
        else{cout<<"error!!!!!"<<endl;}

//        cout<<kdl_cartpos1.p<<"_"<<kdl_cartpos2.p<<"_"<<kdl_cartpos3.p<<"_"<<kdl_cartpos4.p<<"_"<<kdl_cartpos5.p<<"_"<<kdl_cartpos6.p<<endl;
        ros::spinOnce();
        loop.sleep();
    }

}

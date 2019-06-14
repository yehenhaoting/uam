#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolver.hpp>
<<<<<<< HEAD
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
=======
>>>>>>> d4895e3771e61fb031a25039ae990c8c0d6d995f
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <trac_ik/trac_ik.hpp>
<<<<<<< HEAD
=======

>>>>>>> d4895e3771e61fb031a25039ae990c8c0d6d995f
#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/String.h"
#include <sstream>
#include <string>

<<<<<<< HEAD
using namespace KDL;
using namespace std;

<<<<<<< HEAD
double x=0.12,y=0,z=-0.2,w=100;
=======

double x=0.25,y=0,z=0.32,w=100;
>>>>>>> d4895e3771e61fb031a25039ae990c8c0d6d995f
=======
Vector EE_xyz = Vector(0.18, 0.0, 0.28);
Rotation EE_rpy = Rotation::RPY(0.0, 1.5708, 0.0);
bool joint_pub_flag = true;
bool reset_flag = false;
>>>>>>> 31cb061caf1041fd79889e3255a8fe5c84f435c0

void pose_cb(const geometry_msgs::Pose& pose)
{
<<<<<<< HEAD
  x=msg.x;
  y=msg.y;
  z=msg.z;
  w=msg.w;
<<<<<<< HEAD
  cout<<"x:"<<x<<" y:"<<y<<" z:"<<z<<" w:"<<w<<endl;
=======
  std::cout<<"x:"<<x<<" y:"<<y<<" z:"<<z<<" w:"<<w<<std::endl;
>>>>>>> d4895e3771e61fb031a25039ae990c8c0d6d995f
=======
    EE_xyz = Vector(pose.position.x, pose.position.y, pose.position.z);
    EE_rpy = Rotation::RPY(pose.orientation.x, pose.orientation.y, pose.orientation.z);
    reset_flag = bool(pose.orientation.w);

>>>>>>> 31cb061caf1041fd79889e3255a8fe5c84f435c0
}


int main(int argc, char** argv)
{
    //ros
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
<<<<<<< HEAD
<<<<<<< HEAD
    ros::Subscriber joint_state_sub = n.subscribe("position", 1, chatterCallback);
=======
    ros::Subscriber end_effector_sub = n.subscribe("position", 10, chatterCallback);
>>>>>>> d4895e3771e61fb031a25039ae990c8c0d6d995f
=======
    ros::Subscriber EEpose_sub = n.subscribe("/arm/cmd_pose", 10, pose_cb);
>>>>>>> 31cb061caf1041fd79889e3255a8fe5c84f435c0
    ros::Rate loop (50);


    //Trac_IK
<<<<<<< HEAD
<<<<<<< HEAD
    TRAC_IK::TRAC_IK tracik_solver("base_link", "ee_link", "/robot_description", 0.005, 1E-5);
=======
    TRAC_IK::TRAC_IK tracik_solver("base_link", "end_link", "/robot_description", 0.005, 1E-5, TRAC_IK::Distance);
>>>>>>> 31cb061caf1041fd79889e3255a8fe5c84f435c0
    KDL::Chain chain;
    KDL::JntArray ll, ul; //lower joint limits, upper joint limits
    bool valid = tracik_solver.getKDLChain(chain);
 
=======
    TRAC_IK::TRAC_IK tracik_solver("base_link", "end_link", "/robot_description", 0.005, 1E-5, TRAC_IK::Speed); //TRAC_IK::SolveType  Distance
    KDL::Chain chain;
    KDL::JntArray ll, ul; //lower joint limits, upper joint limits
    bool valid = tracik_solver.getKDLChain(chain);

>>>>>>> d4895e3771e61fb031a25039ae990c8c0d6d995f
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
<<<<<<< HEAD
<<<<<<< HEAD
    } 

    //KDL
    Tree my_tree;
    kdl_parser::treeFromFile("/home/ubuntu/uam_ws/src/uam_urdf/urdf/uav2.urdf",my_tree);

    Chain kdl_chain1;
    Chain kdl_chain2;
    Chain kdl_chain3;
    Chain kdl_chain4;
    Chain kdl_chain5;
    Chain kdl_chain6;
=======
    }

    //KDL
    KDL::Tree my_tree;
    kdl_parser::treeFromFile("/home/zm/uam_ws/src/robot_urdf/urdf/robot_urdf.urdf",my_tree);

    KDL::Chain kdl_chain1, kdl_chain2, kdl_chain3, kdl_chain4, kdl_chain5, kdl_chain6;
>>>>>>> d4895e3771e61fb031a25039ae990c8c0d6d995f

    my_tree.getChain("base_link","link1",kdl_chain1);
    my_tree.getChain("base_link","link2",kdl_chain2);
    my_tree.getChain("base_link","link3",kdl_chain3);
    my_tree.getChain("base_link","link4",kdl_chain4);
    my_tree.getChain("base_link","link5",kdl_chain5);
    my_tree.getChain("base_link","link6",kdl_chain6);

    double eps=1E-5;
    int maxiter=1000;
    double eps_joints=1E-15;

<<<<<<< HEAD
    ChainFkSolverPos_recursive kdl_fksolver1 = ChainFkSolverPos_recursive(kdl_chain1);
    ChainFkSolverPos_recursive kdl_fksolver2 = ChainFkSolverPos_recursive(kdl_chain2);
    ChainFkSolverPos_recursive kdl_fksolver3 = ChainFkSolverPos_recursive(kdl_chain3);
    ChainFkSolverPos_recursive kdl_fksolver4 = ChainFkSolverPos_recursive(kdl_chain4);
    ChainFkSolverPos_recursive kdl_fksolver5 = ChainFkSolverPos_recursive(kdl_chain5);
    ChainFkSolverPos_recursive kdl_fksolver6 = ChainFkSolverPos_recursive(kdl_chain6);
=======
    KDL::ChainFkSolverPos_recursive kdl_fksolver1 = KDL::ChainFkSolverPos_recursive(kdl_chain1);
    KDL::ChainFkSolverPos_recursive kdl_fksolver2 = KDL::ChainFkSolverPos_recursive(kdl_chain2);
    KDL::ChainFkSolverPos_recursive kdl_fksolver3 = KDL::ChainFkSolverPos_recursive(kdl_chain3);
    KDL::ChainFkSolverPos_recursive kdl_fksolver4 = KDL::ChainFkSolverPos_recursive(kdl_chain4);
    KDL::ChainFkSolverPos_recursive kdl_fksolver5 = KDL::ChainFkSolverPos_recursive(kdl_chain5);
    KDL::ChainFkSolverPos_recursive kdl_fksolver6 = KDL::ChainFkSolverPos_recursive(kdl_chain6);
>>>>>>> d4895e3771e61fb031a25039ae990c8c0d6d995f

    unsigned int nj1 = kdl_chain1.getNrOfJoints();
    unsigned int nj2 = kdl_chain2.getNrOfJoints();
    unsigned int nj3 = kdl_chain3.getNrOfJoints();
    unsigned int nj4 = kdl_chain4.getNrOfJoints();
    unsigned int nj5 = kdl_chain5.getNrOfJoints();
    unsigned int nj6 = kdl_chain6.getNrOfJoints();

<<<<<<< HEAD
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

    //Rotation rot = Rotation(0.000796325,0.999999,-0.00079379,-2.01992e-9,0.00079379,1,1,-0.000796324,6.34135e-7);
    Rotation rot = Rotation::RPY(1.5707963,0,1.5707963);
=======
    }
    ROS_INFO("Using %d joints", chain.getNrOfJoints());

    unsigned int nj = chain.getNrOfJoints();


    //KDL
    Tree my_tree;
    kdl_parser::treeFromParam("/robot_description",my_tree);

//    Chain kdl_chain1;
//    Chain kdl_chain2;
//    Chain kdl_chain3;
//    Chain kdl_chain4;
//    Chain kdl_chain5;
//    Chain kdl_chain6;
//
//    my_tree.getChain("base_link","link1",kdl_chain1);
//    my_tree.getChain("base_link","link2",kdl_chain2);
//    my_tree.getChain("base_link","link3",kdl_chain3);
//    my_tree.getChain("base_link","link4",kdl_chain4);
//    my_tree.getChain("base_link","link5",kdl_chain5);
//    my_tree.getChain("base_link","link6",kdl_chain6);

    double eps=1E-3;
    int maxiter=1000;
    double eps_joints=1E-15;

//    ChainFkSolverPos_recursive kdl_fksolver1 = ChainFkSolverPos_recursive(kdl_chain1);
//    ChainFkSolverPos_recursive kdl_fksolver2 = ChainFkSolverPos_recursive(kdl_chain2);
//    ChainFkSolverPos_recursive kdl_fksolver3 = ChainFkSolverPos_recursive(kdl_chain3);
//    ChainFkSolverPos_recursive kdl_fksolver4 = ChainFkSolverPos_recursive(kdl_chain4);
//    ChainFkSolverPos_recursive kdl_fksolver5 = ChainFkSolverPos_recursive(kdl_chain5);
//    ChainFkSolverPos_recursive kdl_fksolver6 = ChainFkSolverPos_recursive(kdl_chain6);
//
//    unsigned int nj1 = kdl_chain1.getNrOfJoints();
//    unsigned int nj2 = kdl_chain2.getNrOfJoints();
//    unsigned int nj3 = kdl_chain3.getNrOfJoints();
//    unsigned int nj4 = kdl_chain4.getNrOfJoints();
//    unsigned int nj5 = kdl_chain5.getNrOfJoints();
//    unsigned int nj6 = kdl_chain6.getNrOfJoints();
//
//    JntArray kdl_jointpositions1 = JntArray(nj1);
//    JntArray kdl_jointpositions2 = JntArray(nj2);
//    JntArray kdl_jointpositions3 = JntArray(nj3);
//    JntArray kdl_jointpositions4 = JntArray(nj4);
//    JntArray kdl_jointpositions5 = JntArray(nj5);
//    JntArray kdl_jointpositions6 = JntArray(nj6);
//
//    Frame kdl_cartpos1;
//    Frame kdl_cartpos2;
//    Frame kdl_cartpos3;
//    Frame kdl_cartpos4;
//    Frame kdl_cartpos5;
//    Frame kdl_cartpos6;

>>>>>>> 31cb061caf1041fd79889e3255a8fe5c84f435c0
    sensor_msgs::JointState joint_state;
    unsigned int count = 0;

    while(ros::ok()){
<<<<<<< HEAD
        Vector vector = Vector(x,y,z);
        KDL::Frame cartpos = Frame(rot,vector);
=======
    KDL::JntArray kdl_jointpositions1 = KDL::JntArray(nj1);
    KDL::JntArray kdl_jointpositions2 = KDL::JntArray(nj2);
    KDL::JntArray kdl_jointpositions3 = KDL::JntArray(nj3);
    KDL::JntArray kdl_jointpositions4 = KDL::JntArray(nj4);
    KDL::JntArray kdl_jointpositions5 = KDL::JntArray(nj5);
    KDL::JntArray kdl_jointpositions6 = KDL::JntArray(nj6);


    KDL::Frame kdl_cartpos1, kdl_cartpos2, kdl_cartpos3, kdl_cartpos4, kdl_cartpos5, kdl_cartpos6;

    KDL::Rotation rot = KDL::Rotation::RPY(-1.5707963,1.5707963,0);
    sensor_msgs::JointState joint_state;
    int count=0;
    while(ros::ok()){

        KDL::Vector vector(x,y,z);

        KDL::Frame cartpos = KDL::Frame(rot,vector);
>>>>>>> d4895e3771e61fb031a25039ae990c8c0d6d995f
        KDL::JntArray jointpositions;

        int kinematics_status;
        kinematics_status = tracik_solver.CartToJnt(nominal,cartpos,jointpositions);
<<<<<<< HEAD

        if(kinematics_status)
=======
//        kinematics_status = 1;

        if(kinematics_status >= 0)
>>>>>>> d4895e3771e61fb031a25039ae990c8c0d6d995f
=======
        KDL::Frame cartpos = Frame(EE_rpy, EE_xyz);
        KDL::Frame cartpos_init;
        KDL::JntArray result;

        int kinematics_status;
        kinematics_status = tracik_solver.CartToJnt(nominal,cartpos,result);

        if(kinematics_status >= 0)
>>>>>>> 31cb061caf1041fd79889e3255a8fe5c84f435c0
        {
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(nj);
            joint_state.position.resize(nj);
            for(unsigned int i=0;i<nj;i++)
            {
<<<<<<< HEAD
                stringstream ss;
                ss<<i;
                joint_state.name[i] = "revolute_joint_" + ss.str();
<<<<<<< HEAD
                joint_state.position[i] = jointpositions(i);
=======
                std::stringstream ss;
                ss<<i;
                joint_state.name[i] = "joint" + ss.str();
                joint_state.position[i] = jointpositions(i);
                std::cout<<jointpositions(i)<<std::endl;
>>>>>>> d4895e3771e61fb031a25039ae990c8c0d6d995f
                nominal(i)=jointpositions(i);
            }
            for(unsigned int i=0;i<nj1;i++){
                kdl_jointpositions1(i)=jointpositions(i);
=======
                joint_state.position[i] = result(i);
>>>>>>> 31cb061caf1041fd79889e3255a8fe5c84f435c0
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

<<<<<<< HEAD
            kdl_fksolver1.JntToCart( kdl_jointpositions1,kdl_cartpos1);
            kdl_fksolver2.JntToCart( kdl_jointpositions2,kdl_cartpos2);
            kdl_fksolver3.JntToCart( kdl_jointpositions3,kdl_cartpos3);
            kdl_fksolver4.JntToCart( kdl_jointpositions4,kdl_cartpos4);
            kdl_fksolver5.JntToCart( kdl_jointpositions5,kdl_cartpos5);
            kdl_fksolver6.JntToCart( kdl_jointpositions6,kdl_cartpos6);
<<<<<<< HEAD
            if(kdl_cartpos6.p.z()<-0.035&&kdl_cartpos5.p.z()<-0.035&&kdl_cartpos4.p.z()<-0.035&&kdl_cartpos3.p.z()<-0.035&&kdl_cartpos2.p.z()<-0.035)
=======
            if(kdl_cartpos6.p.z() > 0.035 && kdl_cartpos5.p.z() > 0.035 && kdl_cartpos4.p.z() > 0.035 && kdl_cartpos3.p.z() > 0.035 && kdl_cartpos2.p.z() > 0.035)
>>>>>>> d4895e3771e61fb031a25039ae990c8c0d6d995f
=======
            if(joint_pub_flag)
>>>>>>> 31cb061caf1041fd79889e3255a8fe5c84f435c0
            {
                joint_state_pub.publish(joint_state);
                nominal = result;
            }
            joint_pub_flag = true;


//            for(unsigned int i=0;i<nj1;i++){
//                kdl_jointpositions1(i)=result(i);
//            }
//            for(unsigned int i=0;i<nj2;i++){
//                kdl_jointpositions2(i)=result(i);
//            }
//            for(unsigned int i=0;i<nj3;i++){
//                kdl_jointpositions3(i)=result(i);
//            }
//            for(unsigned int i=0;i<nj4;i++){
//                kdl_jointpositions4(i)=result(i);
//            }
//            for(unsigned int i=0;i<nj5;i++){
//                kdl_jointpositions5(i)=result(i);
//            }
//            for(unsigned int i=0;i<nj6;i++){
//                kdl_jointpositions6(i)=result(i);
//            }
//
//            kdl_fksolver1.JntToCart( kdl_jointpositions1,kdl_cartpos1);
//            kdl_fksolver2.JntToCart( kdl_jointpositions2,kdl_cartpos2);
//            kdl_fksolver3.JntToCart( kdl_jointpositions3,kdl_cartpos3);
//            kdl_fksolver4.JntToCart( kdl_jointpositions4,kdl_cartpos4);
//            kdl_fksolver5.JntToCart( kdl_jointpositions5,kdl_cartpos5);
//            kdl_fksolver6.JntToCart( kdl_jointpositions6,kdl_cartpos6);
//            if(kdl_cartpos6.p.z()<-0.035&&kdl_cartpos5.p.z()<-0.035&&kdl_cartpos4.p.z()<-0.035&&kdl_cartpos3.p.z()<-0.035&&kdl_cartpos2.p.z()<-0.035)
//            {
//                joint_state_pub.publish(joint_state);
//            }

        }
<<<<<<< HEAD
<<<<<<< HEAD
        else{cout<<"error!!!!!"<<endl;}

        cout<<kdl_cartpos1.p<<"_"<<kdl_cartpos2.p<<"_"<<kdl_cartpos3.p<<"_"<<kdl_cartpos4.p<<"_"<<kdl_cartpos5.p<<"_"<<kdl_cartpos6.p<<endl;
=======
        else{std::cout<<"error!!!!!"<<std::endl;

        }

//        std::cout<<kdl_cartpos1.p<<"_"<<kdl_cartpos2.p<<"_"<<kdl_cartpos3.p<<"_"<<kdl_cartpos4.p<<"_"<<kdl_cartpos5.p<<"_"<<kdl_cartpos6.p<<std::endl;
//        std::cout<<kdl_cartpos6.p<<std::endl;
>>>>>>> d4895e3771e61fb031a25039ae990c8c0d6d995f
=======
        else{ROS_ERROR_STREAM("Can NOT Solve!");}

//        cout<<kdl_cartpos1.p<<"_"<<kdl_cartpos2.p<<"_"<<kdl_cartpos3.p<<"_"<<kdl_cartpos4.p<<"_"<<kdl_cartpos5.p<<"_"<<kdl_cartpos6.p<<endl;
>>>>>>> 31cb061caf1041fd79889e3255a8fe5c84f435c0
        ros::spinOnce();
        loop.sleep();
        count ++;
    }
<<<<<<< HEAD

<<<<<<< HEAD
=======
    return 0;
>>>>>>> d4895e3771e61fb031a25039ae990c8c0d6d995f
=======
    return 0;
>>>>>>> 31cb061caf1041fd79889e3255a8fe5c84f435c0
}

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
#include "std_msgs/String.h"
#include <sstream>
#include <string>


double x=0.12,y=0,z=-0.2,w=100;

void chatterCallback(const geometry_msgs::Quaternion& msg)
{
  x=msg.x;
  y=msg.y;
  z=msg.z;
  w=msg.w;
  std::cout<<"x:"<<x<<" y:"<<y<<" z:"<<z<<" w:"<<w<<std::endl;
}


int main(int argc, char** argv)
{
    //ros
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Subscriber end_effector_sub = n.subscribe("position", 10, chatterCallback);
    ros::Rate loop (50);


    //Trac_IK
    TRAC_IK::TRAC_IK tracik_solver("base_link", "ee_link", "/robot_description", 0.005, 1E-5);
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
    KDL::Tree my_tree;
<<<<<<< Updated upstream
    kdl_parser::treeFromFile("/home/zm/uam_ws/src/uam_urdf/urdf/uav2.urdf",my_tree);
//    std::string robot_desc_string;
//    n.param("robot_description", robot_desc_string, std::string());
//    kdl_parser::treeFromParam("robot_description", my_tree);


    KDL::Chain kdl_chain1;
    KDL::Chain kdl_chain2;
    KDL::Chain kdl_chain3;
    KDL::Chain kdl_chain4;
    KDL::Chain kdl_chain5;
    KDL::Chain kdl_chain6;
=======
    kdl_parser::treeFromFile("/home/ubuntu/uam_ws/src/uam_urdf/urdf/uav2.urdf",my_tree);

    KDL::Chain kdl_chain1, kdl_chain2, kdl_chain3, kdl_chain4, kdl_chain5, kdl_chain6;
>>>>>>> Stashed changes

    my_tree.getChain("base_link","link1",kdl_chain1);
    my_tree.getChain("base_link","link2",kdl_chain2);
    my_tree.getChain("base_link","link3",kdl_chain3);
    my_tree.getChain("base_link","link4",kdl_chain4);
    my_tree.getChain("base_link","link5",kdl_chain5);
    my_tree.getChain("base_link","link6",kdl_chain6);

    double eps=1E-5;
    int maxiter=1000;
    double eps_joints=1E-15;

    KDL::ChainFkSolverPos_recursive kdl_fksolver1 = KDL::ChainFkSolverPos_recursive(kdl_chain1);
    KDL::ChainFkSolverPos_recursive kdl_fksolver2 = KDL::ChainFkSolverPos_recursive(kdl_chain2);
    KDL::ChainFkSolverPos_recursive kdl_fksolver3 = KDL::ChainFkSolverPos_recursive(kdl_chain3);
    KDL::ChainFkSolverPos_recursive kdl_fksolver4 = KDL::ChainFkSolverPos_recursive(kdl_chain4);
    KDL::ChainFkSolverPos_recursive kdl_fksolver5 = KDL::ChainFkSolverPos_recursive(kdl_chain5);
    KDL::ChainFkSolverPos_recursive kdl_fksolver6 = KDL::ChainFkSolverPos_recursive(kdl_chain6);

    unsigned int nj1 = kdl_chain1.getNrOfJoints();
    unsigned int nj2 = kdl_chain2.getNrOfJoints();
    unsigned int nj3 = kdl_chain3.getNrOfJoints();
    unsigned int nj4 = kdl_chain4.getNrOfJoints();
    unsigned int nj5 = kdl_chain5.getNrOfJoints();
    unsigned int nj6 = kdl_chain6.getNrOfJoints();

    KDL::JntArray kdl_jointpositions1 = KDL::JntArray(nj1);
    KDL::JntArray kdl_jointpositions2 = KDL::JntArray(nj2);
    KDL::JntArray kdl_jointpositions3 = KDL::JntArray(nj3);
    KDL::JntArray kdl_jointpositions4 = KDL::JntArray(nj4);
    KDL::JntArray kdl_jointpositions5 = KDL::JntArray(nj5);
    KDL::JntArray kdl_jointpositions6 = KDL::JntArray(nj6);

<<<<<<< Updated upstream
    KDL::Frame kdl_cartpos1;
    KDL::Frame kdl_cartpos2;
    KDL::Frame kdl_cartpos3;
    KDL::Frame kdl_cartpos4;
    KDL::Frame kdl_cartpos5;
    KDL::Frame kdl_cartpos6;
=======
    KDL::Frame kdl_cartpos1, kdl_cartpos2, kdl_cartpos3, kdl_cartpos4, kdl_cartpos5, kdl_cartpos6;
>>>>>>> Stashed changes

    //Rotation rot = Rotation(0.000796325,0.999999,-0.00079379,-2.01992e-9,0.00079379,1,1,-0.000796324,6.34135e-7);
    KDL::Rotation rot = KDL::Rotation::RPY(1.5707963,0,1.5707963);
    sensor_msgs::JointState joint_state;
    int count=0;
    while(ros::ok()){
<<<<<<< Updated upstream
        KDL::Vector vector = KDL::Vector(x,y,z);
=======
//        KDL::Vector vector = KDL::Vector(x,y,z);
        KDL::Vector vector(x,y,z);
>>>>>>> Stashed changes
        KDL::Frame cartpos = KDL::Frame(rot,vector);
        KDL::JntArray jointpositions;

        int kinematics_status;
        kinematics_status = tracik_solver.CartToJnt(nominal,cartpos,jointpositions);

        if(kinematics_status)
        {
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(nj);
            joint_state.position.resize(nj);
            for(int i=0;i<nj;i++)
            {
                std::stringstream ss;
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
            if(kdl_cartpos6.p.z()<-0.035&&kdl_cartpos5.p.z()<-0.035&&kdl_cartpos4.p.z()<-0.035&&kdl_cartpos3.p.z()<-0.035&&kdl_cartpos2.p.z()<-0.035)
            {
                joint_state_pub.publish(joint_state);
            }
        }
        else{std::cout<<"error!!!!!"<<std::endl;}

        std::cout<<kdl_cartpos1.p<<"_"<<kdl_cartpos2.p<<"_"<<kdl_cartpos3.p<<"_"<<kdl_cartpos4.p<<"_"<<kdl_cartpos5.p<<"_"<<kdl_cartpos6.p<<std::endl;
        ros::spinOnce();
        loop.sleep();
    }

}

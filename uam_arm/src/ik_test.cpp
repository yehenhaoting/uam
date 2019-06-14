//
// Created by zm on 19-6-10.
//

#include "ros/ros.h"
#include <trac_ik/trac_ik.hpp>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

using namespace KDL;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_test");
    ros::NodeHandle nh("~");



    //Trac_IK
    TRAC_IK::TRAC_IK ik_solver("base_link", "end_link", "/robot_description", 0.005, 1E-5, TRAC_IK::Speed); //TRAC_IK::SolveType  Distance

//    int num_samples;
//    std::string chain_start, chain_end, urdf_param;
//    double timeout;
//    const double error = 1e-5;
//
//    nh.param("chain_start", chain_start, std::string(""));
//    nh.param("chain_end", chain_end, std::string(""));
//
//    if (chain_start=="" || chain_end=="") {
//        ROS_FATAL("Missing chain info in launch file");
//        exit (-1);
//    }

//    nh.param("timeout", timeout, 0.005);
//    nh.param("urdf_param", urdf_param, std::string("/robot_description"));

//    if (num_samples < 1)
//        num_samples = 1;


//    TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, error, TRAC_IK::Speed);

    KDL::Chain chain;
    bool valid = ik_solver.getKDLChain(chain);

    if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
        return -1;
    }


    // Set up KDL IK
    KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver based on kinematic chain

    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    ROS_INFO ("Using %d joints",nj);
    KDL::JntArray jointpositions = JntArray(nj);

    // Assign some values to the joint positions
    for(unsigned int i=0;i<nj;i++){
        float myinput;
        printf ("Enter the position of joint %i: ",i);
        scanf ("%e",&myinput);
        jointpositions(i)=(double)myinput;
    }

    // Create the frame that will contain the results
    KDL::Frame cartpos;

    // Calculate forward position kinematics
    int kinematics_status;
    kinematics_status = fk_solver.JntToCart(jointpositions,cartpos);

    Vector p = cartpos.p;   // Origin of the Frame
    Rotation M = cartpos.M; // Orientation of the Frame

    double roll, pitch, yaw;
    M.GetRPY(roll,pitch,yaw);

    if(kinematics_status>=0){
        printf("%s \n","KDL FK Succes");
        std::cout <<"Origin: " << p(0) << "," << p(1) << "," << p(2) << std::endl;
        std::cout <<"RPY: " << roll << "," << pitch << "," << yaw << std::endl;

    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }


    KDL::JntArray joint_seed(nj);
    KDL::SetToZero(joint_seed);
    KDL::JntArray result(joint_seed);

    int rc=ik_solver.CartToJnt(joint_seed,cartpos,result);
    if(rc < 0)
        printf("%s \n","Error: could not calculate forward kinematics :(");
    else{
        printf("%s \n","TRAC IK Succes");
        for(unsigned int i = 0; i < nj; i++)
            std::cout << result(i) << " ";
    }

    return 0;
}
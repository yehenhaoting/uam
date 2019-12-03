#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/PoseStamped.h> 
#include <geometry_msgs/TwistStamped.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

#include <sstream>
#include <string.h>
#include "CFetchViconData.h"

#include <dynamic_reconfigure/server.h>
#include <viconros/init_Config.h>


KDL::Frame F_world_uav, F_world_segment, F_uav_segment;   //无人机FRD坐标系在vicon坐标系下表达，segment在vicon坐标系下表达，segment在无人机FRD坐标系下表达

viconros::init_Config correct_pose;
/**
 * 参数读取callback函数，用于获取飞行控制器的参数
 * @param config
 */
void param_cb(const viconros::init_Config &config)
{
    correct_pose = config;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "viconros");
	ros::NodeHandle n("~");
	std::string ip;
	std::string model;
	std::string segment;


	n.getParam("host",ip);
	n.getParam("model",model);
	n.getParam("segment",segment);
	ROS_INFO("HOST:%s",ip.c_str());
	ROS_INFO("MODEL:%s; SEGMENT:%s",model.c_str(),segment.c_str());

	ros::Publisher vicon_pub = n.advertise<geometry_msgs::PoseStamped> ("/uam_base/pose", 10);
	ros::Publisher vicon_pub1 = n.advertise<geometry_msgs::TwistStamped> ("/mocap/vel", 10);

    // 通过参数服务器的方式，获取控制器的参数
    dynamic_reconfigure::Server<viconros::init_Config> server;
    dynamic_reconfigure::Server<viconros::init_Config>::CallbackType ff;
    ff = boost::bind(&param_cb, _1);
    server.setCallback(ff);


	ros::Rate loop_rate(30);
	int count = 0;
	CFetchViconData * vicon=new CFetchViconData();
	const char * host=ip.c_str();
	ObjStatus objs;
	
	if(!(vicon->IsConnected))
    { 
            ROS_INFO("Connecting to %s",host);
            bool res=vicon->Connect(host);
            if(res==false)
            {
                ROS_INFO("Failed to connect!\r\n");
                    return 0;
            }
            else
            {
                ROS_INFO("Successfully connected!\r\n");
            }

    }
	while (ros::ok()) {
		geometry_msgs::PoseStamped msg;
		geometry_msgs::TwistStamped msg1;

//         objs=vicon->GetStatus(model.c_str(),segment.c_str());
        vicon->GetStatus(objs, model.c_str(),segment.c_str());

//		msg.header.stamp.sec=(int)objs.tm;
//		msg.header.stamp.nsec=(objs.tm-msg.header.stamp.sec)*10000*100000;
//		msg.pose.position.x =objs.pos[0];
//		msg.pose.position.y =objs.pos[1];
//		msg.pose.position.z =objs.pos[2];
//
//        msg.pose.orientation.x =objs.ort[0];
//        msg.pose.orientation.y =objs.ort[1];
//        msg.pose.orientation.z =objs.ort[2];
//        msg.pose.orientation.w =objs.ort[3];
        KDL::Rotation ORT = KDL::Rotation::Quaternion(0,0,0,0);

        F_world_segment = KDL::Frame(KDL::Rotation::Quaternion(objs.ort[0], objs.ort[1], objs.ort[2], objs.ort[3]),KDL::Vector(objs.pos[0], objs.pos[1], objs.pos[2]));
        if(correct_pose.Correct_start){
            ROS_INFO_STREAM("Correcting the pose bias");
            F_uav_segment = KDL::Frame(KDL::Rotation::Quaternion(objs.ort[0], objs.ort[1], objs.ort[2], objs.ort[3]), KDL::Vector(correct_pose.EV_POS_X, correct_pose.EV_POS_Y, correct_pose.EV_POS_Z));
        }
        F_world_uav = F_world_segment * F_uav_segment.Inverse();

        msg.header.stamp.sec=(int)objs.tm;
        msg.header.stamp.nsec=(objs.tm-msg.header.stamp.sec)*10000*100000;
        msg.pose.position.x =F_world_uav.p.data[0];
        msg.pose.position.y =F_world_uav.p.data[1];
        msg.pose.position.z =F_world_uav.p.data[2];

        F_world_uav.M.GetQuaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);


//        msg.pose.orientation.x =objs.euler[0];
//        msg.pose.orientation.y =objs.euler[1];
//        msg.pose.orientation.z =objs.euler[2];
//        msg.pose.orientation.w =0;

//		msg1.header.stamp.sec=(int)objs.tm;
//		msg1.header.stamp.nsec=(objs.tm-msg.header.stamp.sec)*10000*100000;
//		msg1.twist.linear.x =objs.vel[0];
//		msg1.twist.linear.y =objs.vel[1];
//		msg1.twist.linear.z =objs.vel[2];

//		std::cout<<"current_POS"<<"\t"<<objs.pos[0]<<"\t"<<objs.pos[1]<<"\t"<<objs.pos[2]<<std::endl;
//		std::cout<<"current_ort"<<"\tx:"<<objs.ort[0]<<"\ty:"<<objs.ort[1]<<"\tz:"<<objs.ort[2]<<"\tw:"<<objs.ort[3]<<std::endl;

		
		//std::cout<<objs.pos[0]<<"-"<<objs.pos[1]<<std::endl;
		//ROS_INFO("position:%f-%f-%f; velocity: %f-%f-%f", msg.position.x,msg.position.y,msg.position.z,msg.velocity.x,msg.velocity.y,msg.velocity.z);
		
		
		vicon_pub.publish(msg);
//		vicon_pub1.publish(msg1);

		ros::spinOnce();
		loop_rate.sleep();
		//++count;
	}
	return 0;
}

//
// Created by zm on 19-6-19.
//

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

//tf::TransformBroadcaster br;
tf::Transform transform;
geometry_msgs::PoseStamped pose_drone;

void uavpose_cb(const geometry_msgs::PoseStamped& msg)
{
    transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) );
    transform.setRotation( tf::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w) );
//
//    pose_drone.header.stamp = ros::Time::now();
//    pose_drone.pose.position.x = msg.pose.position.x;
//    pose_drone.pose.position.y = msg.pose.position.y;
//    pose_drone.pose.position.z = msg.pose.position.z;
//    pose_drone.pose.orientation.x = msg.pose.orientation.x;
//    pose_drone.pose.orientation.y = msg.pose.orientation.y;
//    pose_drone.pose.orientation.z = msg.pose.orientation.z;
//    pose_drone.pose.orientation.w = msg.pose.orientation.w;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "drone_tf_broadcaster");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
//    tf::Transform transform;

//    ros::Subscriber pos_sub = node.subscribe("/uav/uav_pose", 10, uavpose_cb);
//    ros::Subscriber pos_sub = node.subscribe("/uam/base_pose", 10, uavpose_cb);
      ros::Subscriber pos_sub = node.subscribe("/mavros/local_position/pose", 10, uavpose_cb);

//    ros::Publisher base_pub = node.advertise< geometry_msgs::PoseStamped >("arm/uav_pose", 10);


    ros::Rate rate(20.0);
    while (node.ok()){
        ros::spinOnce();
//        transform.setOrigin( tf::Vector3(-0.05 + 0.05*sin(ros::Time::now().toSec()), 0.05*cos(ros::Time::now().toSec()), 0.05) );
//        transform.setRotation( tf::Quaternion(0, 0, 0, 1) );



//        pose_drone.header.stamp = ros::Time::now();

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "uav_link")); //transform是坐标变换关系，“world”是父Frame，“uav_Link“是子frame
//        base_pub.publish(pose_drone);
        rate.sleep();
    }
    return 0;
}
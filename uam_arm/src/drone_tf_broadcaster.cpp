//
// Created by zm on 19-6-19.
//

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "drone_tf_broadcaster");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::Transform transform;
    geometry_msgs::PoseStamped pose_drone;


    ros::Publisher base_pub = node.advertise< geometry_msgs::PoseStamped >("arm/uav_pose", 10);


    ros::Rate rate(100.0);
    while (node.ok()){
        transform.setOrigin( tf::Vector3(-0.05 + 0.05*sin(ros::Time::now().toSec()), 0.05*cos(ros::Time::now().toSec()), 0.05) );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1) );



        pose_drone.header.stamp = ros::Time::now();
        pose_drone.pose.position.x = -0.05 + 0.05*sin(ros::Time::now().toSec());
        pose_drone.pose.position.y = 0.05*cos(ros::Time::now().toSec());
        pose_drone.pose.position.z = 0.05;
        pose_drone.pose.orientation.x = 0;
        pose_drone.pose.orientation.y = 0;
        pose_drone.pose.orientation.z = 0;
        pose_drone.pose.orientation.w = 1;

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "uav_link"));
        base_pub.publish(pose_drone);
        rate.sleep();
    }
    return 0;
}
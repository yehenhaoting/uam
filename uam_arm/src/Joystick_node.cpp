//
// Created by zm on 19-6-12.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>

int linear_X, linear_Y, linear_Z, angular_R, angular_P, angular_Y, mode_XYZ, mode_RPY, mode_reset;
double l_scale_X, l_scale_Y, l_scale_Z, a_scale_R, a_scale_P, a_scale_Y;

geometry_msgs::Twist vel_pub;
geometry_msgs::Pose pose_pub, pose_zero;

void joy_init(ros::NodeHandle & nh_)
{
    nh_.param("axis_linear_X", linear_X, 1);
    nh_.param("axis_linear_Y", linear_Y, 0);
    nh_.param("axis_linear_Z", linear_Z, 4);
    nh_.param("axis_angular_R", angular_R, 0);
    nh_.param("axis_angular_P", angular_P, 1);
    nh_.param("axis_angular_Y", angular_Y, 3);
    nh_.param("buttons_mode_XYZ", mode_XYZ, 4);
    nh_.param("buttons_mode_RPY", mode_RPY, 5);
    nh_.param("buttons_reset", mode_reset, 0);

    nh_.param("scale_linear_X", l_scale_X, 1.0);
    nh_.param("scale_linear_Y", l_scale_Y, 1.0);
    nh_.param("scale_linear_Z", l_scale_Z, 1.0);
    nh_.param("scale_angular_R", a_scale_R, 1.0);
    nh_.param("scale_angular_P", a_scale_P, 1.0);
    nh_.param("scale_angular_Y", a_scale_Y, 1.0);
}

void pose_init(ros::NodeHandle & nh_)
{
    pose_zero.position.x = 0.18;
    pose_zero.position.y = 0.00;
    pose_zero.position.z = -0.20;
    pose_zero.orientation.x = 0.0;
    pose_zero.orientation.y = 0.0;
    pose_zero.orientation.z = 0.0;
    pose_zero.orientation.w = 0.0;
    pose_pub = pose_zero;
}

void pose_Integral()
{
    pose_pub.position.x += vel_pub.linear.x/1000;
    pose_pub.position.y += vel_pub.linear.y/1000;
    pose_pub.position.z += vel_pub.linear.z/1000;
    pose_pub.orientation.x += vel_pub.angular.x/1000;
    pose_pub.orientation.y += vel_pub.angular.y/1000;
    pose_pub.orientation.z += vel_pub.angular.z/1000;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(joy->buttons[mode_XYZ] && !joy->buttons[mode_RPY])
    {
        vel_pub.linear.x = l_scale_X*joy->axes[linear_X];
        vel_pub.linear.y = l_scale_Y*joy->axes[linear_Y];
        vel_pub.linear.z = l_scale_Z*joy->axes[linear_Z];
    }

    if(joy->buttons[mode_RPY] && !joy->buttons[mode_XYZ])
    {
        vel_pub.angular.x = a_scale_R*joy->axes[angular_R];
        vel_pub.angular.y = a_scale_P*joy->axes[angular_P];
        vel_pub.angular.z = a_scale_Y*joy->axes[angular_Y];
    }
    if(!joy->buttons[mode_XYZ] && !joy->buttons[mode_RPY])
    {
        vel_pub.linear.x = 0;
        vel_pub.linear.y = 0;
        vel_pub.linear.z = 0;
        vel_pub.angular.x = 0;
        vel_pub.angular.y = 0;
        vel_pub.angular.z = 0;
    }
    if(joy->buttons[mode_reset])
    {
        pose_pub = pose_zero;
    }
    pose_pub.orientation.w = joy->buttons[mode_reset];

}



int main(int argc, char** argv)
{

    ros::init(argc, argv, "teleop_arm");
    ros::NodeHandle nh;
    joy_init(nh);
    pose_init(nh);

    ros::Subscriber joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &joyCallback);
    ros::Publisher vel_pub_  = nh.advertise<geometry_msgs::Twist>("arm/cmd_vel", 1000);
    ros::Publisher pose_pub_ = nh.advertise<geometry_msgs::Pose>("arm/cmd_pose", 1);

    ros::Rate loop_rate(100);


    while(ros::ok())
    {
        ros::spinOnce();
        vel_pub_.publish(vel_pub);

        pose_Integral();
        pose_pub_.publish(pose_pub);

        loop_rate.sleep();
    }
    return 0;
}

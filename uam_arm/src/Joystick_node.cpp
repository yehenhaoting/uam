//
// Created by zm on 19-6-12.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>

int linear_X, linear_Y, linear_Z, angular_R, angular_P, angular_Y, mode_XYZ, mode_RPY, mode_reset, mode_start, mode_absolute;
double l_scale_X, l_scale_Y, l_scale_Z, a_scale_R, a_scale_P, a_scale_Y;
unsigned int modeFlags;
int EE_LeftRight, EE_UpDown; //对于末端软体抓手的继电器控制


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
    nh_.param("buttons_start", mode_start, 1);
    nh_.param("buttons_absolute", mode_absolute, 2);

    nh_.param("scale_linear_X", l_scale_X, 1.0);
    nh_.param("scale_linear_Y", l_scale_Y, 1.0);
    nh_.param("scale_linear_Z", l_scale_Z, 1.0);
    nh_.param("scale_angular_R", a_scale_R, 1.0);
    nh_.param("scale_angular_P", a_scale_P, 1.0);
    nh_.param("scale_angular_Y", a_scale_Y, 1.0);

    nh_.param("axis_Left_Right", EE_LeftRight, 6);
    nh_.param("axis_Up_Down", EE_UpDown, 7);
}

/**
 * 设定初始位置，用于机械臂无法求解时的reset位姿
 */
void pose_init(ros::NodeHandle & nh_)
{
    pose_zero.position.x = 0.07;
    pose_zero.position.y = 0.00;
    pose_zero.position.z = -0.275;
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

/**
 * 通过一个8位的数，传递控制机械臂的模式信息
 * @param index
 * @param set
 */
void mode_encoder(uint8_t index, bool set = true)
{
    if (index > 32) return;
    if (set) {
        modeFlags |= 1u << index;
    } else {
        modeFlags &= ~(1u << index);
    }
}

void mode_Clean(){
    modeFlags = 0;
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
        mode_encoder(0, true);
        pose_pub = pose_zero;
    }
    if(joy->buttons[mode_start]) mode_encoder(1, true);
    if(joy->buttons[mode_absolute]) mode_encoder(2, true);   
    if(joy->axes[EE_LeftRight] >  0.5) mode_encoder(3, true);
    if(joy->axes[EE_LeftRight] < -0.5) mode_encoder(4, true);
    if(joy->axes[EE_UpDown] < -0.5)    mode_encoder(5, true);
    if(joy->axes[EE_UpDown] >  0.5)    mode_encoder(6, true);
    // std::cout<<joy->buttons[mode_start]<<"\t"<<joy->axes[EE_LeftRight]<<std::endl;

    pose_pub.orientation.w = modeFlags;
//    pose_pub.orientation.w = joy->buttons[mode_reset];

}



int main(int argc, char** argv)
{

    ros::init(argc, argv, "uam_joystick");
    ros::NodeHandle nh;
    joy_init(nh);
    pose_init(nh);

    ros::Subscriber joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &joyCallback);
    ros::Publisher vel_pub_  = nh.advertise<geometry_msgs::Twist>("arm/cmd_vel", 10);
    ros::Publisher pose_pub_ = nh.advertise<geometry_msgs::Pose>("arm/cmd_pose", 10);

    ros::Rate loop_rate(100);


    while(ros::ok())
    {
        ros::spinOnce();
        vel_pub_.publish(vel_pub);

        pose_Integral();
        pose_pub_.publish(pose_pub);
        mode_Clean();

        loop_rate.sleep();
//        std::cout<<"modeFlags:"<<modeFlags<<std::endl;

    }
    return 0;
}

//
// Created by zm on 19-6-12.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>


class TeleopArm
{
public:
    TeleopArm();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

//    int linear_, angular_;
//    double l_scale_, a_scale_;
//    ros::Publisher vel_pub_;
//    ros::Subscriber joy_sub_;

    int linear_X, linear_Y, linear_Z, angular_R, angular_P, angular_Y, mode_XYZ, mode_RPY;
    double l_scale_X, l_scale_Y, l_scale_Z, a_scale_R, a_scale_P, a_scale_Y, m_scale_XYZ, m_scale_RPY;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;

};


TeleopArm::TeleopArm():
//        linear_(1),
//        angular_(2)
        linear_X(1), linear_Y(0), linear_Z(4),
        angular_R(3), angular_P(4), angular_Y(1),
        mode_XYZ(4), mode_RPY(5)
{
    nh_.param("axis_linear_X", linear_X, linear_X);
    nh_.param("axis_linear_Y", linear_Y, linear_Y);
    nh_.param("axis_linear_Z", linear_Z, linear_Z);
    nh_.param("axis_angular_R", angular_R, angular_R);
    nh_.param("axis_angular_P", angular_P, angular_P);
    nh_.param("axis_angular_Y", angular_Y, angular_Y);
    nh_.param("buttons_mode_XYZ", mode_XYZ, mode_XYZ);
    nh_.param("buttons_mode_RPY", mode_RPY, mode_RPY);


    nh_.param("scale_linear_X", l_scale_X, l_scale_X);
    nh_.param("scale_linear_Y", l_scale_Y, l_scale_Y);
    nh_.param("scale_linear_Z", l_scale_Z, l_scale_Z);
    nh_.param("scale_angular_R", a_scale_R, a_scale_R);
    nh_.param("scale_angular_P", a_scale_P, a_scale_P);
    nh_.param("scale_angular_Y", a_scale_Y, a_scale_Y);
//    nh_.param("scale_mode_XYZ", m_scale_XYZ, m_scale_XYZ);
//    nh_.param("scale_mode_RPY", m_scale_RPY, m_scale_RPY);


//    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    vel_pub_ = nh_.advertise<geometry_msgs::Pose>("arm/cmd_vel", 1);


    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopArm::joyCallback, this);

}

void TeleopArm::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;

//    if(joy->buttons[mode_XYZ] && !joy->buttons[mode_RPY])
    if(true)
    {
        twist.linear.x = l_scale_X*joy->axes[linear_X];
        twist.linear.y = l_scale_Y*joy->axes[linear_Y];
        twist.linear.z = l_scale_Z*joy->axes[linear_Z];
    }

//    if(joy->buttons[mode_RPY] && !joy->buttons[mode_XYZ])
    if(true)
    {
        twist.angular.x = a_scale_R*joy->axes[angular_R];
        twist.angular.y = a_scale_P*joy->axes[angular_P];
        twist.angular.z = a_scale_Y*joy->axes[angular_Y];
    }
    vel_pub_.publish(twist);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_arm");

        TeleopArm teleop_arm;
        ros::spin();

}

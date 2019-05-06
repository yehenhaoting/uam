
#include "ftServo.h"
#include <string>

using namespace std;
int servo_position1=3109;
int servo_position2=2521;
int servo_position3=92;
int servo_position4=868;
int servo_position5=411;
int servo_position6=470;
int servo_position7=100;
double x=0.12,y=0,z=-0.2;
void chatterCallback(const sensor_msgs::JointState& msg)
{
    servo_position1=(msg.position[0]+1.5707963)/3.1415926*2047+2047;
    servo_position2=(msg.position[1]-0.32)/3.1415926*2047+2047;
    servo_position3=(msg.position[2]+0.4707963)/2.618*511+511;
    servo_position4=(-msg.position[3]+0.02)/2.618*511+511;
    servo_position5=(-msg.position[4]-0.45)/2.618*511+511;
    servo_position6=(-msg.position[5]-0.12)/2.618*511+511;
    cout<<servo_position1<<"-"<<servo_position2<<"-"<<servo_position3<<"-"<<servo_position4<<"-"<<servo_position5<<"-"<<servo_position6<<"-"<<servo_position7<<endl;
}
void chatterCallback2(const geometry_msgs::Quaternion& msg)
{
  servo_position7=msg.w;
}
int main(int argc, char** argv)
{
    string device = "/dev/ttyUSB0";
    int serial_fd = open(device.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd == -1)
    {
        ROS_INFO("Open Error!");
        exit(1);
    }
    int ret = set_serial(serial_fd, BAUD, 8, 'N', 1);
    if (ret == -1)
    {
        ROS_INFO("Set Serial Error!");
        exit(1);
    }
    ROS_INFO("ttyUSB connect success!");

    unsigned char cmdBuffer1[20] = {0xff, 0xff, 0x01, 0x09, 0x03, 0x2a, 0x64, 0x00, 0x2c, 0x01, 0x00, 0x00, 0x00};
    unsigned char cmdBuffer2[20] = {0xff, 0xff, 0x02, 0x09, 0x03, 0x2a, 0x64, 0x00, 0x2c, 0x01, 0x00, 0x00, 0xff};
    unsigned char cmdBuffer3[20] = {0xff, 0xff, 0x03, 0x09, 0x03, 0x2a, 0x00, 0x64, 0x01, 0x2c, 0x00, 0x00, 0xff};
    unsigned char cmdBuffer4[20] = {0xff, 0xff, 0x04, 0x09, 0x03, 0x2a, 0x00, 0x64, 0x01, 0x2c, 0x00, 0x00, 0xff};
    unsigned char cmdBuffer5[20] = {0xff, 0xff, 0x05, 0x09, 0x03, 0x2a, 0x00, 0x64, 0x01, 0x2c, 0x00, 0x00, 0xff};
    unsigned char cmdBuffer6[20] = {0xff, 0xff, 0x06, 0x09, 0x03, 0x2a, 0x00, 0x64, 0x01, 0x2c, 0x00, 0x00, 0xff};
    unsigned char cmdBuffer7[20] = {0xff, 0xff, 0x07, 0x09, 0x03, 0x2a, 0x00, 0x64, 0x01, 0x2c, 0x00, 0x00, 0xff};
    //unsigned char cmdBuffer[30] = {0xff, 0xff, 0x04, 0x09, 0x03, 0x2a, 0x00, 0x64, 0x00, 0x64, 0x00, 0x00, 0xfd,0xff, 0xff, 0x05, 0x09, 0x03, 0x2a, 0x00, 0x64, 0x00, 0x64, 0x00, 0x00, 0xfc};
    ros::init(argc, argv, "ftServo");
    ros::NodeHandle n;
    ros::Subscriber joint_state_sub = n.subscribe("/joint_states", 1, chatterCallback);
    ros::Subscriber position_sub = n.subscribe("position", 1, chatterCallback2);
    ros::Rate loop (50);
    while (ros::ok())
    {
        cmdBuffer1[7]=(servo_position1&0xFF00)>>8;
        cmdBuffer1[6]=servo_position1&0x00FF;
        cmdBuffer1[12]=((cmdBuffer1[2]+cmdBuffer1[3]+cmdBuffer1[4]+cmdBuffer1[5]+cmdBuffer1[6]+cmdBuffer1[7]+cmdBuffer1[8]+cmdBuffer1[9]+cmdBuffer1[10]+cmdBuffer1[11])&255)^255;
        ret = static_cast<int>(write(serial_fd, cmdBuffer1, 13));
        usleep(5000);
        cmdBuffer2[7]=(servo_position2&0xFF00)>>8;
        cmdBuffer2[6]=servo_position2&0x00FF;
        cmdBuffer2[12]=((cmdBuffer2[2]+cmdBuffer2[3]+cmdBuffer2[4]+cmdBuffer2[5]+cmdBuffer2[6]+cmdBuffer2[7]+cmdBuffer2[8]+cmdBuffer2[9]+cmdBuffer2[10]+cmdBuffer2[11])&255)^255;
        ret = static_cast<int>(write(serial_fd, cmdBuffer2, 13));
        usleep(5000);
        cmdBuffer3[6]=(servo_position3&0xFF00)>>8;
        cmdBuffer3[7]=servo_position3&0x00FF;
        cmdBuffer3[12]=((cmdBuffer3[2]+cmdBuffer3[3]+cmdBuffer3[4]+cmdBuffer3[5]+cmdBuffer3[6]+cmdBuffer3[7]+cmdBuffer3[8]+cmdBuffer3[9]+cmdBuffer3[10]+cmdBuffer3[11])&255)^255;
        ret = static_cast<int>(write(serial_fd, cmdBuffer3, 13));
        usleep(5000);
        cmdBuffer4[6]=(servo_position4&0xFF00)>>8;
        cmdBuffer4[7]=servo_position4&0x00FF;
        cmdBuffer4[12]=((cmdBuffer4[2]+cmdBuffer4[3]+cmdBuffer4[4]+cmdBuffer4[5]+cmdBuffer4[6]+cmdBuffer4[7]+cmdBuffer4[8]+cmdBuffer4[9]+cmdBuffer4[10]+cmdBuffer4[11])&255)^255;
        ret = static_cast<int>(write(serial_fd, cmdBuffer4, 13));
        usleep(5000);
        cmdBuffer5[6]=(servo_position5&0xFF00)>>8;
        cmdBuffer5[7]=servo_position5&0x00FF;
        cmdBuffer5[12]=((cmdBuffer5[2]+cmdBuffer5[3]+cmdBuffer5[4]+cmdBuffer5[5]+cmdBuffer5[6]+cmdBuffer5[7]+cmdBuffer5[8]+cmdBuffer5[9]+cmdBuffer5[10]+cmdBuffer5[11])&255)^255;
        ret = static_cast<int>(write(serial_fd, cmdBuffer5, 13));
        usleep(5000);
        cmdBuffer6[6]=(servo_position6&0xFF00)>>8;
        cmdBuffer6[7]=servo_position6&0x00FF;
        cmdBuffer6[12]=((cmdBuffer6[2]+cmdBuffer6[3]+cmdBuffer6[4]+cmdBuffer6[5]+cmdBuffer6[6]+cmdBuffer6[7]+cmdBuffer6[8]+cmdBuffer6[9]+cmdBuffer6[10]+cmdBuffer6[11])&255)^255;
        ret = static_cast<int>(write(serial_fd, cmdBuffer6, 13));
        usleep(5000);
        if(servo_position7%2!=0)
        {
            cmdBuffer7[6]=(800&0xFF00)>>8;
            cmdBuffer7[7]=800&0x00FF;
            cmdBuffer7[12]=((cmdBuffer7[2]+cmdBuffer7[3]+cmdBuffer7[4]+cmdBuffer7[5]+cmdBuffer7[6]+cmdBuffer7[7]+cmdBuffer7[8]+cmdBuffer7[9]+cmdBuffer7[10]+cmdBuffer7[11])&255)^255;
            ret = static_cast<int>(write(serial_fd, cmdBuffer7, 13));
        }
        else
        {
            cmdBuffer7[6]=(940&0xFF00)>>8;
            cmdBuffer7[7]=940&0x00FF;
            cmdBuffer7[12]=((cmdBuffer7[2]+cmdBuffer7[3]+cmdBuffer7[4]+cmdBuffer7[5]+cmdBuffer7[6]+cmdBuffer7[7]+cmdBuffer7[8]+cmdBuffer7[9]+cmdBuffer7[10]+cmdBuffer7[11])&255)^255;
            ret = static_cast<int>(write(serial_fd, cmdBuffer7, 13));
        }
        ros::spinOnce();
        loop.sleep();
    }

    close(serial_fd);
}

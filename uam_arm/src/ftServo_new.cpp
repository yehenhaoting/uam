#include "ftServo.h"
#include <string>
#include <geometry_msgs/Pose.h>
#include "sensor_msgs/JointState.h"

#define COMMANDLEN 13
#define PI 3.1415926535898

using namespace std;

void EE_decoder(int mode_index);
unsigned char Checksum(unsigned char *argc);
void SetPosition(unsigned char *argc, int id, int pos);

int ServoPosition[] = {3109, 2521, 92, 868, 411, 470, 100};
int ServoSpeed_Init[] = {0, 0, 0, 1023, 0, 1023, 0, 0};
int EndEffector_mode = 0;

unsigned char BaseCommand[] = { 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


void jointposition_cb(const sensor_msgs::JointState& msg)
{
    ServoPosition[0]=(msg.position[0]+1.5072)/PI*2047+2047;
    ServoPosition[1]=(-msg.position[1]-0.4396)/PI*2047+2047;
    ServoPosition[2]=(msg.position[2]+0.5652)/2.618*511+511;
//    ServoPosition[3]=(-msg.position[3]+0.02)/2.618*511+511;
    ServoPosition[3]=(-msg.position[3]-0.2512)/PI*2048 + 4096;   //按照正负1圈来运行 2PI=360
    ServoPosition[4]=(msg.position[4]-0.5024)/2.618*511+511;
//    ServoPosition[5]=(-msg.position[5]-0.12)/2.618*511+511;
    ServoPosition[5]=(-msg.position[5]+0.0628)/PI*2048 + 4096;
    ServoPosition[6]=EndEffector_mode;

    for(int i=0;i<7;i++)
        cout << ServoPosition[i] << "-";
    cout << endl;
}
void EndEffector_cb(const geometry_msgs::Pose &pose)
{
    EE_decoder(pose.orientation.w);   //运行模式解析
}


int main(int argc, char** argv)
{
    string device = "/dev/ftservo";
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

    ros::init(argc, argv, "ftServo");
    ros::NodeHandle n;

    ros::Subscriber joint_state_sub = n.subscribe("/joint_states", 1, jointposition_cb);
    ros::Subscriber EE_pose_sub = n.subscribe("/arm/cmd_pose", 10, EndEffector_cb);

    ros::Rate loop (50);

    while (ros::ok())
    {
        for(int i=0; i<7; i++)
        {
            SetPosition(BaseCommand, i, ServoPosition[i]);
            ret = static_cast<int>(write(serial_fd, BaseCommand, COMMANDLEN));
            usleep(5000);
        }
        ros::spinOnce();
        loop.sleep();
    }

    close(serial_fd);
}



void EE_decoder(int mode_index){
//    std::cout<<"mode_index:"<<mode_index<<std::endl;
    //模式判断部分
    switch (mode_index)
    {
        case (8):{    //按下左键，软体手张开
            EndEffector_mode = 0;
            break;
        }
        case (16):{    //按下右键，软体手闭合
            EndEffector_mode = 1;
            break;
        }
        case (32):{    //按下上键，阀开，继电器全部工作
            EndEffector_mode = 2;
            break;
        }
        case (64):{    //按下下键，阀关，继电器不工作
            EndEffector_mode = 3;
            break;
        }
        default:{
            EndEffector_mode = 3; //需要长按才能实现以上功能
            break;
        }
    }
}


unsigned char Checksum(unsigned char *argc)
{
    unsigned char sum = 0;
    for (int i = 2; i < COMMANDLEN - 1; i++)
        sum += argc[i];
    sum = (sum & 0xFF) ^ 0xFF;

    return sum;
}

void SetPosition(unsigned char *argc, int id, int pos)
{
    argc[2] = id+1;
    argc[3] = 0x09;
    argc[4] = 0x03;
    argc[5] = 0x2A;

    if(id == 0 || id == 1 || id == 3 || id == 5){
        // pose
        argc[6] = pos & 0x00FF;
        argc[7] = pos >> 8;

        argc[8] = 0x00;
        argc[9] = 0x00;

        // speed
        int temp_speed = ServoSpeed_Init[id];
        argc[10] = temp_speed & 0x00FF;
        argc[11] = temp_speed >> 8;
    } else{

        // pose
        argc[7] = pos & 0x00FF;
        argc[6] = pos >> 8;

        argc[8] = 0x00;
        argc[9] = 0x00;

        // speed
        int temp_speed = ServoSpeed_Init[id];
        argc[11] = temp_speed & 0x00FF;
        argc[10] = temp_speed >> 8;
    }

    argc[12] = Checksum(argc);
//    if(id == 3){
//        for(int nn=0;nn<13;nn++)
//            cout<<hex<<(unsigned int)argc[nn]<< "-";
//        cout<<endl;
//    }
}
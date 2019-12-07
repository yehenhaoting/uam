#include "ftServo.h"
#include <string>

#define COMMANDLEN 13
#define PI 3.1415926535898

using namespace std;

int ServoPosition[] = {3109, 2521, 92, 868, 411, 470, 100};
int ServoSpeed_Init[] = {0, 0, 0, 1023, 0, 1023, 0, 0};

unsigned char BaseCommand[] = { 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

void jointposition_cb(const sensor_msgs::JointState& msg)
{
    ServoPosition[0]=(msg.position[0]+1.5707963)/3.1415926*2047+2047;
    ServoPosition[1]=(-msg.position[1]-0.32)/3.1415926*2047+2047;
    ServoPosition[2]=(msg.position[2]+0.4707963)/2.618*511+511;
//    ServoPosition[3]=(-msg.position[3]+0.02)/2.618*511+511;
    ServoPosition[3]=(-msg.position[3]+0.02)/PI*4096 + 3*4096;   //按照正负3圈来运行
    ServoPosition[4]=(msg.position[4]-0.45)/2.618*511+511;
//    ServoPosition[5]=(-msg.position[5]-0.12)/2.618*511+511;
    ServoPosition[5]=(-msg.position[5]-0.12)/PI*4096 + 3*4096;
    ServoPosition[6]=(int)msg.position[6];

    for(int i=0;i<7;i++)
        cout << ServoPosition[i] << "-";
    cout << endl;
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

    ros::init(argc, argv, "ftServo");
    ros::NodeHandle n;
    ros::Subscriber joint_state_sub = n.subscribe("/joint_states", 1, jointposition_cb);
    ros::Rate loop (50);

    while (ros::ok())
    {
        for(int i=0; i<7; i++)
        {
            SetPosition(BaseCommand, i, ServoPosition[i]);
            ret = static_cast<int>(write(serial_fd, BaseCommand, COMMANDLEN));
            usleep(5000);
        }

        ret = static_cast<int>(write(serial_fd, BaseCommand, COMMANDLEN));
            usleep(5000);

        ros::spinOnce();
        loop.sleep();
    }

    close(serial_fd);
}

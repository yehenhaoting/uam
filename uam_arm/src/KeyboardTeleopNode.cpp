#include <termios.h>  
#include <signal.h>  
#include <math.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <sys/poll.h>  
  
#include <boost/thread/thread.hpp>  
#include <ros/ros.h>  
#include <geometry_msgs/Twist.h>  
#include <geometry_msgs/Quaternion.h>

#define KEYCODE_W 0x77  
#define KEYCODE_A 0x61  
#define KEYCODE_S 0x73  
#define KEYCODE_D 0x64  
#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53  
#define KEYCODE_W_CAP 0x57  
  

class SmartCarKeyboardTeleopNode  
{  
    private:  
        double x;  
        double y;
        double z;  
        double w;

        geometry_msgs::Quaternion position_; 
        ros::NodeHandle n_;  
        ros::Publisher pub_;  
  
    public:  
        SmartCarKeyboardTeleopNode()  
        {  
            x=0.12;y=0;z=-0.2;w=100;
            pub_ = n_.advertise<geometry_msgs::Quaternion>("position", 1);  
              
        }  
          
        ~SmartCarKeyboardTeleopNode() { }  
        void keyboardLoop();  
        void IncreaseX()
        {
            x=x+0.004;
        } 
        void IncreaseY()
        {
            y=y+0.004;
        }
        void IncreaseZ()
        {
            z=z+0.004;
        } 
        void IncreaseW()
        {
            w=w+1;
        } 
        void DecreaseX()
        {
            x=x-0.004;
        } 
        void DecreaseY()
        {
            y=y-0.004;
        } 
        void DecreaseZ()
        {
            z=z-0.004;
        } 
        void DecreaseW()
        {
            w=w-1;
        } 
};  
  
SmartCarKeyboardTeleopNode* tbk;  
int kfd = 0;  
struct termios cooked, raw;  
bool done;  
  
int main(int argc, char** argv)  
{  
    ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);  
    SmartCarKeyboardTeleopNode tbk;  
      
    boost::thread t = boost::thread(boost::bind(&SmartCarKeyboardTeleopNode::keyboardLoop, &tbk));  
      
    ros::spin();  
      
    t.interrupt();  
    t.join();  
    tcsetattr(kfd, TCSANOW, &cooked);  
      
    return 0;
}  
  
void SmartCarKeyboardTeleopNode::keyboardLoop()  
{  
    char c;   
      
    // get the console in raw mode  
    tcgetattr(kfd, &cooked);  
    memcpy(&raw, &cooked, sizeof(struct termios));  
    raw.c_lflag &=~ (ICANON | ECHO);  
    raw.c_cc[VEOL] = 1;  
    raw.c_cc[VEOF] = 2;  
    tcsetattr(kfd, TCSANOW, &raw);  
      
    puts("Reading from keyboard");  
    puts("Use WASD keys to control the robot");  
    puts("Press Shift+W/S to move up/down");  
      
    struct pollfd ufd;  
    ufd.fd = kfd;  
    ufd.events = POLLIN;  
      
    for(;;)  
    {  
        boost::this_thread::interruption_point();  
          
        // get the next event from the keyboard  
        int num;  
          
        if ((num = poll(&ufd, 1, 250)) < 0)  
        {  
            perror("poll():");  
            return;  
        }  
        else if(num > 0)  
        {  
            if(read(kfd, &c, 1) < 0)  
            {  
                perror("read():");  
                return;  
            }  
        }  
        else{continue;}
          
        switch(c)  
        {  
            case KEYCODE_W:
                IncreaseX();                 
                break;  
            case KEYCODE_S:  
                DecreaseX();  
                break;  
            case KEYCODE_A:
                IncreaseY();     
                break;  
            case KEYCODE_D:
                DecreaseY();     
                break;                   
            case KEYCODE_W_CAP:
                IncreaseZ(); 
                break;  
            case KEYCODE_S_CAP:
                DecreaseZ(); 
                break;     
            case KEYCODE_A_CAP:
                IncreaseW();  
                break;  
            case KEYCODE_D_CAP:
                DecreaseW();   
                break;             
            default:  
                 int i=0;
        }  
        position_.x=x;
        position_.y=y;
        position_.z=z;
        position_.w=w;
        pub_.publish(position_);  
    }  
} 

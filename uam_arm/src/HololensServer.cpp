#include <iostream>
#include "string"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include "ros/ros.h"
#include <ros/time.h>  
#include <unistd.h> 
#include "vector"
#include <sstream>
#include <geometry_msgs/Quaternion.h>
#include "sensor_msgs/JointState.h"

using namespace std;

vector<float>positions;
bool IsSend=false;

void chatterCallback(const sensor_msgs::JointState& msg)
{
  positions.clear();
  for(int j=0;j<6;j++)
  {
      positions.push_back(msg.position[j]);
  }
  IsSend=true;
}

template<typename T> string toString(const T& t){
    ostringstream oss;  //创建一个格式化输出流
    oss<<t;             //把值传递如流中
    return oss.str();   
}

template <class Type>  
Type stringToNum(const string& str)  
{  
    istringstream iss(str);  
    Type num;  
    iss >> num;  
    return num;      
}

void SplitString(const string& s, vector<string>& v, const string& c)
{
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while(string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));
         
        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
}
int main(int argc, char **argv)
{
    int client, server;
    int portNum = 6321;
    bool isExit = false;
    int bufsize = 1024;
    char buffer[bufsize];

    struct sockaddr_in server_addr;
    socklen_t size;

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::NodeHandle n2;
    ros::Subscriber sub = n.subscribe("/joint_states", 1, chatterCallback);
    ros::Publisher pub = n2.advertise<geometry_msgs::Quaternion>("/position", 1);
    ros::Rate loop_rate(40);

    client = socket(AF_INET, SOCK_STREAM, 0);

    if (client < 0) 
    {
        cout << "\nError establishing socket..." << endl;
        exit(1);
    }

    
    cout << "\n=> Socket server has been created..." << endl;

   
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htons(INADDR_ANY);
    server_addr.sin_port = htons(portNum);



    if ((bind(client, (struct sockaddr*)&server_addr,sizeof(server_addr))) < 0) 
    {
        cout << "=> Error binding connection, the socket has already been established..." << endl;
        return -1;
    }


    size = sizeof(server_addr);
    cout << "=> Looking for clients..." << endl;


    listen(client, 1);


    int clientCount = 1;
    server = accept(client,(struct sockaddr *)&server_addr,&size);

    // first check if it is valid or not
    if (server < 0) 
        cout << "=> Error on accepting..." << endl;

    while (server > 0) 
    {
        recv(server, buffer, bufsize, 0);
        cout<<buffer<<endl;
        geometry_msgs::Quaternion myVector;
        myVector.x=0.12;
        myVector.y=0;
        myVector.z=-0.2;
        myVector.w=100;
        while(server > 0)
        {
            if(IsSend)
            {
                string strSend="begin_";
                for(int i=0;i<positions.size();i++)
                {
                    string temp=toString(positions[i]);
                    strSend=strSend+temp+"_";
                }
                strSend=strSend+"end";
                for(int i=0;i<strSend.length();i++)
                {
                    buffer[i]=strSend[i];
                }
                strSend[strSend.length()]='\0';
                cout<<"Begin:"<<endl;
                send(server, buffer, bufsize, 0);
                cout<<"send:"<<buffer<<endl;
                recv(server, buffer, bufsize, 0);
                cout<<"received:"<<buffer<<endl;
                cout<<endl;
                string strRec=buffer;
      
                if(strRec[0]=='v'&&strRec[1]=='e')
                {
                    vector<string>parts;
                    SplitString(strRec,parts,"_");
                    try{
                        myVector.x=stringToNum<float>(parts[1]);
                        myVector.y=stringToNum<float>(parts[2]);
                        myVector.z=stringToNum<float>(parts[3]);
                        myVector.w=stringToNum<float>(parts[4]);
                    }
                    catch(exception& e){cout<<"error!!!"<<endl;}
                }
                pub.publish(myVector);
                IsSend=false;
                positions.clear();
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
        close(server);
        cout << "\nGoodbye..." << endl;
        isExit = false;
        exit(1);
    }

    close(client);
    return 0;
}

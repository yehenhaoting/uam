#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/PoseStamped.h> 
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <pwd.h>
#include <sys/stat.h>
#include <sys/types.h>


#include <sstream>
#include <string.h>
#include "CFetchViconData.h"

#include <dynamic_reconfigure/server.h>
#include <viconros/uam_init_Config.h>

std::ofstream logfile;

KDL::Frame F_world_arm, F_world_segment, F_arm_segment;   //机械臂末端FRD坐标系在vicon坐标系下表达，segment在vicon坐标系下表达，segment在机械臂末端FRD坐标系下表达

viconros::uam_init_Config cfg_param;

geometry_msgs::PoseStamped msg;
geometry_msgs::PoseStamped data_pose_uav;

float get_ros_time(ros::Time time_begin);
void data_log(std::ofstream &logfile, float cur_time);
int set_file();
bool log_once_flag = true;
int count = 0;

/**
 * 参数读取callback函数，用于获取飞行控制器的参数
 * @param config
 */
void param_cb(const viconros::uam_init_Config &config)
{
    cfg_param = config;
}

void pose_callback(const geometry_msgs::PoseStamped &pose)
{

    data_pose_uav.header = msg.header;
    data_pose_uav.pose.position = pose.pose.position;

    KDL::Rotation Rotation_uav = KDL::Rotation::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    Rotation_uav.GetRPY(data_pose_uav.pose.orientation.x,data_pose_uav.pose.orientation.y,data_pose_uav.pose.orientation.z);

}


int main(int argc, char **argv) {
	ros::init(argc, argv, "viconros_arm_test");
	ros::NodeHandle n("~");
	std::string ip;
	std::string model;
	std::string segment;

    set_file();


	n.getParam("host",ip);
	n.getParam("model",model);
	n.getParam("segment",segment);
	ROS_INFO("HOST:%s",ip.c_str());
	ROS_INFO("MODEL:%s; SEGMENT:%s",model.c_str(),segment.c_str());


    ros::Subscriber sub = n.subscribe("/uam_base/pose", 10, pose_callback);

	ros::Publisher arm_pub = n.advertise<geometry_msgs::PoseStamped> ("/record/arm_pose", 10);
	ros::Publisher uav_pub = n.advertise<geometry_msgs::PoseStamped> ("/record/uav_pose", 10);

    // 通过参数服务器的方式，获取控制器的参数
    dynamic_reconfigure::Server<viconros::uam_init_Config> server;
    dynamic_reconfigure::Server<viconros::uam_init_Config>::CallbackType ff;
    ff = boost::bind(&param_cb, _1);
    server.setCallback(ff);


	ros::Rate loop_rate(50);
	CFetchViconData * vicon=new CFetchViconData();
	const char * host=ip.c_str();
	ObjStatus objs;
	
	if(!(vicon->IsConnected))
    { 
            ROS_INFO("Connecting to %s",host);
            bool res=vicon->Connect(host);
            if(res==false)
            {
                ROS_INFO("Failed to connect!\r\n");
                    return 0;
            }
            else
            {
                ROS_INFO("Successfully connected!\r\n");
            }

    }
    ros::Time begin_time = ros::Time::now();
	while (ros::ok()) {
        float cur_time = get_ros_time(begin_time);  // 当前时间
        vicon->GetStatus(objs, model.c_str(),segment.c_str());

//        msg.header.stamp.sec=(int)objs.tm;
//        msg.header.stamp.nsec=(objs.tm-msg.header.stamp.sec)*10000*100000;
//        msg.pose.position.x =objs.pos[0];
//        msg.pose.position.y =objs.pos[1];
//        msg.pose.position.z =objs.pos[2];
//
//        msg.pose.orientation.x =objs.ort[0];
//        msg.pose.orientation.y =objs.ort[1];
//        msg.pose.orientation.z =objs.ort[2];
//        msg.pose.orientation.w =objs.ort[3];

        KDL::Rotation ORT = KDL::Rotation::Quaternion(0,0,0,0);

        F_world_segment = KDL::Frame(KDL::Rotation::Quaternion(objs.ort[0], objs.ort[1], objs.ort[2], objs.ort[3]),KDL::Vector(objs.pos[0], objs.pos[1], objs.pos[2]));
        if(cfg_param.Correct_start){
            ROS_INFO_STREAM("Correcting the pose bias");
            F_arm_segment = KDL::Frame(KDL::Rotation::Quaternion(objs.ort[0], objs.ort[1], objs.ort[2], objs.ort[3]), KDL::Vector(cfg_param.EV_POS_X, cfg_param.EV_POS_Y, cfg_param.EV_POS_Z));
        }
        F_world_arm = F_world_segment * F_arm_segment.Inverse();


        msg.header.stamp.sec=(int)objs.tm;
        msg.header.stamp.nsec=(objs.tm-msg.header.stamp.sec)*10000*100000;
        msg.pose.position.x = F_world_arm.p.data[0] + cfg_param.BIAS_POS_X;
        msg.pose.position.y = F_world_arm.p.data[1] + cfg_param.BIAS_POS_Y;
        msg.pose.position.z = F_world_arm.p.data[2] + cfg_param.BIAS_POS_Z;

//        F_world_arm.M.GetQuaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
        F_world_arm.M.GetRPY(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z); //获取欧拉角信息

		arm_pub.publish(msg);
		uav_pub.publish(data_pose_uav);

        if(cfg_param.Enable_log_to_file){
            data_log(logfile, cur_time);                     //log输出
//            debug_log(debugfile, cur_time);
        }

        if(cfg_param.Log_to_file_once && log_once_flag){
            data_log(logfile, cur_time);                     //log输出
            log_once_flag = false;

        }

        if(!cfg_param.Log_to_file_once) log_once_flag = true;

		ros::spinOnce();
		loop_rate.sleep();
	}

    logfile.close();

	return 0;
}

/**
 * 获取当前时间 单位：秒
 */
float get_ros_time(ros::Time time_begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-time_begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - time_begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

/**
 * 设置log文件的储存方式
 * @return
 */
int set_file()
{
    // 更新home的文件夹位置，在home目录下新建OFFBOARD_PX4_log文件夹，用于存放log数据
    id_t uid;
    struct passwd* pwd;
    uid = getuid();
    pwd = getpwuid(uid);
    chdir(pwd->pw_dir);
    mkdir("./UAM_log", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    // 获取系统时间，作为log文件的文件名进行保存
    time_t currtime = time(nullptr);
    tm* p = localtime(&currtime);
    char log_filename[100] = {0};
    char debug_filename[100] = {0};


    sprintf(log_filename,"./UAM_log/log_%d%02d%02d_%02d_%02d_%02d.csv",p->tm_year+1900,p->tm_mon+1,p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);
    sprintf(debug_filename,"./UAM_log/debug_%d%02d%02d_%02d_%02d_%02d.csv",p->tm_year+1900,p->tm_mon+1,p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);


    // log输出文件初始化
    logfile.open(log_filename, std::ios::out);
    if (! logfile.is_open()){
        std::cerr<<"log to file error!"<<std::endl;
        return -1;
    }
}

/**
 * 将进入offboard后的位置&速度&姿态信息记录进文件
 * @param cur_time
 */
void data_log(std::ofstream &logfile, float cur_time)
{
    count ++;
    logfile << cur_time << ","
            <<"UAV"<<","<<data_pose_uav.pose.position.x <<","<<data_pose_uav.pose.position.y <<","<<data_pose_uav.pose.position.z <<","    //uav_pos
            <<data_pose_uav.pose.orientation.x <<","<<data_pose_uav.pose.orientation.y <<","<<data_pose_uav.pose.orientation.z <<"," <<data_pose_uav.pose.orientation.w //uav_ort
            <<","<<"ARM"<<","<<msg.pose.position.x <<","<<msg.pose.position.y <<","<<msg.pose.position.z <<","    //arm_pos
            <<msg.pose.orientation.x <<","<<msg.pose.orientation.y <<","<<msg.pose.orientation.z <<"," <<msg.pose.orientation.w << "," << //arm_ort
            std::endl;
    std::cout<<count<<"\tx:"<<msg.pose.position.x<<"\ty:"<<msg.pose.position.y<<"\tz:"<<msg.pose.position.z <<std::endl;
}
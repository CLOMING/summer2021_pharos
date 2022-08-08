#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/TwistStamped.h>
#include <string.h>
#include <stdlib.h>

#define setNum 16
using namespace std;

string dummy;
std_msgs::String msgInputSetting;
std::string settingValue[setNum] = {"empty"};

void okCheckCB(const std_msgs::String::ConstPtr& msg)
{
    dummy = msg->data; // <ok> 사인은 더이상 필요없으니까 더미에다가 잠깐 저장함. 이거 없으면 에러뜸.
//    system("rosnode kill ebimuSetting");
    ros::shutdown();
}

int main (int argc, char** argv){

    ros::init(argc, argv, "ebimuCalibration");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Publisher settingValuePub = nh.advertise<std_msgs::String>("settingValue", 1);
    ros::Subscriber okCheckSub = nh.subscribe("okTopic", 1, okCheckCB);

    pnh.param<std::string>("settingValue0", settingValue[0], "empty");
    pnh.param<std::string>("settingValue1", settingValue[1], "empty");
    pnh.param<std::string>("settingValue2", settingValue[2], "empty");
    pnh.param<std::string>("settingValue3", settingValue[3], "empty");
    pnh.param<std::string>("settingValue4", settingValue[4], "empty");
    pnh.param<std::string>("settingValue5", settingValue[5], "empty");
    pnh.param<std::string>("settingValue6", settingValue[6], "empty");
    pnh.param<std::string>("settingValue7", settingValue[7], "empty");
    pnh.param<std::string>("settingValue8", settingValue[8], "empty");
    pnh.param<std::string>("settingValue9", settingValue[9], "empty");
    pnh.param<std::string>("settingValue10", settingValue[10], "empty");
    pnh.param<std::string>("settingValue11", settingValue[11], "empty");
    pnh.param<std::string>("settingValue12", settingValue[12], "empty");
    pnh.param<std::string>("settingValue13", settingValue[13], "empty");
    pnh.param<std::string>("settingValue14", settingValue[14], "empty");
    pnh.param<std::string>("settingValue15", settingValue[15], "empty");

    std::stringstream inputSettingParam;
    for (int i = 0; i < setNum; ++i)
    {
        if(settingValue[i] != "empty")
        {
            inputSettingParam << "<" << settingValue[i] << ">" << endl;
        }
    }

    ros::Rate loopRate(0.5);

    while(ros::ok())
    {
        ros::spinOnce();
        msgInputSetting.data = inputSettingParam.str();
        settingValuePub.publish(msgInputSetting);

        loopRate.sleep();
    }


}

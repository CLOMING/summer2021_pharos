#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <std_msgs/Header.h>

#define repeatNum 4
#define waitingTime 2

bool isInit(true);
bool isFinishSetting(false);

using namespace std;
std_msgs::Header::_stamp_type previousStamp;

int main (int argc, char** argv)
{
    ros::init(argc, argv, "setting_for_imuReset");
    ros::Time::init();

    ros::NodeHandle nh;
    ros::Publisher settingValuePub = nh.advertise<std_msgs::String>("/ebimu/settingValue", 10);
    std::string inputString;
    std_msgs::String msg;

    previousStamp = ros::Time::now();
    unsigned int stateNum(0);
    while(ros::ok())
    {
        if(ros::Duration(ros::Time::now() - previousStamp).toSec() > waitingTime)
        {
            if(stateNum == 0)
            {
                msg.data = "<sog1>\n";
                settingValuePub.publish(msg);
                stateNum++;
            }
            else if(stateNum == 1)
            {
                msg.data = "<soa1>\n";
                settingValuePub.publish(msg);
                stateNum++;
            }
            else if(stateNum == 2)
            {
                msg.data = "<sot1>\n";
                settingValuePub.publish(msg);
                stateNum++;
            }
            else if(stateNum == 3)
            {
                ros::shutdown();
            }
            previousStamp = ros::Time::now();
        }

    }


}

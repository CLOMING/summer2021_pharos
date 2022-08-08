#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

using namespace std;

int main (int argc, char** argv)
{
    ros::init(argc, argv, "inputConsoleEbimuSetting");

    ros::NodeHandle nh;
    ros::Publisher settingValuePub = nh.advertise<std_msgs::String>("/ebimu/settingValue", 10);
    while(ros::ok())
    {
        std::string inputString;
        cout << "inputSettingValue : " << endl;
        std::getline(std::cin, inputString);
        std_msgs::String msg;
        msg.data = inputString;
        settingValuePub.publish(msg);
    }
}

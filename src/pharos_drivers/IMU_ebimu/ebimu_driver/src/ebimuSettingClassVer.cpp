#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/TwistStamped.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>

#define setNum 16
using namespace std;

class imuSetting
{
private:
    string dummy;
    std_msgs::String msgInputSetting;
    std::string settingValue[setNum] = {""};
    std::stringstream inputSettingParam;
    ros::Publisher settingValuePub;
    ros::Subscriber okCheckSub;
    bool isOkChecked;

public:
    imuSetting()
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        settingValuePub = nh.advertise<std_msgs::String>("settingValue", 1, this);
        okCheckSub = nh.subscribe("okTopic", 1, &imuSetting::okCheckCB, this);

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

        isOkChecked = false;
    }

    ~imuSetting()
    {

    }
    void okCheckCB(const std_msgs::String::ConstPtr& msg)
    {
        isOkChecked = true;
        dummy = msg->data; // <ok> 사인은 더이상 필요없으니까 더미에다가 잠깐 저장함. 이거 없으면 에러뜸.
//    system("rosnode kill ebimuSetting");
        inputSettingParam.str("");
        msgInputSetting.data = "";
        settingValuePub.publish(msgInputSetting);
        ros::shutdown();
    }

    void topicPublisher()
    {
        for (int i = 0; i < setNum; ++i)
        {
            if(settingValue[i] != "empty") //런치파일에 값이 안들어온 경우
            {
                inputSettingParam << "<" << settingValue[i] << ">" << endl; //xml형식인 launch 파일은 < 을 인식하기 때문에 이렇게 작성
            }
        }

        ros::Rate loopRate(0.5); // 빠르게 입력해서 좋을거 없었음. 2초에 한 번 Publish가  적당한 듯.

        while(ros::ok())
        {
            if(isOkChecked)
            {
                break;
            }
            ros::spinOnce();
            msgInputSetting.data = inputSettingParam.str();
            settingValuePub.publish(msgInputSetting);
            loopRate.sleep();
        }
    }


};





    /*
     *v01 190224 2230, 한지완
     *
     * 1. ser.write 기능 활용. setting 런치파일을 키면 런치파일에 있는
     *    세팅 내용을 시리얼에 입력하여 바로 세팅을 적용할 수 있다.
     */


int main (int argc, char** argv)
{

    ros::init(argc, argv, "ebimuSetting");
    imuSetting imuSetting1;
    imuSetting1.topicPublisher();
    return 0;

}

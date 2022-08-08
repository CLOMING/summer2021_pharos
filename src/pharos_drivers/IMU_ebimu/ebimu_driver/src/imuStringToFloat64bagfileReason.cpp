#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/TwistStamped.h>
#include <string.h>
#include <stdlib.h>
#include <pharos_msgs/imuDataStamped.h>

#define stringSize 256

double hzNum;
bool isRad;

using namespace std;
//double roll, pitch, yaw;
//geometry_msgs::TwistStamped imuRPY; // 퍼블리시할 메시지, geometry_msgs::TwistStamped 메시지 선정.

pharos_msgs::imuDataStamped imuData;

/*
 * 이상한 값(회로에서 오류가 나서 이상한 값이 나올 경우
 * 에러가 뜨므로 모든 data 에서 아스키코드를 이용하여 원하는 값이 나왔는지 확인한다.
 */

bool abnormalCheck(char *data)
{
    for (int i = 0; i < stringSize; ++i)
    {
        if(0 <= data[i] && data[i] <= 114) // 아스키코드 참고, 이 안에 있지 않은 아스키코드는 모두 무시함.
        {
            if(data[i] == '<') // <ok> 사인도 무시
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    return true; // 깐깐한 검증을 통과한 완전한 데이터임을 보증합니다. ^^
}

void imuCB(const std_msgs::String::ConstPtr& msg) // imu 콜백함수.
{
    /*
     * v01 1909221 1355, 한지완.
     * C기반의 스트링 선언 후 우선 토큰스타로 *을 거른 후,
     * comma 단위로 자름, 자른 string을 바로 float형으로 변환한 후에 RPY에 대입함.
     *
     * 그리고 Publish할 메시지에 각각의 값을 대입함.
     *  frame id와 time stamp를 대입하여 현재시간과 fream_id 기입. 콜백함수 끝.
     *
     * v02 190221 1552, 한지완
     * 추가 기능 :
     * 1. 런치파일에서 USB0와 같은 설정을 입력 할 수 있도록 함.
     * 2. 라디안 deg 고를수 있게 설정함. isRad 이라는 불리언이 true이면 rad 반환.
     *
     * v03 190224 2230, 한지완
     * 추가 기능 :
     * 1. ser.write 기능 활용. setting 런치파일을 키면 런치파일에 있는
     *    세팅 내용을 시리얼에 입력하여 바로 세팅을 적용할 수 있다.
     *
     */
    try
    {
        char rawCstring[stringSize] = {0};
        strcat(rawCstring, msg->data.c_str());
        if(abnormalCheck(rawCstring))
        {
            char *tokenStar = strtok(rawCstring, "*");
            /*
             * 자름과 동시에 값을 float로 바꾸고 대입함.
             * 중간중간에 -1을 곱한건 공간좌표 좌표축을 통일하기 위해 방향을 바꾼 것.
             */
            char *tokenComma = strtok(tokenStar, ",");
            imuData.imudata.RPY.x = atof(tokenComma) * -1.0;
            tokenComma = strtok(NULL, ",");
            imuData.imudata.RPY.y = atof(tokenComma);
            tokenComma = strtok(NULL, ",");
            imuData.imudata.RPY.z = atof(tokenComma) * -1.0;
            tokenComma = strtok(NULL, ","); // 자르고 남은건 버림.
            imuData.imudata.AngularVelocity.x = atof(tokenComma) * -1.0;
            tokenComma = strtok(NULL, ",");
            imuData.imudata.AngularVelocity.y = atof(tokenComma) * -1.0;
            tokenComma = strtok(NULL, ",");
            imuData.imudata.AngularVelocity.z = atof(tokenComma);
            tokenComma = strtok(NULL, ",");
            imuData.imudata.Accel.x = atof(tokenComma);
            tokenComma = strtok(NULL, ",");
            imuData.imudata.Accel.y = atof(tokenComma);
            tokenComma = strtok(NULL, ",");
            imuData.imudata.Accel.z = atof(tokenComma)  * -1.0;
            tokenComma = strtok(NULL, ",");
            imuData.imudata.Temp = atof(tokenComma);
            tokenComma = strtok(NULL, "\r");
        }
    }
    catch (exception &e)
    {
        cout << e.what() << endl;
    }

    /*
     * 라디안을 하고 싶은 경우에는 아래와 같이
     */
    if(isRad)
    {
        imuData.imudata.RPY.x = imuData.imudata.RPY.x * M_PI/180.0;
        imuData.imudata.RPY.y = imuData.imudata.RPY.y * M_PI/180.0;
        imuData.imudata.RPY.z = imuData.imudata.RPY.z * M_PI/180.0;
    }

    imuData.header.frame_id = "ebimu";
    imuData.header.stamp = ros::Time::now();


}


int main (int argc, char** argv){
    /*
     * 전반적인 로스 Subscriber, Publisher 선언, 노드 핸들 선언.
     * 100hz 로 Publish하도록 설정되어 있으며
     * ros::spinOnce() 로 topic을 전달받으면 한번 콜백함수가 돌도록 되어있음.
     */
    ros::init(argc, argv, "imuStringToFloat64_bagfile");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<double>("setHz", hzNum, 100);
    pnh.param<bool>("isRad", isRad, 0);

    ros::Subscriber imuDataSub = nh.subscribe("/ebimu/read", 1, imuCB);

    ros::Publisher imuDataPub = nh.advertise<pharos_msgs::imuDataStamped>("/ebimu/Data02", 1);

    ros::Rate loop_rate(hzNum);
    while(ros::ok()){
        ros::spinOnce();
        imuDataPub.publish(imuData);
        loop_rate.sleep();
    }
}

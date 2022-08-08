#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <pharos_msgs/StateStamped2016.h>
#include <pharos_msgs/ObjectInfoArray.h>
#include <pharos_msgs/ObjectPose.h>
#include <pharos_msgs/ObjectSize.h>
#include <pharos_msgs/ObjectInfo.h>
#include <pharos_msgs/ObjectCollision.h>
#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>
#include "wave_tcp/BasicSafetyMessage_DATA.h"
#include "wave_tcp/BasicSafetyMessage_DATA_List.h"
#include "../../../pharos_drivers/wave_tcp/include/CommunicationHeader.h" // 세번 뒤로.
#include <string>
#include <gps_common/conversions.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int64MultiArray.h>

#define Num 3
using namespace std;

std_msgs::Header rosheader;


int main(int argc, char **argv)          //노드 메인함수 char **이다 계속 int하는데 주의할것
{
    ros::init(argc, argv, "v2v_practiceJiwan_Pub");       //노드명 초기화
    ros::NodeHandle nh;
    ROS_INFO("START");

    ros::Publisher stringPub = nh.advertise<std_msgs::String>("temporaryV2Vinfo", 1);

    ros::Rate loop_rate(5);
    std_msgs::String PubPubV2X2;

    rosheader.stamp = ros::Time::now();
    while (ros::ok())
    {
        std_msgs::String Pubmsg01;

        if(rosheader.stamp.sec + 0 <= ros::Time::now().sec && ros::Time::now().sec < rosheader.stamp.sec + 2)
        {
            Pubmsg01.data = "id1";
            stringPub.publish(Pubmsg01);
            ROS_INFO_STREAM(Pubmsg01.data << endl);
            ROS_INFO_STREAM("---------------------0" << endl);
        }
        if(rosheader.stamp.sec + 2 <= ros::Time::now().sec && ros::Time::now().sec < rosheader.stamp.sec + 4)
        {
            Pubmsg01.data += "id1\nid2";
            stringPub.publish(Pubmsg01);
            ROS_INFO_STREAM(Pubmsg01.data << endl);
            ROS_INFO_STREAM("---------------------1" << endl);
        }

        else if(rosheader.stamp.sec + 4 <= ros::Time::now().sec && ros::Time::now().sec < rosheader.stamp.sec + 6)
        {
            /*
             * +=이 안 먹는다. std::string만 되는듯 하다
             */
            Pubmsg01.data += "id1\nid2\nid3";
            stringPub.publish(Pubmsg01);
            ROS_INFO_STREAM(Pubmsg01.data << endl);
            ROS_INFO_STREAM("---------------------2" << endl);

        }

        else if(rosheader.stamp.sec + 6 <= ros::Time::now().sec && ros::Time::now().sec < rosheader.stamp.sec + 8)
        {
            string Pubstring = "id1";
            Pubstring += "\nid2";
            Pubmsg01.data = Pubstring;
            stringPub.publish(Pubmsg01);
            ROS_INFO_STREAM(Pubmsg01.data << endl);
            ROS_INFO_STREAM("---------------------3" << endl);



        }

        else if(rosheader.stamp.sec + 8 <= ros::Time::now().sec && ros::Time::now().sec < rosheader.stamp.sec + 10)
        {
            rosheader.stamp = ros::Time::now();

        }

        loop_rate.sleep();
    }





    return 0;
}



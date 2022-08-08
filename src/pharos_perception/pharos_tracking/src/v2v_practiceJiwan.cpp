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

using namespace std;
std::string helloV2X;

void msgCB(const std_msgs::String &msg)
{
    helloV2X = msg.data;
    ROS_INFO_STREAM("helloV2X : " << endl << helloV2X << endl);

}

int main(int argc, char **argv)          //노드 메인함수 char **이다 계속 int하는데 주의할것
{
    ros::init(argc, argv, "v2v_practiceJiwanSub");       //노드명 초기화
    ros::NodeHandle nh;
    ROS_INFO("START");

    ros::Subscriber stringSub = nh.subscribe("temporaryV2Vinfo", 1, msgCB);


    ros::spin();



    return 0;
}



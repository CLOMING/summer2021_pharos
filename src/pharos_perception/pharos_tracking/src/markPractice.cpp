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

int main(int argc, char **argv)          //노드 메인함수 char **이다 계속 int하는데 주의할것
{
    ros::init(argc, argv, "markPractice");       //노드명 초기화
    ros::NodeHandle nh;
    ROS_INFO("START");
    ros::Rate r(10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

//    uint32_t shape = visualization_msgs::Marker::CUBE;
    unsigned int shape = visualization_msgs::Marker::CUBE;

    visualization_msgs::Marker marker;
    marker.pose.position.x = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = cosf(22.5 * M_PI/180.);
    marker.pose.orientation.w = sinf(22.5 * M_PI/180.);
    cout << sinf(22.5 * M_PI/180.) << endl;
    cout << sin(22.5 * M_PI/180.) << endl;
    cout << abs(-0.5) << endl;
    double a = -5.2;
    cout << abs(a) << endl;
    cout << fabs(a) << endl;
    while(ros::ok())
    {

        marker.header.frame_id = "/myMarkerFrame";
        marker.header.stamp = ros::Time::now();

        marker.ns = "kCity";
        marker.id = 0;
        marker.type = shape;

        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x++;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;

//        marker.pose.orientation.x++;
//        marker.pose.orientation.y++;


        marker.scale.x = 3.0;
        marker.scale.y = 5.0;
        marker.scale.z = 6.0;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration(); // If you don't enter anything, Duration is 0.
//        cout << marker.lifetime << endl;
        while (marker_pub.getNumSubscribers() < 1)
        {
            if(!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to this marker.");
            sleep(1);
        }
        marker_pub.publish(marker);

        switch (shape)
        {
//            case visualization_msgs::Marker::CUBE:
//                shape = visualization_msgs::Marker::SPHERE;
//                break;
//            case visualization_msgs::Marker::SPHERE:
//                shape = visualization_msgs::Marker::ARROW;
//                break;
//            case visualization_msgs::Marker::ARROW:
//                shape = visualization_msgs::Marker::CYLINDER;
//                break;
//            case visualization_msgs::Marker::CYLINDER:
//                shape = visualization_msgs::Marker::CUBE;
//                break;
        }

        r.sleep();

    }


    return 0;
}



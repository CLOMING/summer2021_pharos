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
#include "../../../pharos_drivers/wave_tcp/include/CommunicationHeader.h"
#include <string>
#include <gps_common/conversions.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

using std::string;


#define PI 3.14159265359
wave_tcp::BasicSafetyMessage_DATA_ListPtr BSM_data (new wave_tcp::BasicSafetyMessage_DATA_List);

int step = 0;
std::vector<wave_tcp::BasicSafetyMessage_DATA_List> BSM_vec;
// int bsm_size = 0;

class v2v_ {
public:

    v2v_() {
        nh = ros::NodeHandlePtr(new ros::NodeHandle(""));

        pharos_sub = nh->subscribe("pharos_tracked2_msg", 10, &v2v_::pharos_CB, this);
        wave_sub = nh->subscribe("/wave/BasicSafetyMessage_DATA",10, &v2v_::wave_CB, this);
        pharos_pub = nh->advertise<pharos_msgs::ObjectInfoArray>("/tracked_pharos_msgs", 10);
        wave_pub = nh->advertise<geometry_msgs::PoseArray>("/wave_posearray", 10);
        pcl_pub = nh->advertise<sensor_msgs::PointCloud2>("/wave_pointcloud", 10);
        vis_pub = nh->advertise<visualization_msgs::MarkerArray>("/wave_markerarray",10);

        ros::param::get("gps/origin/x", origin_x_);
        ros::param::get("gps/origin/y", origin_y_);
        ros::param::get("gps/origin/z", origin_z_);
    }

    void wave_CB(const wave_tcp::BasicSafetyMessage_DATA_ListPtr &msg) {   //우선 들어오는 값들은 sensormsg 형태로 들어오게 된다.
        // std::cout << "BSM_sub" << std::endl;
        // bsm_size = msg->BasicSafetyMessage_DATA_List.size();
        for(int i = 0; i < msg->BasicSafetyMessage_DATA_List.size(); i++)
        {
            bool matched = false;
            for(int j =0; j<BSM_data->BasicSafetyMessage_DATA_List.size();j++)
            {
                if(msg->BasicSafetyMessage_DATA_List[i].payload.BSMcoreData.Tempoaray_ID == BSM_data->BasicSafetyMessage_DATA_List[j].payload.BSMcoreData.Tempoaray_ID)
                    {
                        BSM_data->BasicSafetyMessage_DATA_List[j]= msg->BasicSafetyMessage_DATA_List[i];
                        matched = true;
                    }
            }
            if(!matched)
            {
                wave_tcp::BasicSafetyMessage_DATAPtr v2v_data (new wave_tcp::BasicSafetyMessage_DATA);
                *v2v_data = msg->BasicSafetyMessage_DATA_List[i];
                BSM_data->BasicSafetyMessage_DATA_List.push_back(*v2v_data);
            }
        }

    }

    void pharos_CB(const pharos_msgs::ObjectInfoArrayPtr &msg) {   //우선 들어오는 값들은 sensormsg 형태로 들어오게 된다.
        // std::cout << "-------------------------------------------" << std::endl;
        pharos_msgs::ObjectInfoArrayPtr wave_odom_Infoarray (new pharos_msgs::ObjectInfoArray);
        wave_odom_Infoarray=msg;
        pcl::PointCloud<pcl::PointXYZI>::Ptr wave_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
        wave_tcp::BasicSafetyMessage_DATA_ListPtr v2v_datas (new wave_tcp::BasicSafetyMessage_DATA_List);
        geometry_msgs::PoseArrayPtr wave_posearray(new geometry_msgs::PoseArray);

        BSM_vec.push_back(*BSM_data);

        if(BSM_vec.size()==1)
            v2v_datas->BasicSafetyMessage_DATA_List = BSM_vec[0].BasicSafetyMessage_DATA_List;

        if(BSM_vec.size()==2)
        {
            v2v_datas->BasicSafetyMessage_DATA_List = BSM_vec[1].BasicSafetyMessage_DATA_List;
            for(int i=0; i<BSM_vec[0].BasicSafetyMessage_DATA_List.size(); i++)
            {
                bool matched=false;
                for(int j=0; j<v2v_datas->BasicSafetyMessage_DATA_List.size(); j++)
                {
                    if(v2v_datas->BasicSafetyMessage_DATA_List[j].payload.BSMcoreData.Tempoaray_ID == BSM_vec[0].BasicSafetyMessage_DATA_List[i].payload.BSMcoreData.Tempoaray_ID)
                    {
                        matched = true;
                    }
                }
                if(!matched)
                {
                    wave_tcp::BasicSafetyMessage_DATAPtr v2v_data (new wave_tcp::BasicSafetyMessage_DATA);
                    *v2v_data = BSM_vec[0].BasicSafetyMessage_DATA_List[i];
                    v2v_datas->BasicSafetyMessage_DATA_List.push_back(*v2v_data);
                }
            }
            
        }

        //경도,위도를 x,y로 변환
        for(int i = 0; i < v2v_datas->BasicSafetyMessage_DATA_List.size(); i++)
        {
            double latitude = v2v_datas->BasicSafetyMessage_DATA_List[i].payload.BSMcoreData.GPS_Position.Latitude;
            double longitude = v2v_datas->BasicSafetyMessage_DATA_List[i].payload.BSMcoreData.GPS_Position.Longitude;

            double coordinated_x,coordinated_y,coordinated_z;

            std::string zone;
            gps_common::LLtoUTM(latitude,longitude,coordinated_y,coordinated_x,zone);

            ros::param::get("gps/origin/x", origin_x_);
            ros::param::get("gps/origin/y", origin_y_);
            ros::param::get("gps/origin/z", origin_z_);

            coordinated_x -= origin_x_; //odom_x
            coordinated_y -= origin_y_; //odom_y

            //북위 기준 Heading을 odom기준으로 맞춘다.
            double heading = v2v_datas->BasicSafetyMessage_DATA_List[i].payload.BSMcoreData.Heading+80;

            //heading은 dgree 이므로 radian으로 변환해준다.
            double angle = (PI/180)*heading;
            double speed = v2v_datas->BasicSafetyMessage_DATA_List[i].payload.BSMcoreData.Speed;

            pharos_msgs::ObjectInfoArrayPtr wave_Infoarray (new pharos_msgs::ObjectInfoArray);
            wave_Infoarray=wave_odom_Infoarray;
            pharos_msgs::ObjectInfoPtr wave_Objectinfo (new pharos_msgs::ObjectInfo);
//            wave_Objectinfo = wave_Infoarray.objects[i];
            wave_Objectinfo->id = v2v_datas->BasicSafetyMessage_DATA_List[i].payload.BSMcoreData.Tempoaray_ID;
            wave_Objectinfo->pose.x = coordinated_x;
            wave_Objectinfo->pose.y = coordinated_y;

            wave_Objectinfo->speed = speed;
            wave_Objectinfo->pose.theta = angle;

            //BSM의 운행상태를 판단한다. 2 -> 자율주행 3->정비차량 4->사고차량
            if (v2v_datas->BasicSafetyMessage_DATA_List[i].payload.VehicleSafetyExtensions.VehicleEventFlags.reserved == 64)
            {wave_Objectinfo->vehicle_state = 2;}
            if (v2v_datas->BasicSafetyMessage_DATA_List[i].payload.VehicleSafetyExtensions.VehicleEventFlags.reserved == 65)
            {wave_Objectinfo->vehicle_state = 3;}
            if (v2v_datas->BasicSafetyMessage_DATA_List[i].payload.VehicleSafetyExtensions.VehicleEventFlags.reserved == 66)
            {wave_Objectinfo->vehicle_state = 4;}


            double x_tracking_max = wave_Objectinfo->pose.x+5;
            double x_tracking_min = wave_Objectinfo->pose.x-5;
            double y_tracking_max = wave_Objectinfo->pose.y+5;
            double y_tracking_min = wave_Objectinfo->pose.y-5;


            //BSM이 들어올 경우 해당 영역안에 tracking code는 배제시킨다.
            double x_max = wave_Objectinfo->pose.x+3;
            double x_min = wave_Objectinfo->pose.x-3;
            double y_max = wave_Objectinfo->pose.y+3;
            double y_min = wave_Objectinfo->pose.y-3;

            bool matched = false;
            for(int j=0; j<wave_odom_Infoarray->objects.size(); j++)
            {
                if(wave_odom_Infoarray->objects[j].pose.x<x_tracking_max &&
                   wave_odom_Infoarray->objects[j].pose.x>x_tracking_min &&
                   wave_odom_Infoarray->objects[j].pose.y<y_tracking_max &&
                   wave_odom_Infoarray->objects[j].pose.y>y_tracking_min)
                {
                    if(wave_odom_Infoarray->objects[j].pose.x>x_min &&
                       wave_odom_Infoarray->objects[j].pose.x<x_max &&
                       wave_odom_Infoarray->objects[j].pose.y>y_min &&
                       wave_odom_Infoarray->objects[j].pose.y<y_max)
                    {
                        wave_odom_Infoarray->objects[j].speed = wave_Objectinfo->speed;
                        wave_odom_Infoarray->objects[j].pose.theta = wave_Objectinfo->pose.theta;
                        wave_odom_Infoarray->objects[j].vehicle_state = wave_Objectinfo->vehicle_state;
                        wave_odom_Infoarray->objects[j].id = wave_Objectinfo->id;

                    }
                matched = true;
                }
            }
            if(!matched){
                wave_odom_Infoarray->objects.push_back(*wave_Objectinfo);
                geometry_msgs::Pose pose;
                pcl::PointXYZI wave_point;
                
                wave_point.x=coordinated_x;
                wave_point.y=coordinated_y;
                wave_point.z=10;
                wave_point.intensity=10;

                pose.position.x = coordinated_x;
                pose.position.y = coordinated_y;
                pose.position.z = 0.0;

                wave_pointcloud->points.push_back(wave_point);  
                wave_posearray->poses.push_back(pose);  
            }
            
        }
        for(int i = 0; i < wave_odom_Infoarray->objects.size(); i++)
        {
            string s;
            float an = wave_odom_Infoarray->objects[i].pose.theta;
            float sp = wave_odom_Infoarray->objects[i].speed;
            int st = wave_odom_Infoarray->objects[i].vehicle_state;
            s = "Heading : " + std::to_string(an)
                + "\nSPEED : " + std::to_string(sp) + "\nSTATE : "
                + std::to_string(st);
            visualization_msgs::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = wave_odom_Infoarray->header.stamp;
            marker.ns = "my_namespace";
            marker.id = i;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.text = s;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = wave_odom_Infoarray->objects[i].pose.x;
            marker.pose.position.y = wave_odom_Infoarray->objects[i].pose.y;
            marker.pose.position.z = 10;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.a = 5.0;
            marker.color.r = 2.0;
            marker.color.g = 2.0;
            marker.color.b = 0.0;
            marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

            visualization_msgs::MarkerArray markerarray;
            markerarray.markers.push_back(marker);
            vis_pub.publish(markerarray);
        }

        sensor_msgs::PointCloud2 center_pointcloud;
        pcl::toROSMsg(*wave_pointcloud,center_pointcloud);
        center_pointcloud.header.frame_id = "/odom";
        center_pointcloud.header.stamp = wave_odom_Infoarray->header.stamp;

        wave_posearray->header.frame_id = "odom";
        pcl_pub.publish(center_pointcloud);
        wave_pub.publish(*wave_posearray);
        pharos_pub.publish(*wave_odom_Infoarray);

        // std::cout << "step : " <<step <<std::endl;
        step ++;
        // bsm_size_vec.push_back(bsm_size);
        // std::cout << "BSM_size : " <<bsm_size <<std::endl;
        // bsm_size = 0;
        // for(int i=0; i<v2v_datas->BasicSafetyMessage_DATA_List.size(); i++)
        // {
        // std::cout << "v2v_datas : " <<v2v_datas->BasicSafetyMessage_DATA_List.size() <<std::endl;
        // }
        if(BSM_vec.size()==2)
        {
            BSM_vec.erase(BSM_vec.begin());
        }
        BSM_data->BasicSafetyMessage_DATA_List.clear();

        // if(step > 3)
        // {
        //     v2v_datas->BasicSafetyMessage_DATA_List.erase(v2v_datas->BasicSafetyMessage_DATA_List.begin());
        //     step = 0;
        // }
    }










protected:
    ros::NodeHandlePtr nh;
    ros::NodeHandle pnh;

    ros::Publisher pharos_pub;
    ros::Publisher wave_pub;
    ros::Publisher pcl_pub;
    ros::Publisher vis_pub;

    ros::Subscriber pharos_sub;
    ros::Subscriber wave_sub;

    double origin_x_, origin_y_, origin_z_;
};




int main(int argc, char **argv)          //노드 메인함수 char **이다 계속 int하는데 주의할것
{
    ros::init(argc, argv, "V2V");       //노드명 초기화

    ROS_INFO("START");
    v2v_ v2v;
    ros::spin();



    return 0;
}



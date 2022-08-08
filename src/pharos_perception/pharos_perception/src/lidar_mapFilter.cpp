#include "utility.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>

double diff_value;
bool isMapInit = false;
std::vector<int> MapInitVector;

nav_msgs::Odometry Odom_;

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
nav_msgs::OccupancyGridPtr drivable_map (new nav_msgs::OccupancyGrid);


class Map_Filter {
private:
    ros::NodeHandle nh; 
    ros::NodeHandlePtr pnh;
    
public:
    ros::Subscriber sub_Obstacle;
    ros::Subscriber sub_Surface;
    ros::Subscriber sub_Map;
    ros::Subscriber sub_odom_topic;
    ros::Publisher pub_obstacle_FILT;
    ros::Publisher pub_surface_FILT;

    std::string sub_obstacle_Param, sub_surface_Param, sub_ObstMap_Param;
    std::string pub_obstacleFilt_Param, pub_surfaceFilt_Param;

    std::string target_frame = "map";
    sensor_msgs::PointCloud2 obstacleFilt_PCL_ROS;
    sensor_msgs::PointCloud2 surfaceFilt_PCL_ROS;

    Map_Filter(){
        Eigen::Affine3f vehicle_tf;

        pnh = ros::NodeHandlePtr(new ros::NodeHandle("~"));
        pnh->param<std::string>("sub_obstacle_Param", sub_obstacle_Param, "sub_obstacle_Param");
        pnh->param<std::string>("sub_surface_Param", sub_surface_Param, "sub_surface_Param");
        pnh->param<std::string>("sub_ObstMap_Param", sub_ObstMap_Param, "sub_ObstMap_Param");

        pnh->param<std::string>("pub_obstacleFilt_Param", pub_obstacleFilt_Param, "pub_obstacleFilt_Param");
        pnh->param<std::string>("pub_surfaceFilt_Param", pub_surfaceFilt_Param, "pub_surfaceFilt_Param");

        sub_Obstacle = nh.subscribe<sensor_msgs::PointCloud2>(sub_obstacle_Param , 10 , &Map_Filter::mapFilter_Obstacle_CB, this);
        sub_Surface = nh.subscribe<sensor_msgs::PointCloud2>(sub_surface_Param , 10 , &Map_Filter::mapFilter_Surface_CB, this);
        sub_Map = nh.subscribe(sub_ObstMap_Param , 10 , &Map_Filter::map_CB, this);

        pub_obstacle_FILT = nh.advertise<sensor_msgs::PointCloud2>(pub_obstacleFilt_Param,10);
        pub_surface_FILT = nh.advertise<sensor_msgs::PointCloud2>(pub_surfaceFilt_Param,10);
    }

    void map_CB(const nav_msgs::OccupancyGridPtr& map)
    {
        isMapInit = true;
        drivable_map = map;
    }

    void mapFilter_Obstacle_CB(const sensor_msgs::PointCloud2::ConstPtr& input_PCL_ROS)
    {   
        if(!isMapInit)
        {
            return;
        }

        obstacleFilt_PCL_ROS = mapFilter(input_PCL_ROS);
        pub_obstacle_FILT.publish(obstacleFilt_PCL_ROS);

    }  

    void mapFilter_Surface_CB(const sensor_msgs::PointCloud2::ConstPtr& input_PCL_ROS)
    {   
        if(!isMapInit)
        {
            return;
        }

        surfaceFilt_PCL_ROS = mapFilter(input_PCL_ROS);
        pub_surface_FILT.publish(surfaceFilt_PCL_ROS);

    }  

    sensor_msgs::PointCloud2 mapFilter(const sensor_msgs::PointCloud2ConstPtr& input_PCL_ROS)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_PCL (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input_PCL_ROS, *input_PCL);

        pcl::PointCloud<pcl::PointXYZI>::Ptr inputMap_PCL_TF (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr outputMap_PCL_TF (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_PCL (new pcl::PointCloud<pcl::PointXYZI>);

        static tf::TransformListener listener;

        tf::StampedTransform transform;
        try {
            listener.lookupTransform(target_frame, "vehicle_frame", ros::Time(0), transform);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            sensor_msgs::PointCloud2 error_PCL;
            return error_PCL;
        }

        pcl_ros::transformPointCloud(*input_PCL, *inputMap_PCL_TF, transform);
        double resolutionInverse = 1 / drivable_map->info.resolution;
        
        for(int nIndex=0; nIndex < inputMap_PCL_TF->points.size(); nIndex++){
            if (drivable_map->data.size() != 0) { 

                int xIndex, yIndex;
                xIndex = (int) (( inputMap_PCL_TF->points[nIndex].x ) * resolutionInverse); 
                yIndex = (int) (( inputMap_PCL_TF->points[nIndex].y ) * resolutionInverse);

                int mapIndex = MAP_IDX(drivable_map->info.width, xIndex, yIndex);

                if(mapIndex < 0){
                    ROS_ERROR(" [PHAROS Perception] PointCloud Divider : Out of map");
                }else{
                    int mapdata = drivable_map->data[mapIndex];
                    if (mapdata == 0) {
                        outputMap_PCL_TF->points.push_back(inputMap_PCL_TF->points[nIndex]);
                    } 
                }

            }
            else if(drivable_map->data.size() == 0 && nIndex == 0){              
                ROS_ERROR(" [PHAROS Perception] PointCloud Divider : None TF data available");
            } 
        }

        pcl_ros::transformPointCloud(*outputMap_PCL_TF, *output_PCL, transform.inverse());
        sensor_msgs::PointCloud2 pub_Filtered_PCL;
        pcl::toROSMsg(*output_PCL, pub_Filtered_PCL);
        pub_Filtered_PCL.header.frame_id = "vehicle_frame";
        pub_Filtered_PCL.header.stamp = input_PCL_ROS->header.stamp;

        return pub_Filtered_PCL;

    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Map_Filter_node");

    Map_Filter mapfilter;

    ROS_INFO("\033[1;32m----> [PHAROS Perception] PointCloud Map Filter : Initialized\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}

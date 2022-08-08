#include "utility.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>

std::string sub_Fused_Param;
std::string pub_Surface_Param, pub_Obstacle_Param;
float zSurface, zBoundary;

class LidarSegmentation {
private:
    ros::NodeHandle nh; 
    ros::NodeHandlePtr pnh;
    
public:
    ros::Subscriber subtransformedOuster;
    ros::Subscriber subOuster;
    ros::Publisher pubCombined;
    ros::Publisher pub_Surface;
    ros::Publisher pub_Obstacle;
    ros::Publisher pubObject_percep;
    
    LidarSegmentation(){
      pnh = ros::NodeHandlePtr(new ros::NodeHandle("~"));
      pnh->param<std::string>("sub_Fused_Param", sub_Fused_Param, "sub_Fused_Param");
      pnh->param<std::string>("pub_Surface_Param", pub_Surface_Param, "pub_Surface_Param");
      pnh->param<std::string>("pub_Obstacle_Param", pub_Obstacle_Param, "pub_Obstacle_Param");

      pnh->param<float>("zSurface", zSurface, 0.0);
      pnh->param<float>("zBoundary", zBoundary, 0.0);
    
      subOuster = nh.subscribe<sensor_msgs::PointCloud2>(sub_Fused_Param, 5, &LidarSegmentation::segmentation_CB, this, ros::TransportHints().tcpNoDelay());

      pub_Surface = nh.advertise<sensor_msgs::PointCloud2> (pub_Surface_Param, 10);
      pub_Obstacle = nh.advertise<sensor_msgs::PointCloud2> (pub_Obstacle_Param, 10);

    }

    void segmentation_CB(const sensor_msgs::PointCloud2ConstPtr& fused_PCL_ROS) {

      pcl::PointCloud<pcl::PointXYZI>::Ptr fused_PCL (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::PointCloud<pcl::PointXYZI>::Ptr surface_PCL (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_PCL (new pcl::PointCloud<pcl::PointXYZI> ());
   
      pcl::PointCloud<pcl::PointXYZI>::Ptr ground_PCL(new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::PointCloud<pcl::PointXYZI>::Ptr groundNeg_PCL(new pcl::PointCloud<pcl::PointXYZI> ());
            
      pcl::fromROSMsg(*fused_PCL_ROS, *fused_PCL);


      // [PHAROS Perception] - Vehicle Box Outlier
      pcl::CropBox<pcl::PointXYZI> vehicleROIFilter;
      vehicleROIFilter.setMin(Eigen::Vector4f(1, -1.5, 0, 1.0));      // X, Y, Z, 1
      vehicleROIFilter.setMax(Eigen::Vector4f(4, 1.5, 2.5, 1.0));      // X, Y, Z, 1
      vehicleROIFilter.setInputCloud(fused_PCL);
      vehicleROIFilter.setNegative(true);
      vehicleROIFilter.filter(*fused_PCL);

      // [PHAROS Perception] - Surfaace Outlier
      pcl::PassThrough<pcl::PointXYZI> obstCutROI;
      obstCutROI.setInputCloud (fused_PCL);                                     
      obstCutROI.setFilterFieldName ("z");                                         
      obstCutROI.setFilterLimits(zSurface-zBoundary, zSurface+zBoundary);         
      obstCutROI.filter(*ground_PCL);
      
      obstCutROI.setNegative(true);
      obstCutROI.filter(*groundNeg_PCL);
      
      // [PHAROS Perception] - Surface Detector
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::SACSegmentation<pcl::PointXYZI> surfaceROI;
      
      surfaceROI.setOptimizeCoefficients (true);
      surfaceROI.setModelType (pcl::SACMODEL_PLANE);
      surfaceROI.setMethodType (pcl::SAC_RANSAC);
      surfaceROI.setDistanceThreshold (0.3);
      surfaceROI.setInputCloud (ground_PCL);
      surfaceROI.segment (*inliers, *coefficients);
      
      pcl::copyPointCloud<pcl::PointXYZI>(*ground_PCL, *inliers, *surface_PCL);
      pcl::PointCloud<pcl::PointXYZI>::Ptr outlier_PCL (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::ExtractIndices<pcl::PointXYZI> extract;
      
      extract.setInputCloud(ground_PCL);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*outlier_PCL);
       
      // [PHAROS Perception] - Ostacle
      *obstacle_PCL += *groundNeg_PCL;
      *obstacle_PCL += *outlier_PCL;

      pcl::PassThrough<pcl::PointXYZI> boxfilterNEW;
      boxfilterNEW.setInputCloud (obstacle_PCL);
      boxfilterNEW.setFilterFieldName ("y");
      boxfilterNEW.setFilterLimits (-10.0, 10.0);
      boxfilterNEW.filter (*obstacle_PCL);
      
      sensor_msgs::PointCloud2 pub_SurfaceMsg;
      sensor_msgs::PointCloud2 pub_ObstacleMsg;
      
      pcl::toROSMsg(*obstacle_PCL, pub_ObstacleMsg);
      pub_ObstacleMsg.header.frame_id = "vehicle_frame";
      pub_Obstacle.publish(pub_ObstacleMsg);
      
      pcl::toROSMsg(*surface_PCL, pub_SurfaceMsg);
      pub_SurfaceMsg.header.frame_id = "vehicle_frame";
      pub_Surface.publish(pub_SurfaceMsg);
      
    }
    
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidarseg_node");

    LidarSegmentation lidarseg;

    ROS_INFO("\033[1;32m----> [PHAROS Perception] PointCloud Divider : Initialized\033[0m");

    ros::MultiThreadedSpinner spinner(10);
    spinner.spin();

    return 0;
}

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "divdearea_from_pcd");
    ros::NodeHandle node;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("autoware-180817.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;


    sensor_msgs::PointCloud2Ptr pcl_ros (new sensor_msgs::PointCloud2);

    pcl::toROSMsg(*cloud, *pcl_ros);

    pcl_ros->header.frame_id = "odom";
    pcl_ros->header.stamp = ros::Time::now();

    ros::Publisher pcl_ros_pub = node.advertise<sensor_msgs::PointCloud2>("raw_pcd", 1);

    while(ros::ok()){
        pcl_ros_pub.publish(pcl_ros);

        ros::Duration(10).sleep();
        std::cout << "pub" << std::endl;
    }


    return 0;
}
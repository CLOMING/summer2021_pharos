#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Subscriber pcl_sub;
ros::Publisher pcl_pub;

void VelodynePointsCB(const sensor_msgs::PointCloud2Ptr &msg){
    sensor_msgs::PointCloud2 cloud;
    cloud = *msg;
    pcl_pub.publish(cloud);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "velo_for_map_node");
    ros::NodeHandle nh;

    pcl_sub = nh.subscribe("velodyne_points", 1, VelodynePointsCB);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("points_raw", 1);

    ros::spin();

    return 0;
}
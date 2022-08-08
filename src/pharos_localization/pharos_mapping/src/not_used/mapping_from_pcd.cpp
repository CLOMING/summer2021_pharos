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

    ros::init(argc, argv, "mapping_from_pcd");       //노드명 초기화
    ros::NodeHandle node;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_road_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_vertical_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_vertical2d_filtered (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointIndicesPtr ground (new pcl::PointIndices);

    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("cloud_1.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;

//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZI>);
//    for(int i=0; i<cloud->points.size(); i++){
//        if(cloud->points[i].x>=0 && cloud->points[i].x<100 &&
//           cloud->points[i].y >=-100 && cloud->points[i].y <0){
//            cloud_1->points.push_back(cloud->points[i]);
//        }
//    }
//    cloud_1->width = cloud_1->points.size();
//    cloud_1->height = 1;
//
    // downsampling
    pcl::PCLPointCloud2Ptr cloud_pclcloud2 (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2Ptr pclcloud2_downsampled (new pcl::PCLPointCloud2 ());

    pcl::toPCLPointCloud2(*cloud, *cloud_pclcloud2);
    pcl::VoxelGrid<pcl::PCLPointCloud2> ds;
    ds.setInputCloud (cloud_pclcloud2);
    ds.setLeafSize (0.1f, 0.1f, 0.1f);
    ds.filter (*cloud_pclcloud2);
    pcl::fromPCLPointCloud2(*cloud_pclcloud2, *cloud_downsampled);
    std::cout << "downsample done " << cloud_downsampled->width << std::endl;

    // remove noise
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> rm;
    rm.setInputCloud (cloud_downsampled);
    rm.setMeanK (50);
    rm.setStddevMulThresh (1);
    rm.filter (*cloud_filtered);
    std::cout << "remove noise done " << cloud_filtered->width << std::endl;


//    // Create the filtering object
//    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZI> pmf;
//    pmf.setInputCloud (cloud_downsampled);
//    pmf.setMaxWindowSize (20);
//    pmf.setSlope (1.0f);
//    pmf.setInitialDistance (0.5f);
//    pmf.setMaxDistance (3.0f);
//    pmf.extract (ground->indices);
//    // Create the filtering object
//    pcl::ExtractIndices<pcl::PointXYZI> extract;
//    extract.setInputCloud (cloud);
//    extract.setIndices (ground);
//    extract.filter (*cloud_road_filtered);

    float min = 10000;
    for(int i=0;i<cloud_filtered->width; i++){
        if(cloud_filtered->points[i].z < min){
            min = cloud_filtered->points[i].z;
        }
    }

    float road_height = 2.5;
    for(int j=0;j<cloud_filtered->width; j++){

        if(cloud_filtered->points[j].z<min+road_height){
            cloud_road_filtered->points.push_back(cloud_filtered->points[j]);
        }
        else{
            cloud_vertical_filtered->points.push_back(cloud_filtered->points[j]);
        }
    }

    int bd[500][500]={0,};
    std::vector<std::pair<int, int> > Index;
    Index.clear();
    for(int k=0;k<cloud_vertical_filtered->points.size();k++){
        int xIndex=0, yIndex=0;
        xIndex = (int)(floor(cloud_vertical_filtered->points[k].x*5+0.5));
        yIndex = (int)(floor(cloud_vertical_filtered->points[k].y*5+0.5)) + 500;

        if(xIndex > 499 || yIndex > 499 || xIndex < 0 || yIndex < 0)
            continue;
        else{
            float z = cloud_vertical_filtered->points[k].z+min-road_height-0.5;
            if(z>0 && z<=1){
                bd[xIndex][yIndex] = bd[xIndex][yIndex] | 128;
                Index.emplace_back(std::make_pair(xIndex,yIndex));
            }
            else if(z >1 && z<=2){
                bd[xIndex][yIndex] = bd[xIndex][yIndex] | 64;
                Index.emplace_back(std::make_pair(xIndex,yIndex));
            }
            else if(z >2 && z<=3){
                bd[xIndex][yIndex] = bd[xIndex][yIndex] | 32;
                Index.emplace_back(std::make_pair(xIndex,yIndex));
            }
            else if(z >3 && z<=4){
                bd[xIndex][yIndex] = bd[xIndex][yIndex] | 16;
                Index.emplace_back(std::make_pair(xIndex,yIndex));
            }
            else if(z >4 && z<=5){
                bd[xIndex][yIndex] = bd[xIndex][yIndex] | 8;
                Index.emplace_back(std::make_pair(xIndex,yIndex));
            }
            else if(z >5 && z<=6){
                bd[xIndex][yIndex] = bd[xIndex][yIndex] | 4;
                Index.emplace_back(std::make_pair(xIndex,yIndex));
            }
            else if(z >6 && z<=7){
                bd[xIndex][yIndex] = bd[xIndex][yIndex] | 2;
                Index.emplace_back(std::make_pair(xIndex,yIndex));
            }
            else if(z >7 && z<=8){
                bd[xIndex][yIndex] = bd[xIndex][yIndex] | 1;
                Index.emplace_back(std::make_pair(xIndex,yIndex));
            }
        }
    }
    if(!Index.empty()){

        std::sort(Index.begin(),Index.end());

        for(int i=0;i<Index.size();i++){
            pcl::PointXYZI points;
            if(i==0){
                points.x = (Index[0].first)/5.0;
                points.y = (Index[0].second-500)/5.0;
                points.z = std::bitset<8>(bd[Index[0].first][Index[0].second]&255).count();
                points.intensity = bd[Index[0].first][Index[0].second];
                cloud_vertical2d_filtered->push_back(points);
            }
            else{
                if(Index[i-1]!=Index[i]){
                    points.x = (Index[i].first)/5.0;
                    points.y = (Index[i].second-500)/5.0;
                    points.z = std::bitset<8>(bd[Index[i].first][Index[i].second]&255).count();
                    points.intensity = bd[Index[i].first][Index[i].second];
                    cloud_vertical2d_filtered->push_back(points);
                }
            }
        }
    }


//    pcl::PCDWriter writer;
//    writer.write<pcl::PointXYZI> ("road_1.pcd", *cloud_1, false);

    sensor_msgs::PointCloud2Ptr pcl_ros (new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2Ptr pcl_ros_road_filtered (new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2Ptr pcl_ros_vertical_filtered (new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2Ptr pcl_ros_vertical2d_filtered (new sensor_msgs::PointCloud2);

    pcl::toROSMsg(*cloud, *pcl_ros);
    pcl::toROSMsg(*cloud_road_filtered, *pcl_ros_road_filtered);
    pcl::toROSMsg(*cloud_vertical_filtered, *pcl_ros_vertical_filtered);
    pcl::toROSMsg(*cloud_vertical2d_filtered, *pcl_ros_vertical2d_filtered);
    std::cout << "road filter done" << std::endl;

    pcl_ros->header.frame_id = "odom";
    pcl_ros->header.stamp = ros::Time::now();

    pcl_ros_road_filtered->header.frame_id = "odom";
    pcl_ros_road_filtered->header.stamp = ros::Time::now();

    pcl_ros_vertical_filtered->header.frame_id = "odom";
    pcl_ros_vertical_filtered->header.stamp = ros::Time::now();

    pcl_ros_vertical2d_filtered->header.frame_id = "odom";
    pcl_ros_vertical2d_filtered->header.stamp = ros::Time::now();

    ros::Publisher pcl_ros_pub = node.advertise<sensor_msgs::PointCloud2>("raw_pcd", 1);
    ros::Publisher pcl_ros_pub2 = node.advertise<sensor_msgs::PointCloud2>("road_filtered_pcd", 1);
    ros::Publisher pcl_ros_pub3 = node.advertise<sensor_msgs::PointCloud2>("vertical_filtered_pcd", 1);
    ros::Publisher pcl_ros_pub4 = node.advertise<sensor_msgs::PointCloud2>("vertical_filtered2d_pcd", 1);

    while(ros::ok()){
        pcl_ros_pub.publish(pcl_ros);
        pcl_ros_pub2.publish(pcl_ros_road_filtered);
        pcl_ros_pub3.publish(pcl_ros_vertical_filtered);
        pcl_ros_pub4.publish(pcl_ros_vertical2d_filtered);

        ros::Duration(5).sleep();
        std::cout << "pub" << std::endl;
    }

    return 0;
}
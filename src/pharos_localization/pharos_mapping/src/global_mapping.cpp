#include <stdlib.h>
#include <vector>
#include <queue>
#include <string>
#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetMapResponse.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <pcl_ros/transforms.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>

#include <boost/shared_ptr.hpp>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

const uint8_t OCC_FREE = 0;
const uint8_t OCC_FULL = 255;
const uint8_t OCC_UNKNOW = 255;
const uint8_t OCC_LANE = 201;
const uint8_t OCC_INCREMENT = 1;

const float resolution = 0.10f;
const float resolutionInverse = 1/resolution;

const int LoadinfoMapArraySizeX = 10000;
const int LoadinfoMapArraySizeY = 10000;

const int RoadinfoXOrigin = 5000;
const int RoadinfoYOrigin = 5000;
unsigned long RoadinfoMapArraySize = LoadinfoMapArraySizeX * LoadinfoMapArraySizeY;

const std::string fix_frame = "map";

nav_msgs::OccupancyGrid RoadinfoMap;
nav_msgs::OccupancyGrid VerticalinfoMap;
nav_msgs::OccupancyGrid CheckMap;

ros::Publisher RoadinfoMapPublisher;
ros::Publisher VerticalinfoMapPublisher;


ros::ServiceClient map_service_client;

tf::TransformListener * pListener = NULL;
tf::TransformListener * pListener2 = NULL;

typedef std::vector<int> mapping_log;
std::queue<boost::shared_ptr<mapping_log> > mapping_Q;
const int updateRate = 120.0;
double decay_time = 0.0;
int QSize = 3;
bool gaussian = false;

void GaussianMapping(unsigned int map_width, int xIndex, int yIndex){
    int IDXarray[25];
    float array[5][5] = {0,    0,     0.25,  0,     0,
                         0,    0.25,  0.5,   0.25,  0,
                         0.25, 0.5,   1,     0.5,   0.25,
                         0,    0.25,  0.5,   0.25,  0,
                         0,    0,     0.25,  0,     0,};
    int counter = 0;
    for (auto i=0;i<=2;i++) {
        for (auto j = 0; j <=4*i; ++j) {
            IDXarray[counter++] = MAP_IDX(map_width, xIndex + i, yIndex + j);
        }
    }
}

void updateRoadinfoMap_Pointcloud(const PointCloudConstPtr& globalCloud){

    float array[7][7] = {0,    0,    0,     0.1 ,  0,     0,    0,
                         0,    0,    0.1,   0.25,  0.1,   0,    0,
                         0,    0.1,  0.25,  0.5,   0.25,  0.1,  0,
                         0.1,  0.25, 0.5,   1,     0.5,   0.25, 0.1,
                         0,    0.1,  0.25,  0.5,   0.25,  0.1,  0,
                         0,    0,    0.1,   0.25,  0.1,   0,    0,
                         0,    0,    0,     0.1,   0,     0,    0};
    unsigned int Roadinfo_map_width = RoadinfoMap.info.width;
    unsigned long Roadinfo_map_array_size = RoadinfoMap.data.size();

    PointCloud::const_iterator iter = globalCloud->begin();
    boost::shared_ptr<mapping_log> log(new mapping_log);

    while(iter != globalCloud->end()){
        int xIndex, yIndex, mapIndex;

        xIndex = (int)((*iter).x * resolutionInverse)+RoadinfoXOrigin;
        yIndex = (int)((*iter).y * resolutionInverse)+RoadinfoYOrigin;

        CheckMap.data[MAP_IDX(Roadinfo_map_width,xIndex,yIndex)] += 1;

        if(CheckMap.data[MAP_IDX(Roadinfo_map_width,xIndex,yIndex)] > 3){

            CheckMap.data[MAP_IDX(Roadinfo_map_width,xIndex,yIndex)] = 20;

            if(!gaussian){
                mapIndex = MAP_IDX(Roadinfo_map_width,xIndex,yIndex);
                if(RoadinfoMap.data[mapIndex]==0)
                    RoadinfoMap.data[mapIndex] = (int)iter->intensity;
                else{
                    RoadinfoMap.data[mapIndex] = (RoadinfoMap.data[mapIndex] + (int)iter->intensity)/2;
                }
            }
            else{
                for(int i=0;i<7;i++){
                    for(int j=0;j<7;j++){
                        mapIndex = MAP_IDX(Roadinfo_map_width,xIndex+i-2,yIndex+j-2);
                        if(mapIndex >= 0 && mapIndex < Roadinfo_map_array_size){
                            int roaddata;
                            if(RoadinfoMap.data[mapIndex]<0)
                                roaddata = RoadinfoMap.data[mapIndex]+256;
                            else
                                roaddata = RoadinfoMap.data[mapIndex];
                            if(roaddata<(int)(255*array[i][j])){
                                RoadinfoMap.data[mapIndex] = (int)(255*array[i][j]);
                            }
                        }
                    }
                    log->push_back(mapIndex);
                }
            }


        }

        iter++;
    }
    RoadinfoMapPublisher.publish(RoadinfoMap);
}

void updateVerticalinfoMap_Pointcloud(const PointCloudConstPtr& globalCloud){

    unsigned int Roadinfo_map_width = RoadinfoMap.info.width;
    unsigned long Roadinfo_map_array_size = RoadinfoMap.data.size();

    PointCloud::const_iterator iter = globalCloud->begin();
    boost::shared_ptr<mapping_log> log(new mapping_log);

    while(iter != globalCloud->end()){
        int xIndex, yIndex;

        xIndex = (int)((*iter).x * resolutionInverse)+RoadinfoXOrigin;
        yIndex = (int)((*iter).y * resolutionInverse)+RoadinfoYOrigin;

        int mapIndex = MAP_IDX(Roadinfo_map_width,xIndex,yIndex);
        if(mapIndex >= 0 && mapIndex < Roadinfo_map_array_size){
//            std::cout << VerticalinfoMap.data[mapIndex] << std::endl;
//            printf("%d \n",(int)VerticalinfoMap.data[mapIndex]);
            VerticalinfoMap.data[mapIndex] = VerticalinfoMap.data[mapIndex] | (int)iter->intensity;
//            printf("%d  %d \n\n",(int)VerticalinfoMap.data[mapIndex],(int)iter->intensity);


//            if(VerticalinfoMap.data[mapIndex] > 30){
//                VerticalinfoMap.data[mapIndex] = 255;
//                VerticalinfoMap.data[mapIndex] = 200;
//            }

            log->push_back(mapIndex);
        }
        iter++;
    }
    mapping_Q.push(log);
    VerticalinfoMapPublisher.publish(VerticalinfoMap);
}

PointCloudPtr ObstacleCloudFiltering(const sensor_msgs::PointCloud2ConstPtr &input_pc, tf::TransformListener *plistener){
    PointCloudPtr obstacleRaw(new PointCloud);
    PointCloudPtr obstacleFiltered(new PointCloud);
    PointCloudPtr obstacleTransformed(new PointCloud);

    pcl::fromROSMsg(*input_pc,*obstacleRaw);
    tf::StampedTransform transform;
    try{
        plistener->lookupTransform(fix_frame,input_pc->header.frame_id,input_pc->header.stamp,transform);
    }catch(tf::LookupException& e){
        ROS_ERROR("%s",e.what());
        return obstacleTransformed;
    }
    catch(tf::ConnectivityException& e){
        ROS_ERROR("%s",e.what());
        return obstacleTransformed;
    }catch(tf::ExtrapolationException& e){
        ROS_ERROR("%s",e.what());
        return obstacleTransformed;
    }

    pcl_ros::transformPointCloud(*obstacleRaw,*obstacleTransformed,transform);

    return obstacleTransformed;
}

void RoadinfoCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg){
    updateRoadinfoMap_Pointcloud(ObstacleCloudFiltering(msg, pListener));
}
void VerticalinfoCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg){
    updateVerticalinfoMap_Pointcloud(ObstacleCloudFiltering(msg, pListener2));
}


void initRoadinfoMap(void){
    RoadinfoMap.header.frame_id = fix_frame;
    RoadinfoMap.data.resize(RoadinfoMapArraySize,OCC_FREE);
    RoadinfoMap.info.width = LoadinfoMapArraySizeX;
    RoadinfoMap.info.height = LoadinfoMapArraySizeY;
    RoadinfoMap.info.origin.position.x = 0;
    RoadinfoMap.info.origin.position.y = 0;
    RoadinfoMap.info.origin.orientation.w = 1.0;
    RoadinfoMap.info.origin.orientation.x = 0.0;
    RoadinfoMap.info.origin.orientation.y = 0.0;
    RoadinfoMap.info.origin.orientation.z = 0.0;
    RoadinfoMap.info.resolution = resolution;

    CheckMap.header.frame_id = fix_frame;
    CheckMap.data.resize(RoadinfoMapArraySize,OCC_FREE);
    CheckMap.info.width = LoadinfoMapArraySizeX;
    CheckMap.info.height = LoadinfoMapArraySizeY;
    CheckMap.info.origin.position.x = 0;
    CheckMap.info.origin.position.y = 0;
    CheckMap.info.origin.orientation.w = 1.0;
    CheckMap.info.origin.orientation.x = 0.0;
    CheckMap.info.origin.orientation.y = 0.0;
    CheckMap.info.origin.orientation.z = 0.0;
    CheckMap.info.resolution = resolution;

    printf("GlobalMap Initialize\n");
}
void initVerticalinfoMap(void){
    VerticalinfoMap.header.frame_id = fix_frame;
    VerticalinfoMap.data.resize(RoadinfoMapArraySize,OCC_FREE);
    VerticalinfoMap.info.width = LoadinfoMapArraySizeX;
    VerticalinfoMap.info.height = LoadinfoMapArraySizeY;
    VerticalinfoMap.info.origin.position.x = 0;
    VerticalinfoMap.info.origin.position.y = 0;
    VerticalinfoMap.info.origin.orientation.w = 1.0;
    VerticalinfoMap.info.origin.orientation.x = 0.0;
    VerticalinfoMap.info.origin.orientation.y = 0.0;
    VerticalinfoMap.info.origin.orientation.z = 0.0;
    VerticalinfoMap.info.resolution = resolution;


    printf("GlobalMap Initialize\n");
}

void mapsave(nav_msgs::OccupancyGrid map, std::string mapname)
{
    printf("Received a %d X %d map @ %.3f m/pix\n",
             map.info.width,
             map.info.height,
             map.info.resolution);


    std::string mapdatafile = mapname + ".pgm";
    printf("Writing map occupancy data to %s\n", mapdatafile.c_str());
    FILE* out = fopen(mapdatafile.c_str(), "w");
    if (!out)
    {
        printf("Couldn't save map file to %s", mapdatafile.c_str());
        return;
    }

    fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
            map.info.resolution, map.info.width, map.info.height);
    for(unsigned int y = 0; y < map.info.height; y++) {
        for(unsigned int x = 0; x < map.info.width; x++) {
            unsigned int i = x + (map.info.height - y - 1) * map.info.width;
//          if (map->data[i] == 0) { //occ [0,0.1)
//            fputc(254, out);
//          } else if (map->data[i] == +100) { //occ (0.65,1]
//            fputc(000, out);
//          } else { //occ [0.1,0.65]
//            fputc(205, out);
//          }
            fputc(map.data[i],out);
        }
    }

    fclose(out);

    std::string mapmetadatafile = mapname + ".yaml";
    printf("Writing map occupancy data to %s\n", mapmetadatafile.c_str());
    FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


    /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

     */

    geometry_msgs::Quaternion orientation = map.info.origin.orientation;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
            mapdatafile.c_str(), map.info.resolution, map.info.origin.position.x, map.info.origin.position.y, yaw);

    fclose(yaml);

    printf("%s Done\n\n",mapname.c_str());
}

int main(int argc, char** argv){
    ros::init(argc,argv,"global_mapping_node");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    p_nh.param<double>("decay_time",decay_time,2.0);

    if(decay_time > 10.0){
        ROS_WARN("max decay time is 10 sec. force to decay_time = 10.0");
        decay_time = 10.0;
    }

    QSize = decay_time * updateRate;

    tf::TransformListener listener;
    tf::TransformListener listener2;

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_roadinfo_subscriber;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_verticalinfo_subscriber;


    tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter_bumper;
    tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter_bumper2;

    cloud_roadinfo_subscriber.subscribe(nh,"/cloud_road",10000);
    cloud_verticalinfo_subscriber.subscribe(nh, "/cloud_verticalinfo_2d",10000);


    tf_filter_bumper = new tf::MessageFilter<sensor_msgs::PointCloud2>(cloud_roadinfo_subscriber,listener,fix_frame,10000);
    tf_filter_bumper->registerCallback(boost::bind(RoadinfoCloudCallback,_1));

    tf_filter_bumper2 = new tf::MessageFilter<sensor_msgs::PointCloud2>(cloud_verticalinfo_subscriber,listener2,fix_frame,10000);
    tf_filter_bumper2->registerCallback(boost::bind(VerticalinfoCloudCallback,_1));

    pListener = &listener;
    pListener2 = &listener2;

    RoadinfoMapPublisher = nh.advertise<nav_msgs::OccupancyGrid>("map",10000);
    VerticalinfoMapPublisher = nh.advertise<nav_msgs::OccupancyGrid>("map2",10000);
    map_service_client = nh.serviceClient<nav_msgs::GetMap>("static_map");

    initRoadinfoMap();
    initVerticalinfoMap();

    ros::spin();

    std::cout <<" aaaa" << std::endl;

    mapsave(RoadinfoMap, "road");
    mapsave(VerticalinfoMap, "vertical");

    return 1;
}

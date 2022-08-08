#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

const uint8_t OCC_FREE = 0;
const uint8_t OCC_FULL = 250;

const float resolution = 0.4;
const float resolutionInverse = 1/resolution;

const int globalMapArraySizeX = 10000;
const int globalMapArraySizeY = 10000;
const int globalXOrigin = globalMapArraySizeX/2;
const int globalYOrigin = globalMapArraySizeY/2;
const int globalMapArraySize = globalMapArraySizeX * globalMapArraySizeY;

const int cropSize = 300;
const int cropMapArraySize = cropSize*cropSize;


class ObstacleMapping
{
public:
    int32_t publish_rate_;
    ros::NodeHandlePtr node_;
    ros::NodeHandlePtr pnode_;

    ros::Publisher obstacle_map_pub_;
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber vehicle_odom_sub_;

    nav_msgs::OccupancyGrid globalObstacleMap_;

    void VehicleOdomCallback(const nav_msgs::OdometryConstPtr &msg){
        ros::Rate r(15);
        //return;
    //  printf("Callback\n");
    //    tf::StampedTransform transform;
    //    try{
    //        pListener->lookupTransform("/odom","/base_footprint",msg->header.stamp,transform);
    //        //pListener->lookupTransform("/odom","/base_footprint",ros::Time(0),transform);
    //        //pListener->lookupTransform("/odom",obstacleRaw->header.frame_id,ros::Time(0),transform);
    //    }catch(tf::LookupException& e){
    //        ROS_ERROR("%s",e.what());
    //        return;
    //    }
    //    catch(tf::ConnectivityException& e){
    //        ROS_ERROR("%s",e.what());
    //        return;
    //    }catch(tf::ExtrapolationException& e){
    //        ROS_ERROR("%s",e.what());
    //        return;
    //    }
    //    tf::Vector3 vehiclePos = transform.getOrigin();
    //    printf("local map publish\n");

        double vehicle_pose_x = msg->pose.pose.position.x;
        double vehicle_pose_y = msg->pose.pose.position.y;

        nav_msgs::OccupancyGridPtr localObstacleMap(new nav_msgs::OccupancyGrid);
        cropMap(*localObstacleMap,vehicle_pose_x,vehicle_pose_y);

        localObstacleMap->header.stamp = msg->header.stamp;
        obstacle_map_pub_.publish(localObstacleMap);
        r.sleep();
    }


    void ObstaclePointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
        
    }
  
    void cropMap(nav_msgs::OccupancyGrid& mapCropped, const double vehiclePosX, const double vehiclePosY){
    

        mapCropped.info.map_load_time = globalObstacleMap_.info.map_load_time;
        mapCropped.info.resolution = globalObstacleMap_.info.resolution;
        mapCropped.header.frame_id = "odom";
        mapCropped.info.width = mapCropped.info.height = cropSize;
        //origin :  The 2-D pose of the lower-left pixel in the map
        mapCropped.info.origin.position.x = vehiclePosX - cropSize * 0.5 * globalObstacleMap_.info.resolution;
        mapCropped.info.origin.position.y = vehiclePosY - cropSize * 0.5 * globalObstacleMap_.info.resolution;

        mapCropped.data.resize(cropMapArraySize);

        int global_x_origin = globalObstacleMap_.info.width/2;
        int global_y_origin = globalObstacleMap_.info.height/2;

        double global_map_position_x = globalObstacleMap_.info.origin.position.x;
        double global_map_position_y = globalObstacleMap_.info.origin.position.y;


        int int_vehicle_x = (int)((vehiclePosX - global_map_position_x)/globalObstacleMap_.info.resolution);
        int int_vehicle_y = (int)((vehiclePosY - global_map_position_y)/globalObstacleMap_.info.resolution);
       
        int topY = int_vehicle_y + cropSize*0.5;
        int bottomY = topY - cropSize;
        int leftX = int_vehicle_x - cropSize*0.5;


        int localYCount = 0;

        // for(int yCount = bottomY; yCount < topY;yCount++,localYCount++){
        //     memcpy(&mapCropped.data[MAP_IDX(cropSize,0,localYCount)],&globalObstacleMap_.data[MAP_IDX(globalObstacleMap_.info.width,leftX,yCount)],cropSize*sizeof(mapCropped.data[0]));
        // }

       
    //    for(int i=0; i<mapCropped.data.size();i++){
    //        mapCropped.data.at(i) = 200;
    //    }


        //map refinement.
        for(int i=cropMapArraySize -1; i>=0; i--){
            if(mapCropped.data[i] != 0)
                mapCropped.data[i] = OCC_FULL;
            else{
                mapCropped.data[i] = 0;
            }
        }
    }

    void initGlobalObstacleMap(void){
        globalObstacleMap_.header.frame_id = "odom";
        globalObstacleMap_.data.resize(globalMapArraySize,OCC_FREE);
        globalObstacleMap_.info.width = globalMapArraySizeX;
        globalObstacleMap_.info.height = globalMapArraySizeY;
        globalObstacleMap_.info.origin.position.x = globalXOrigin*resolution;
        globalObstacleMap_.info.origin.position.y = globalYOrigin*resolution;
        globalObstacleMap_.info.origin.orientation.w = 1.0;
        globalObstacleMap_.info.origin.orientation.x = 0.0;
        globalObstacleMap_.info.origin.orientation.y = 0.0;
        globalObstacleMap_.info.origin.orientation.z = 0.0;
        globalObstacleMap_.info.resolution = resolution;

        printf("GlobalMap Initialize\n");
    }

    // void initGlobalObstacleMap_new(void){

    //     nav_msgs::GetMap srv_map;

    //     while (!ros::service::waitForService("static_map", ros::Duration(3.0))) {
    //         ROS_INFO("Waiting for service static_map to become available");
    //     }

    //     if (map_service_client.call(srv_map))
    //     {
    //         ROS_INFO("Map service called successfully");
    //         globalObstacleMap_new = srv_map.response.map;

    //         //const nav_msgs::OccupancyGrid& map (res);
    //         //do something with the map
    //     }
    //     else
    //     {
    //         ROS_ERROR("Failed to call map service");
    //         return;
    //     }
    // }

    int init()
    {
        publish_rate_ = 10;
        node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        obstacle_map_pub_ = node_->advertise<nav_msgs::OccupancyGrid>("/obstacle_map", 10);

        pointcloud_sub_ = node_->subscribe<sensor_msgs::PointCloud2>("/pointcloud/obstacles",10, &ObstacleMapping::ObstaclePointCloudCallback, this);
        vehicle_odom_sub_ = node_->subscribe("/odom/vehicle",1, &ObstacleMapping::VehicleOdomCallback,this);

        initGlobalObstacleMap();
        return 0;
    }

    // Publish data
    void publish()
    {
        ros::Rate loop_rate(publish_rate_);

        while (node_->ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_mapping_node");

    ObstacleMapping obstacle_mapping_class;
    if (obstacle_mapping_class.init())
    {
        ROS_FATAL("obstacle_mapping_node initialization failed");
        return -1;
    }
    obstacle_mapping_class.publish();

    return 0;
}


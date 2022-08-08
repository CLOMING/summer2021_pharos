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

#define vehicle_width 1.8f
#define vehicle_length 4.7f
#define vehicle_wheel_base 2.7f

const uint8_t OCC_FREE = 0;
const uint8_t OCC_FULL = 250;
const uint8_t OCC_UNKNOW = 255;
const uint8_t OCC_LANE = 201;
const uint8_t OCC_INCREMENT = 50;

const float resolution = 0.40f;
const float resolutionInverse = 1/resolution;

const int globalMapArraySizeX = 10000;
const int globalMapArraySizeY = 10000;
const int globalXOrigin = globalMapArraySizeX/2;
const int globalYOrigin = globalMapArraySizeY/2;
const int globalMapArraySize = globalMapArraySizeX * globalMapArraySizeY;

const int cropSize = 300;
const int cropMapArraySize = cropSize*cropSize;
const int cropXOrigin = cropSize/2;
const int cropYOrigin = cropSize/2;

const int originPosition = cropSize/2;

nav_msgs::OccupancyGrid globalObstacleMap;
nav_msgs::OccupancyGrid globalObstacleMap_new;
nav_msgs::OccupancyGridPtr localObstacleMap(new nav_msgs::OccupancyGrid);

ros::Publisher obstacleMapPublisher;

ros::Subscriber tracked_vehicle_info_subscriber;

ros::Subscriber vehicle_odom_sub_;


ros::Subscriber bumper_lidar_sub_;
ros::ServiceClient map_service_client;

tf::TransformListener * pListener = NULL;

typedef std::vector<int> mapping_log;
std::queue<boost::shared_ptr<mapping_log> > mapping_Q;
const int updateRate = 10.0;
double decay_time = 0.0;
int QSize = 3;


void cropMap_new(nav_msgs::OccupancyGrid& mapCropped, const double vehiclePosX, const double vehiclePosY){
//    printf("height: %d\n",globalObstacleMap_new.info.height);
//    printf("widht: %d\n",globalObstacleMap_new.info.width);
//
//    printf("resolution: %.3f\n",globalObstacleMap_new.info.resolution);

    mapCropped.info.map_load_time = globalObstacleMap_new.info.map_load_time;
    mapCropped.info.resolution = globalObstacleMap_new.info.resolution;
    mapCropped.header.frame_id = "odom";
    mapCropped.info.width = mapCropped.info.height = cropSize;
    //origin :  The 2-D pose of the lower-left pixel in the map
    mapCropped.info.origin.position.x = vehiclePosX - cropSize * 0.5 * globalObstacleMap_new.info.resolution;
    mapCropped.info.origin.position.y = vehiclePosY - cropSize * 0.5 * globalObstacleMap_new.info.resolution;

    mapCropped.data.resize(cropMapArraySize);

    int global_x_origin = globalObstacleMap_new.info.width/2;
    int global_y_origin = globalObstacleMap_new.info.height/2;

    double global_map_position_x = globalObstacleMap_new.info.origin.position.x;
    double global_map_position_y = globalObstacleMap_new.info.origin.position.y;


    int int_vehicle_x = (int)((vehiclePosX - global_map_position_x)/globalObstacleMap_new.info.resolution);
    int int_vehicle_y = (int)((vehiclePosY - global_map_position_y)/globalObstacleMap_new.info.resolution);
//    printf("int vehicle_x: %d, int vehicle_y: %d\n",int_vehicle_x, int_vehicle_y);
//    printf("global_x_origin: %d, global_y_origin: %d\n",global_map_position_x, global_map_position_y);

    int topY = int_vehicle_y + cropSize*0.5;
    int bottomY = topY - cropSize;
    int leftX = int_vehicle_x - cropSize*0.5;


    int localYCount = 0;

    for(int yCount = bottomY; yCount < topY;yCount++,localYCount++){
        memcpy(&mapCropped.data[MAP_IDX(cropSize,0,localYCount)],&globalObstacleMap_new.data[MAP_IDX(globalObstacleMap_new.info.width,leftX,yCount)],cropSize*sizeof(mapCropped.data[0]));
    }

//    printf("test2\n");
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

void cropMap(nav_msgs::OccupancyGrid& mapCropped, const double vehiclePosX, const double vehiclePosY){
    mapCropped.info.map_load_time = globalObstacleMap.info.map_load_time;
	mapCropped.info.resolution = resolution;
	mapCropped.header.frame_id = "odom";
	mapCropped.info.width = mapCropped.info.height = cropSize;
	//origin :  The 2-D pose of the lower-left pixel in the map
	mapCropped.info.origin.position.x = vehiclePosX - cropSize * 0.5 * resolution;
	mapCropped.info.origin.position.y = vehiclePosY - cropSize * 0.5 * resolution;

	mapCropped.data.resize(cropMapArraySize);


	int topY = floor((vehiclePosY * resolutionInverse) + globalYOrigin + cropSize * 0.5+0.5);
	int bottomY = topY - cropSize;
	int leftX = floor((vehiclePosX * resolutionInverse) + globalXOrigin - cropSize * 0.5+0.5);

	int localYCount = 0;
	for(int yCount = bottomY; yCount < topY;yCount++,localYCount++){
		memcpy(&mapCropped.data[MAP_IDX(cropSize,0,localYCount)],&globalObstacleMap.data[MAP_IDX(globalMapArraySizeX,leftX,yCount)],cropSize*sizeof(mapCropped.data[0]));
	}


	//map refinement.
	for(int i=cropMapArraySize -1; i>=0; i--){
		if(mapCropped.data[i] > 2)
			mapCropped.data[i] = OCC_FULL;
        else{
            mapCropped.data[i] = 0;
        }
	}
}
void UpdateMap(PointCloudPtr obstacleTransformed){

    PointCloud::const_iterator iter = obstacleTransformed->begin();
    boost::shared_ptr<mapping_log> log(new mapping_log);


//    if(mapping_Q.size() == QSize){
//        std::cout << "aaaaa" <<std::endl;
//        boost::shared_ptr<mapping_log> past_log = mapping_Q.front();
//        mapping_Q.pop();
//        mapping_log::iterator iter = past_log->begin();
//        while(iter != past_log->end()){
//            globalObstacleMap.data[*iter] -= 1;
//            iter++;
//        }
//    }

    while(iter != obstacleTransformed->end()){
        int xIndex, yIndex;

        xIndex = (int)((*iter).x * resolutionInverse)+globalXOrigin;
        yIndex = (int)((*iter).y * resolutionInverse)+globalYOrigin;

        int mapIndex = MAP_IDX(globalMapArraySizeX,xIndex,yIndex);
        if(mapIndex >= 0 && mapIndex < globalMapArraySize){
            globalObstacleMap.data[mapIndex] += 1;
            if(globalObstacleMap.data[mapIndex] >3)
                globalObstacleMap.data[mapIndex] = 3;
//            if(VerticalinfoMap.data[mapIndex] > 30){
//                VerticalinfoMap.data[mapIndex] = 255;
//                VerticalinfoMap.data[mapIndex] = 200;
//            }

            log->push_back(mapIndex);
        }
        iter++;
    }
    mapping_Q.push(log);
}

void obstacleCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
	PointCloudPtr obstacleRaw(new PointCloud);
	PointCloudPtr obstacleFiltered(new PointCloud);
	PointCloudPtr obstacleTransformed(new PointCloud);

	pcl::fromROSMsg(*msg,*obstacleRaw);

	tf::StampedTransform transform;
	try{
		pListener->lookupTransform("/odom",obstacleRaw->header.frame_id,msg->header.stamp,transform);
		//pListener->lookupTransform("/odom",obstacleRaw->header.frame_id,ros::Time(0),transform);
	}catch(tf::LookupException& e){
		ROS_ERROR("%s",e.what());
		return;
	}
	catch(tf::ConnectivityException& e){
		ROS_ERROR("%s",e.what());
		return;
	}catch(tf::ExtrapolationException& e){
		ROS_ERROR("%s",e.what());
		return;
	}

//	pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
//	sor.setInputCloud(obstacleRaw);
//	sor.setMeanK(5);
//	sor.setStddevMulThresh(1.0);
//	sor.filter(*obstacleFiltered);
//
	pcl_ros::transformPointCloud(*obstacleRaw,*obstacleTransformed,transform);
	UpdateMap(obstacleTransformed);

	tf::Vector3 vehiclePos = transform.getOrigin();

    cropMap(*localObstacleMap,vehiclePos.x(),vehiclePos.y());
    obstacleMapPublisher.publish(localObstacleMap);
}


void VehicleOdomCallback(const nav_msgs::OdometryConstPtr &msg){
    return;
    printf("Callback\n");
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
    //cropMap_new(*localObstacleMap,vehiclePos.x(),vehiclePos.y());
    cropMap(*localObstacleMap,vehicle_pose_x,vehicle_pose_y);
    obstacleMapPublisher.publish(localObstacleMap);

}


void initGlobalObstacleMap(void){
	globalObstacleMap.header.frame_id = "odom";
	globalObstacleMap.data.resize(globalMapArraySize,OCC_FREE);
	globalObstacleMap.info.width = globalMapArraySizeX;
	globalObstacleMap.info.height = globalMapArraySizeY;
	globalObstacleMap.info.origin.position.x = globalXOrigin*resolution;
	globalObstacleMap.info.origin.position.y = globalYOrigin*resolution;
	globalObstacleMap.info.origin.orientation.w = 1.0;
	globalObstacleMap.info.origin.orientation.x = 0.0;
	globalObstacleMap.info.origin.orientation.y = 0.0;
	globalObstacleMap.info.origin.orientation.z = 0.0;
	globalObstacleMap.info.resolution = resolution;

    printf("GlobalMap Initialize\n");
}
void initLocalObstacleMap(void){
    localObstacleMap->header.frame_id = "odom";
    localObstacleMap->data.resize(cropMapArraySize,OCC_FREE);
    localObstacleMap->info.width = cropSize;
    localObstacleMap->info.height = cropSize;
    localObstacleMap->info.origin.position.x = cropXOrigin*resolution;
    localObstacleMap->info.origin.position.y = cropXOrigin*resolution;
    localObstacleMap->info.origin.orientation.w = 1.0;
    localObstacleMap->info.origin.orientation.x = 0.0;
    localObstacleMap->info.origin.orientation.y = 0.0;
    localObstacleMap->info.origin.orientation.z = 0.0;
    localObstacleMap->info.resolution = resolution;

    printf("LocallMap Initialize\n");
}
void initGlobalObstacleMap_new(void){

    nav_msgs::GetMap srv_map;

    while (!ros::service::waitForService("verticalinfo_static_map", ros::Duration(3.0))) {
        ROS_INFO("Waiting for service static_map to become available");
    }

    if (map_service_client.call(srv_map))
    {
        ROS_INFO("Map service called successfully");
        globalObstacleMap_new = srv_map.response.map;

        //const nav_msgs::OccupancyGrid& map (res);
        //do something with the map
    }
    else
    {
        ROS_ERROR("Failed to call map service");
        return;
    }

}


int main(int argc, char** argv){
	ros::init(argc,argv,"time_accum_map_node");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");

	p_nh.param<double>("decay_time",decay_time,2.0);

	if(decay_time > 10.0){
		ROS_WARN("max decay time is 10 sec. force to decay_time = 10.0");
		decay_time = 10.0;
	}

	QSize = decay_time * updateRate;
    initGlobalObstacleMap();
    initLocalObstacleMap();
	tf::TransformListener listener;
	message_filters::Subscriber<sensor_msgs::PointCloud2> obstacle_subscriber;
	tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter;

	//obstacle_subscriber.subscribe(nh,"velodyneOtherPoints",15);
	obstacle_subscriber.subscribe(nh,"/double_cluster",30);
	tf_filter = new tf::MessageFilter<sensor_msgs::PointCloud2>(obstacle_subscriber,listener,"odom",15);
	tf_filter->registerCallback(boost::bind(obstacleCloudCallback,_1));
	pListener = &listener;

	obstacleMapPublisher = nh.advertise<nav_msgs::OccupancyGrid>("obstacleMap",20);

    vehicle_odom_sub_ = nh.subscribe("/MCL_odom",10,VehicleOdomCallback);

    map_service_client = nh.serviceClient<nav_msgs::GetMap>("verticalinfo_static_map");


    initGlobalObstacleMap_new();

    ros::spin();

	return 1;
}



#include <stdlib.h>
#include <vector>
#include <queue>
#include <string>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetMapResponse.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
// #include <pcl-1.6/pcl/ros/conversions.h>
#include <pcl/conversions.h>
#include <std_msgs/Int32.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <pcl_ros/transforms.h>

#include <boost/shared_ptr.hpp>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

const uint8_t OCC_FREE = 0;
const uint8_t OCC_FULL = 200;
const uint8_t OCC_UNKNOW = 255;
const uint8_t OCC_LANE = 201;
const uint8_t OCC_INCREMENT = 1;

const float resolution = 0.1f;
const float resolutionInverse = 1/resolution;

const int globalMapArraySizeX = 25000;
const int globalMapArraySizeY = 25000;
const int globalXOrigin = globalMapArraySizeX/2;
const int globalYOrigin = globalMapArraySizeY/2;
const int globalMapArraySize = globalMapArraySizeX * globalMapArraySizeY;

const int cropSize = 1000;
const int cropMapArraySize = cropSize*cropSize;

const int originPosition = cropSize/2;

nav_msgs::OccupancyGrid globalObstacleMap;
nav_msgs::OccupancyGrid globalObstacleMap_new;
ros::Publisher obstacleMapPublisher;

tf::TransformListener * pListener = NULL;

ros::ServiceClient map_service_client;
ros::Subscriber vehicle_odom_sub_;
ros::Subscriber global_map_sub_;
ros::Subscriber tracked_object_sub_;
ros::Subscriber Mission_state_sub_;

typedef std::vector<int> mapping_log;
std::queue<boost::shared_ptr<mapping_log> > mapping_Q;
const int updateRate = 10.0;
double decay_time = 0.0;
int QSize = 10;

bool map_get = false;

std_msgs::Int32 mission_state_;

void updateMap(const PointCloudConstPtr& globalCloud){
	if(mapping_Q.size() == QSize){
		boost::shared_ptr<mapping_log> past_log = mapping_Q.front();
		mapping_Q.pop();
		mapping_log::iterator iter = past_log->begin();
		while(iter != past_log->end()){
			globalObstacleMap.data[*iter] -= OCC_INCREMENT;
			iter++;
		}
	}

	std::vector<bool> map_indexes;
	map_indexes.resize(globalMapArraySizeX*globalMapArraySizeY);

	PointCloud::const_iterator iter = globalCloud->begin();
	boost::shared_ptr<mapping_log> log(new mapping_log);

	while(iter != globalCloud->end()){
		int xIndex = (int)((*iter).x * resolutionInverse)+globalXOrigin;
		int yIndex = (int)((*iter).y * resolutionInverse)+globalYOrigin;
		int mapIndex = MAP_IDX(globalMapArraySizeX,xIndex,yIndex);
		if(mapIndex >= 0 && mapIndex < globalMapArraySize){
			if(map_indexes.at(mapIndex)){

			}
			else{
				if(globalObstacleMap.data[mapIndex] <= 255){
					globalObstacleMap.data[mapIndex] += OCC_INCREMENT;
				}
				log->push_back(mapIndex);
				map_indexes.at(mapIndex) = true;
			}

		}
		iter++;
	}
	mapping_Q.push(log);
}
void cropMap_new(nav_msgs::OccupancyGrid& mapCropped, const double vehiclePosX, const double vehiclePosY){
   // printf("height: %d\n",globalObstacleMap_new.info.height);
   // printf("widht: %d\n",globalObstacleMap_new.info.width);
   // printf("resolution: %.3f\n",globalObstacleMap_new.info.resolution);

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
	// for(int i=cropMapArraySize -1; i>=0; i--){
	// 	if(mapCropped.data[i] > 1)
	// 		mapCropped.data[i] = OCC_FULL;   
	// 	// mapCropped.data[i] = OCC_UNKNOW
	// }

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
	int topY = (int)(vehiclePosY * resolutionInverse) + globalYOrigin + cropSize * 0.5;
	int bottomY = topY - cropSize;
	int leftX = (int)(vehiclePosX * resolutionInverse) + globalXOrigin - cropSize * 0.5;

	int localYCount = 0;
	for(int yCount = bottomY; yCount < topY;yCount++,localYCount++){
		memcpy(&mapCropped.data[MAP_IDX(cropSize,0,localYCount)],&globalObstacleMap.data[MAP_IDX(globalMapArraySizeX,leftX,yCount)],cropSize*sizeof(mapCropped.data[0]));
	}

	//map refinement.
	// for(int i=cropMapArraySize -1; i>=0; i--){
	// 	if(mapCropped.data[i] > 3)
	// 		mapCropped.data[i] = OCC_FULL;
	// }
}
void updateMap_Pointcloud(const PointCloudConstPtr& globalCloud){
    double resolution_new = globalObstacleMap_new.info.resolution;
    double resolutionInverse_new = 1/resolution_new;
    unsigned int global_map_width = globalObstacleMap_new.info.width;
    unsigned int global_map_height = globalObstacleMap_new.info.height;
    unsigned int global_map_array_size = globalObstacleMap_new.data.size();
    unsigned int global_map_x_origin = global_map_width/2;
    unsigned int global_map_y_origin = global_map_height/2;
    double global_map_position_x = globalObstacleMap_new.info.origin.position.x;
    double global_map_position_y = globalObstacleMap_new.info.origin.position.y;

    // global_value_map_width = global_map_width;

    if(mapping_Q.size() == QSize){
        boost::shared_ptr<mapping_log> past_log = mapping_Q.front();
        mapping_Q.pop();
        mapping_log::iterator iter = past_log->begin();
        while(iter != past_log->end()){
            globalObstacleMap_new.data[*iter] -= OCC_INCREMENT;
            iter++;
        }
    }

    PointCloud::const_iterator iter = globalCloud->begin();
    boost::shared_ptr<mapping_log> log(new mapping_log);

    while(iter != globalCloud->end()){
        int xIndex, yIndex;

        xIndex = (int)(((*iter).x-globalObstacleMap_new.info.origin.position.x) * resolutionInverse_new);
        yIndex = (int)(((*iter).y-globalObstacleMap_new.info.origin.position.y) * resolutionInverse_new);
        // if(speedium_map){
        //     xIndex = (int)(((*iter).x-globalObstacleMap_new.info.origin.position.x) * resolutionInverse_new);
        //     yIndex = (int)(((*iter).y-globalObstacleMap_new.info.origin.position.y) * resolutionInverse_new);
        // }
        // else{
        //     xIndex = (int)((*iter).x * resolutionInverse)+globalXOrigin;
        //     yIndex = (int)((*iter).y * resolutionInverse)+globalYOrigin;
        // }


        int mapIndex = MAP_IDX(global_map_width,xIndex,yIndex);
        if(mapIndex >= 0 && mapIndex < global_map_array_size){
            if(globalObstacleMap_new.data[mapIndex] < 255){

                globalObstacleMap_new.data[mapIndex] += OCC_INCREMENT;
            }
            log->push_back(mapIndex);
        }
        iter++;
    }
    mapping_Q.push(log);
}

void ObstacleCloudFiltering(const sensor_msgs::PointCloud2ConstPtr &input_pc){
    PointCloudPtr obstacleRaw(new PointCloud);
    PointCloudPtr obstacleFiltered(new PointCloud);
    PointCloudPtr obstacleTransformed(new PointCloud);

    pcl::fromROSMsg(*input_pc,*obstacleRaw);
    
    //TF_map to odom
    tf::StampedTransform transform;
    try{
    	pListener->lookupTransform("odom",obstacleRaw->header.frame_id,input_pc->header.stamp,transform);
    }
    catch(tf::LookupException& e){
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
    if(obstacleRaw->points.empty()) return;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(obstacleRaw);
    sor.setMeanK(10);
    sor.setStddevMulThresh(0.8);
    sor.filter(*obstacleFiltered);

    pcl_ros::transformPointCloud(*obstacleFiltered,*obstacleTransformed,transform);
    pcl_ros::transformPointCloud(*obstacleRaw,*obstacleTransformed,transform);
    updateMap_Pointcloud(obstacleTransformed);
    // TF
    updateMap_Pointcloud(obstacleRaw);
}

void obstacleCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){	
	ObstacleCloudFiltering(msg);

	return;

	PointCloudPtr obstacleRaw(new PointCloud);
	PointCloudPtr obstacleFiltered(new PointCloud);
	PointCloudPtr obstacleTransformed(new PointCloud);

	pcl::fromROSMsg(*msg,*obstacleRaw);
	// tf::TransformListener pListener;

	tf::StampedTransform transform;
	try{
		// pListener->lookupTransform("odom",obstacleRaw->header.frame_id,obstacleRaw->header.stamp,transform);
		pListener->lookupTransform("odom",obstacleRaw->header.frame_id,ros::Time(0),transform);
		// pListener->lookupTransform("odom","velodyne",ros::Time::now(),transform);
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

	pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
	sor.setInputCloud(obstacleRaw);
	sor.setMeanK(20);
	sor.setStddevMulThresh(1.0);
	sor.filter(*obstacleFiltered);

	pcl_ros::transformPointCloud(*obstacleFiltered,*obstacleTransformed,transform);
	// pcl_ros::transformPointCloud(*obstacleRaw,*obstacleTransformed,transform);
	updateMap(obstacleTransformed);

	tf::Vector3 vehiclePos = transform.getOrigin();
	nav_msgs::OccupancyGridPtr localObstacleMap(new nav_msgs::OccupancyGrid);
	cropMap(*localObstacleMap,vehiclePos[0],vehiclePos[1]);

	obstacleMapPublisher.publish(localObstacleMap);
}

void MissionStateCallback(const std_msgs::Int32ConstPtr &msg)
{
    mission_state_ = *msg;
}

void VehicleOdomCallback(const nav_msgs::OdometryConstPtr &msg){

	// return;

	if(!map_get) return;

	ros::Rate r(15);
	//return;
//	printf("Callback\n");
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
	cropMap_new(*localObstacleMap,vehicle_pose_x,vehicle_pose_y);
	//cropMap(*localObstacleMap,vehicle_pose_x,vehicle_pose_y);
	obstacleMapPublisher.publish(localObstacleMap);
	// printf("obstacle_map publish\n");
	r.sleep();
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
}

void initGlobalObstacleMap_new(void){

	nav_msgs::GetMap srv_map;

	while (!ros::service::waitForService("/drivable_road_map_map", ros::Duration(15.0))) {
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

void GlobalMapCallback(const nav_msgs::OccupancyGridConstPtr &msg){
	
	if(map_get) return;
	globalObstacleMap_new = *msg;
	printf("Global Obstacle Map callback\n");
	map_get = true;
}


int main(int argc, char** argv){
	printf("aaaa\n");
	ros::init(argc,argv,"time_accum_map_node");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");

	printf("ddddd\n");
	p_nh.param<double>("decay_time",decay_time,2.0);

	if(decay_time > 10.0){
		ROS_WARN("max decay time is 10 sec. force to decay_time = 10.0");
		decay_time = 10.0;
	}
//
//    if(mission_state_.data == 4)
//    {
//        decay_time = 1;
//    }

	QSize = decay_time * updateRate;
	ROS_INFO("QSize = %d",QSize);
	printf("QSize : %d , ",QSize);
	// initGlobalObstacleMap();

	tf::TransformListener listener;
	message_filters::Subscriber<sensor_msgs::PointCloud2> obstacle_subscriber;
	tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter;



	// obstacle_subscriber.subscribe(nh,"velodyneOtherPoints",15);
	obstacle_subscriber.subscribe(nh,"/vlp_cloud/pharos_pathplanning",1);
	
	// obstacle_subscriber.subscribe(nh,"/remove_ground",15);
	tf_filter = new tf::MessageFilter<sensor_msgs::PointCloud2>(obstacle_subscriber,listener,"odom",15);
	tf_filter->registerCallback(boost::bind(obstacleCloudCallback,_1));
	pListener = &listener;

    Mission_state_sub_ = nh.subscribe("/mission_state",10,MissionStateCallback);
	vehicle_odom_sub_ = nh.subscribe("/odom/vehicle",1,VehicleOdomCallback);
	global_map_sub_ = nh.subscribe("drivable_map_planner_map",1,GlobalMapCallback);
	// tracked_object_sub_ = nh.subscribe("tracked_pharos_msgs",1,GlobalMapCallback);

	obstacleMapPublisher = nh.advertise<nav_msgs::OccupancyGrid>("/obstacle_map",20);

	map_service_client = nh.serviceClient<nav_msgs::GetMap>("/drivable_map_planner_map");
	printf("sssss\n");

	// initGlobalObstacleMap_new();

	ros::spin();

	return 1;
}

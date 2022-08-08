
#ifndef _PATH_PLANNER_H_
#define _PATH_PLANNER_H_

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//#include <pharos_lane_detect_ransac/Lane_center.h>
#include <pharos_msgs/MotionCommandStamped2.h>
//#include <pharos_msgs/VelocityLimit.h>
#include <pharos_msgs/StateStamped2016.h>
#include <pharos_msgs/State2016.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>
#include <fstream>
#include <iostream>

#include <pcl_ros/transforms.h>

//#include <pharos_stopline_localizer/stopline_offset.h>

#define num_path 9
#define planning_sample 100


extern nav_msgs::OccupancyGrid obstacle_map_;
extern int mission_state;

class CPathPlanning{

	public:

	int current_wp_index_;
	bool wp_index_get_;

	int driving_mode_;
	int current_in_sector_;
	int sector_;
	int lane_sector_;
	int waypoint_matching_sig_;
	std::vector<tf::Point> Wp_;
	std::vector<tf::Point> Curvature_;

	double lane_center_;
	double lateral_path_offset_;

	nav_msgs::OccupancyGrid Map_;

	std::vector<tf::Point> path_;
	nav_msgs::Path Final_path_;


	// TF
	tf::TransformListener tf_;


	tf::Point gps_pt_;

	double angle_diff_;
	double curvature_;
	double lateral_offset_;
	double speed_command_;
	double crio_speed_;
	double speed_;
	double theta_;
	double beta_;



	//double trajectory_[num_path][planning_sample][3];
	//double path_[planning_sample][2];

	double lookahead_x;
	double lookahead_y;


	double sample_time;

	double ref_lateral_offset_;


	double origin_gps_x_;
	double origin_gps_y_;

	int heading_condition;



	ros::NodeHandlePtr node_;
	ros::NodeHandlePtr pnode_;

	//ros::NodeHandle n_;

	ros::Publisher heading_pub_;
	ros::Publisher motion_pub_;
	ros::Publisher trajectory_pub_;
	ros::Publisher parking_path_pub_;

	ros::Publisher candidate_path_pub_1_;
	ros::Publisher candidate_path_pub_2_;
	ros::Publisher candidate_path_pub_3_;
	ros::Publisher candidate_path_pub_4_;
	ros::Publisher candidate_path_pub_5_;
	ros::Publisher candidate_path_pub_6_;
	ros::Publisher candidate_path_pub_7_;
	ros::Publisher candidate_path_pub_8_;
	ros::Publisher candidate_path_pub_9_;


	ros::Subscriber gps_sub_;

	ros::Subscriber waypoint_sub_;
	ros::Subscriber curvature_sub_;

	ros::Subscriber parking_pt_sub_;
	ros::Subscriber parking_points_sub_;
	ros::Subscriber final_parking_pt_sub_;

	ros::Subscriber vehicle_sub_;
	ros::Subscriber roof_cloud_sub_;
	ros::Subscriber bumper_cloud_sub_;

	ros::Subscriber local_map_sub_;

	ros::Subscriber lane_center_sub_;

	ros::Subscriber obstacle_map_sub_;

	ros::Subscriber current_wp_sub_;

	ros::Subscriber Mission_state_sub_;
	ros::Subscriber sector_sub_;
	ros::Subscriber lane_sector_sub_;

	ros::Subscriber  current_in_sector_sub_;
	ros::Subscriber  stopline_offset_sub_;

	int init();
	void publish();

	void WaypointCallback(const nav_msgs::PathConstPtr& wp);
	void CurvatureCallback(const nav_msgs::PathConstPtr& curv);
	void LocalMapCallback(const nav_msgs::OccupancyGridConstPtr& map);
	void MotionCommandCallback(const pharos_msgs::MotionCommandStamped2ConstPtr& msg);
	void VehicleStateCallback(const pharos_msgs::StateStamped2016ConstPtr& msg);
	void GPSCallback(const nav_msgs::OdometryConstPtr& msg);
	void ObstacleMapCallback(const nav_msgs::OccupancyGridConstPtr& map);
	void MissionStateCallback(const std_msgs::Int32ConstPtr& msg);
	void WpIndexCallback(const std_msgs::Int32ConstPtr& msg);

	int T_closestwaypoint (std::vector<tf::Point> *path, double gpsx, double gpsy, double heading);

	int ClosetWaypoint_order(int old_current,double gpsx, double gpsy, double heading);

};


#endif







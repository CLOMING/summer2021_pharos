
#include <path_planner.h>


#include <lateral_offset_planning.h>
#include <rs_path.h>
#include <parking_planning.h>

#define data_size 5
#define lat_off_rate 10
#define GPStoLaser 3.66


CLatOffsetPlanning latoffset_planning;
nav_msgs::OccupancyGrid obstacle_map_;


void CPathPlanning::WpIndexCallback(const std_msgs::Int32ConstPtr& msg){
	current_wp_index_ = msg->data;
	wp_index_get_ = true;
}


void CPathPlanning::ObstacleMapCallback(const nav_msgs::OccupancyGridConstPtr& map){
	//ROS_INFO("h: %d, w: %d, resoultion: %.3lf",map->info.height,map->info.width,map->info.resolution);
	Map_.data = map->data;
	Map_.header = map->header;
	Map_.info = map->info;

	obstacle_map_.data = map->data;
	obstacle_map_.header = map->header;
	obstacle_map_.info = map->info;
}


void CPathPlanning::WaypointCallback(const nav_msgs::PathConstPtr& wp)
{
	if(Wp_.size()!=0){
		return;
	}
	else if(Wp_.size()==0){
		tf::Point pt;
		double x,y;
		Wp_.clear();
		for(int i=0; i < wp->poses.size(); i++){
			x = wp->poses[i].pose.position.x;
			y = wp->poses[i].pose.position.y;
			pt.setValue(x,y,0.0);
			Wp_.push_back(pt);
		}
		printf("[Path_planner] Waypoints is received!\n");
	}
}

void CPathPlanning::CurvatureCallback(const nav_msgs::PathConstPtr& curv)
{
	if(Curvature_.size()!=0){
		return;
	}
	else if(Curvature_.size()==0){
		tf::Point pt;
		double x,y,c;
		Curvature_.clear();
		for(int i=0; i < curv->poses.size(); i++){
			x = curv->poses[i].pose.position.x;
			y = curv->poses[i].pose.position.y;
			c = curv->poses[i].pose.position.z;
			pt.setValue(x,y,0.0);
			Curvature_.push_back(pt);
		}
		printf("[Path_planner] Waypoints is received!\n");
	}
}

void CPathPlanning::MotionCommandCallback(const pharos_msgs::MotionCommandStamped2ConstPtr& msg){
	lateral_offset_ = msg->lateral_offset;
}

void CPathPlanning::VehicleStateCallback(const pharos_msgs::StateStamped2016ConstPtr& msg){
	crio_speed_ = msg->state.velocity;
}

void CPathPlanning::GPSCallback(const nav_msgs::OdometryConstPtr& msg)
{

	ros::Time t1 = ros::Time::now();

	if(Wp_.size() == 0 ){
		ROS_FATAL("There is no WayPoint");
		return;
	}

	//--- Current Position
	static double heading;

	double lookahead = 3.0;

	gps_pt_.setValue(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0);
	// heading
	tf::Pose pose;
	tf::poseMsgToTF(msg->pose.pose, pose);

	double Roll,Pitch,Yaw;
	pose.getBasis().getRPY(Roll, Pitch, Yaw);
	heading = Yaw;

	lookahead_x = gps_pt_.x() + lookahead*cos(heading);
	lookahead_y = gps_pt_.y() + lookahead*sin(heading);



	//// Closest Way Point ////

	int current = 0;
	//int lookahead_current = 0;
	static int old_current = 0;

	//current = ClosetWaypoint_order(old_current, gps_pt_.x(), gps_pt_.y(), heading);
	current = current_wp_index_;

	//lookahead_current = ClosetWaypoint_order(old_current, lookahead_x, lookahead_y, heading);
	old_current = current;

	if(current < (Wp_.size()-1)){
		ref_lateral_offset_ = latoffset_planning.lateraloffset(gps_pt_.x(), gps_pt_.y(), Wp_[current].x(), Wp_[current].y(), Wp_[current+1].x(), Wp_[current+1].y());
	}
	else{
		ref_lateral_offset_ = latoffset_planning.lateraloffset(gps_pt_.x(), gps_pt_.y(), Wp_[current-1].x(), Wp_[current-1].y(), Wp_[current].x(), Wp_[current].y());
	}
	//printf("ref_lateral_offset: %lf\n",ref_lateral_offset_);


	//-----------------Area and Driving Mode Setting ----------------------------//

	//driving mode : 1 waypoint following
	//driving mode : 2 waypoint following
	//driving mode : 3 waypoint following

	driving_mode_ = 1;

	waypoint_matching_sig_= 0;

	//--- Path Planning



		////////------------ Planning Simulation ------------////////

		static int path_choice = (num_path-1)/2;
		static double shifting[num_path];
		double path_width = 0.8; //0.5

		static double trajectory_dist = 0.0;
		static int collision_flag = 0;
		static double maximum_curvature = 0.0;

		static double traj_dist[num_path];
		static int col_flag[num_path];
		static double max_curve[num_path];


		for (int i=0;i<num_path;i++){
			shifting[i] = -lateral_path_offset_*((num_path-1)/2) + i * lateral_path_offset_; // To get several candidate paths that have some lateral offset from reference path
		}


		//----------------------------------------------------

		double velocity;//m/s

		velocity = crio_speed_; // m/s
		//velocity = 1.0;
		double min_lat = 20.0;
		path_choice = (num_path-1)/2;

		std::vector<tf::Point> candidate_path[num_path];
		std::vector<tf::Point> final_path;

		std::vector<tf::Point> wp_segment;
		wp_segment.clear();


		std::vector<nav_msgs::Path> candidate_path_nav;
		candidate_path_nav.clear();
		candidate_path_nav.resize(num_path);

		for (int i=0;i<num_path;i++){

			candidate_path[i].clear();
			candidate_path[i]=latoffset_planning.trajectory_simulation(&Wp_,&Curvature_,i,current, gps_pt_.x(), gps_pt_.y(), heading, shifting[i], lane_center_,velocity, trajectory_dist,collision_flag, maximum_curvature);

			traj_dist[i] = trajectory_dist;
			col_flag[i] = collision_flag;
			max_curve[i] = maximum_curvature;


			candidate_path_nav.at(i).poses.clear();
			candidate_path_nav.at(i).poses.resize(candidate_path[i].size());



			for(int j=0; j<candidate_path[i].size();j++){
				candidate_path_nav.at(i).poses[j].pose.position.x = candidate_path[i].at(j).x();
				candidate_path_nav.at(i).poses[j].pose.position.y = candidate_path[i].at(j).y();
				candidate_path_nav.at(i).poses[j].pose.position.z = 0;
			}

		}

		candidate_path_nav.at(0).header.stamp = ros::Time::now();
		candidate_path_nav.at(0).header.frame_id = "odom";
		candidate_path_pub_1_.publish(candidate_path_nav.at(0));

		candidate_path_nav.at(1).header.stamp = ros::Time::now();
		candidate_path_nav.at(1).header.frame_id = "odom";
		candidate_path_pub_2_.publish(candidate_path_nav.at(1));

		candidate_path_nav.at(2).header.stamp = ros::Time::now();
		candidate_path_nav.at(2).header.frame_id = "odom";
		candidate_path_pub_3_.publish(candidate_path_nav.at(2));

		candidate_path_nav.at(3).header.stamp = ros::Time::now();
		candidate_path_nav.at(3).header.frame_id = "odom";
		candidate_path_pub_4_.publish(candidate_path_nav.at(3));

		candidate_path_nav.at(4).header.stamp = ros::Time::now();
		candidate_path_nav.at(4).header.frame_id = "odom";
		candidate_path_pub_5_.publish(candidate_path_nav.at(4));

		candidate_path_nav.at(5).header.stamp = ros::Time::now();
		candidate_path_nav.at(5).header.frame_id = "odom";
		candidate_path_pub_6_.publish(candidate_path_nav.at(5));

		candidate_path_nav.at(6).header.stamp = ros::Time::now();
		candidate_path_nav.at(6).header.frame_id = "odom";
		candidate_path_pub_7_.publish(candidate_path_nav.at(6));

		candidate_path_nav.at(7).header.stamp = ros::Time::now();
		candidate_path_nav.at(7).header.frame_id = "odom";
		candidate_path_pub_8_.publish(candidate_path_nav.at(7));

		candidate_path_nav.at(8).header.stamp = ros::Time::now();
		candidate_path_nav.at(8).header.frame_id = "odom";
		candidate_path_pub_9_.publish(candidate_path_nav.at(8));


		double path_cost[num_path];
		double w_lat = 0.05;
		double w_curv = 0;//1;
		double min_cost = DBL_MAX;
		double max_dist = DBL_MIN;
		double collision_count =0;
		static int old_choice = (num_path-1)/2;
		int path_choice_buf = (num_path-1)/2;


		for (int i=0;i<num_path;i++){
			path_cost[i] = w_lat * abs(shifting[i]) + w_curv * max_curve[i];
			if (col_flag[i] == 0){
				//          if (abs(shifting[i] < min_lat)){
				//            min_lat = abs(shifting[i]);
				//            path_choice = i;
				//          }
				if (path_cost[i] < min_cost){
					min_cost = path_cost[i];
					path_choice_buf = i;
				}
			}
			else{
				collision_count += 1;
			}
		}

		//path_choice = path_choice_buf;
		//ROS_INFO("lateral: %f, curvature: %f",shifting[path_choice_buf], max_curve[path_choice_buf]);


		if (abs(traj_dist[path_choice_buf] - traj_dist[old_choice])/traj_dist[old_choice] < 0.08 )
		{
			path_choice_buf = old_choice;
		}


		path_choice = path_choice_buf;

		if (collision_count == 0 ){
			path_choice = (num_path-1)/2;
		}
		else{
			if (col_flag[int(num_path-1)/2] == 0 && collision_count < 9){
				path_choice = (num_path-1)/2;
			}
			else{
				if (col_flag[old_choice] == 0){
					path_choice = old_choice;
				}
				else if (collision_count == num_path){
					for (int i=((num_path-1)/2);i<num_path;i++){
						if (traj_dist[i]>max_dist){
							max_dist = traj_dist[i];
							path_choice = i;
							//if (abs(traj_dist[i] - traj_dist[old_choice])/traj_dist[old_choice] < 0.12 ){
							//  path_choice = old_choice;
							//}
						}
					}
				}
				else{
					path_choice = path_choice_buf;
				}
			}
		}

		old_choice = path_choice;



		//-------------------------------------------------------------


		final_path.clear();
		final_path = candidate_path[path_choice];

		if(final_path.size() == 0){
			ROS_INFO("no path");
			return;
		}

		path_.resize(planning_sample);

		for (int i=0;i<planning_sample;i++){
			path_[i].setValue(final_path[i].x(),final_path[i].y(),0.0);
		}


	//--- Final Path Publish

	Final_path_.poses.resize(path_.size());
	Final_path_.header.stamp = ros::Time::now();
	Final_path_.header.frame_id = "odom";
	for(int i=0; i<path_.size();i++){
		Final_path_.poses[i].pose.position.x = path_[i].x();
		Final_path_.poses[i].pose.position.y = path_[i].y();
		Final_path_.poses[i].pose.position.z = path_[i].z();
	}
	trajectory_pub_.publish(Final_path_);

	ros::Time t2 = ros::Time::now();
	double dt = (t2-t1).toSec();
	//ROS_INFO("dt: %lf",dt);

}

int CPathPlanning::T_closestwaypoint (std::vector<tf::Point> *path, double gpsx, double gpsy, double heading){

	int order;
	double dist;
	double min_dist = DBL_MAX;

	for (int i=0; i<path->size();i++)
	{
		dist = sqrt(pow((gpsx - path->at(i).x()),2) + pow((gpsy - path->at(i).y()),2));
		if (min_dist > dist)
		{
			min_dist = dist;
			order = i;
		}
	}
	return order;
}

int CPathPlanning::ClosetWaypoint_order(int old_current,double gpsx, double gpsy, double heading){

	tf::Point vehicle_front;
	tf::Point gps;
	double dist_gpsTolaser = 3.66;
	double vehicle_front_x;
	double vehicle_front_y;

	gps.setX(gpsx);
	gps.setY(gpsy);

	vehicle_front_x = gpsx + dist_gpsTolaser * cos(heading);
	vehicle_front_y = gpsy + dist_gpsTolaser * sin(heading);
	vehicle_front.setX(vehicle_front_x);
	vehicle_front.setY(vehicle_front_y);

	double dist = 0.0;
	double min_dist = DBL_MAX;
	static size_t order = 0;

	double wpx;
	double wpy;

	for (int j = 0; j < Wp_.size(); j++){
		dist = gps.distance(Wp_[j]);
		if (min_dist > dist){
			min_dist = dist;
			order = j;
		}
	}


	//	if(old_current == 0){
	//		for (int j = old_current; j < Wp_.size(); j++){
	//			dist = gps.distance(Wp_[j]);
	//			if (min_dist > dist){
	//				min_dist = dist;
	//				order = j;
	//			}
	//		}
	//	}
	//	else{
	//		for (int j = old_current; j < old_current+100; j++){
	//			dist = gps.distance(Wp_[j]);
	//			if (min_dist > dist){
	//				min_dist = dist;
	//				order = j;
	//			}
	//		}
	//	}

	return order;
}




// Load parameters etc
int CPathPlanning::init()
{
	ROS_INFO("Path planner initialize");

	lane_center_ = 0;
	lane_sector_ = 1;
	current_wp_index_ =
	wp_index_get_ = false;
	Wp_.clear();
	Curvature_.clear();

	node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
	pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

	pnode_->param("lateral_path_offset", lateral_path_offset_, 1.0);

	int path_q_size = 3;
	trajectory_pub_ = node_->advertise<nav_msgs::Path>(std::string("trajectory"), path_q_size);

	candidate_path_pub_1_ = node_->advertise<nav_msgs::Path>(std::string("/path/candidate_path_1"), path_q_size);
	candidate_path_pub_2_ = node_->advertise<nav_msgs::Path>(std::string("/path/candidate_path_2"), path_q_size);
	candidate_path_pub_3_ = node_->advertise<nav_msgs::Path>(std::string("/path/candidate_path_3"), path_q_size);
	candidate_path_pub_4_ = node_->advertise<nav_msgs::Path>(std::string("/path/candidate_path_4"), path_q_size);
	candidate_path_pub_5_ = node_->advertise<nav_msgs::Path>(std::string("/path/candidate_path_5"), path_q_size);
	candidate_path_pub_6_ = node_->advertise<nav_msgs::Path>(std::string("/path/candidate_path_6"), path_q_size);
	candidate_path_pub_7_ = node_->advertise<nav_msgs::Path>(std::string("/path/candidate_path_7"), path_q_size);
	candidate_path_pub_8_ = node_->advertise<nav_msgs::Path>(std::string("/path/candidate_path_8"), path_q_size);
	candidate_path_pub_9_ = node_->advertise<nav_msgs::Path>(std::string("/path/candidate_path_9"), path_q_size);


	gps_sub_ = node_->subscribe<nav_msgs::Odometry>("/ekf_odom", 50, &CPathPlanning::GPSCallback, this);
	vehicle_sub_ = node_->subscribe<pharos_msgs::StateStamped2016>("/vehicle/state2016", 30, &CPathPlanning::VehicleStateCallback, this);

	waypoint_sub_ = node_->subscribe<nav_msgs::Path>("/road_data/waypoint",10, &CPathPlanning::WaypointCallback, this);
	curvature_sub_ = node_->subscribe<nav_msgs::Path>("/road_data/curvature",10, &CPathPlanning::CurvatureCallback, this);

	obstacle_map_sub_ = node_->subscribe<nav_msgs::OccupancyGrid>("/obstacleMap",20, &CPathPlanning::ObstacleMapCallback, this);

	current_wp_sub_= node_->subscribe<std_msgs::Int32>("/current_wp_index",10, &CPathPlanning::WpIndexCallback, this);

	return 0;
}



// Publish data
void CPathPlanning::publish()
{
	ros::Rate loop_rate(100);

	while (node_->ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}


int main(int argc, char **argv)
{
	ROS_INFO("path planner main start");

	ros::init(argc, argv, "path_planner");

	CPathPlanning rp_ros;

	if (rp_ros.init())
	{
		ROS_FATAL("PathPlannerNode initialization failed");
		return -1;
	}

	rp_ros.publish();

	return 0;
}

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>

#include <fstream>

#include <pharos_msgs/WaypointOffset.h>


class RoadInformationNode
{
public:

	std::vector<tf::Point> wp_;
	std::vector<tf::Point> curvature_;
	std::vector<tf::Point> ref_speed_;

	int32_t publish_rate_;

	std::string road_data_filename_;

	ros::NodeHandlePtr node_;
	ros::NodeHandlePtr pnode_;

	ros::Publisher waypoint_pub_;
	ros::Publisher curvature_pub_;
	ros::Publisher ref_speed_pub_;

	ros::Subscriber wp_offset_sub_;

	double origin_gps_x_;
	double origin_gps_y_;
	bool relative_gps_;
	bool gps_origin_check_;

	double lat_offset_x_;
	double lat_offset_y_;
	double ver_offset_x_;
	double ver_offset_y_;

	double slow_down_scale_;

	bool Reverse_waypoint_;

	RoadInformationNode() :
			publish_rate_(10)
	{
	}
	~RoadInformationNode()
	{
	}

	void WpOffsetCallback(const pharos_msgs::WaypointOffsetConstPtr& offset){

		double new_lat_offset_x_ = offset->lateral_x;
		double new_lat_offset_y_ = offset->lateral_y;
		double new_ver_offset_x_ = offset->vertical_x;
		double new_ver_offset_y_ = offset->vertical_y;

		lat_offset_x_ += new_lat_offset_x_;
		lat_offset_y_ += new_lat_offset_y_;
		ver_offset_x_ += new_ver_offset_x_;
		ver_offset_y_ += new_ver_offset_y_;
	}

	double CurveSpeed(const double origin_speed,const int obstacle){
		double final_speed;
		if(obstacle == 1) final_speed = origin_speed*slow_down_scale_;
		else final_speed = origin_speed;
		return final_speed;
	}
	void ReversePath(const nav_msgs::PathConstPtr &in_path, nav_msgs::PathPtr &reverse_path){
		*reverse_path = *in_path;
		unsigned long path_size = in_path->poses.size();

		for(unsigned int i=0; i<in_path->poses.size();i++){
			reverse_path->poses.at(i) = in_path->poses.at(path_size-1-i);
		}
	}

	// Load parameters etc
	int init()
	{
		node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
		pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

		pnode_->param("publish_rate", publish_rate_, 5);
		pnode_->param("road_data_file", road_data_filename_, std::string(""));

		pnode_->getParam("speed/down_scale",slow_down_scale_);

		pnode_->param("reverse_waypoint", Reverse_waypoint_, false);



		if (node_->getParam("gps/origin/x", origin_gps_x_)) {
			ROS_INFO("gps/origin/x : %f",origin_gps_x_);
		}
		else {
			ROS_INFO("Failed to get param /gps/origin/x");
			origin_gps_x_ = 0.0;
		}
		if (node_->getParam("gps/origin/y", origin_gps_y_)) {
			ROS_INFO("gps/origin/y : %f",origin_gps_y_);
		}
		else {
			ROS_INFO("Failed to get param /gps_ekf/oriSpeediumStartFinishLineVisualizationgin/y");
			origin_gps_y_ = 0.0;
		}
		if (node_->getParam("gps/origin/y", origin_gps_y_) && node_->getParam("gps/origin/x", origin_gps_x_)){
			ROS_INFO("Get gps/origin finish");
			gps_origin_check_ = true;
		}
		else{
			gps_origin_check_ = false;
		}
		// if (node_->getParam("gps_ekf/relative_gps", relative_gps_)) {
		// 	ROS_INFO("gps_ekf/relative_gps : %d",relative_gps_);
		// }
		// else {
		// 	ROS_INFO("Failed to get param /gps_ekf/relative_gps");
		// }
		if (!road_data_filename_.size())
		{
			ROS_FATAL("Parameter road_data_file is not specified or empty!");
			return -1;
		}


		if (loadRoadData())
			return -1;

		waypoint_pub_ = node_->advertise<nav_msgs::Path>(std::string("/road_data/waypoint"), 5);
		curvature_pub_ = node_->advertise<nav_msgs::Path>(std::string("/road_data/curvature"), 5);
		ref_speed_pub_ = node_->advertise<nav_msgs::Path>(std::string("/road_data/ref_speed"), 5);

		wp_offset_sub_ = node_->subscribe<pharos_msgs::WaypointOffset>("/waypoint_offset",10, &RoadInformationNode::WpOffsetCallback, this);

		lat_offset_x_ = 0.0;
		ver_offset_x_ = 0.0;
		lat_offset_y_ = 0.0;
		ver_offset_y_ = 0.0;

		return 0;
	}


	int loadRoadData()
	{
		std::fstream fs_test;
		std::fstream fs_left_wp;
		std::fstream fs_right_wp;

		fs_test.open(road_data_filename_.c_str(), std::fstream::in);
		if (!fs_test.is_open())
		{
			ROS_FATAL("Cannot open Road data file: %s", road_data_filename_.c_str());
			return -1;
		}

		std::string str;
		std::istringstream ss;
		std::getline(fs_test, str);
		ss.str(str);
		unsigned  int num_of_data = std::distance(std::istream_iterator<std::string>(ss), std::istream_iterator<std::string>());
		printf("Number of waypoint data: %d\n",num_of_data);
		std::fstream fs;
		fs.open(road_data_filename_.c_str(), std::fstream::in);
		if (!fs.is_open())
		{
			ROS_FATAL("Cannot open Road data file: %s", road_data_filename_.c_str());
			return -1;
		}

		wp_.clear();
		curvature_.clear();
		ref_speed_.clear();

		int iter = 0;
		while(1)
		{
			iter++;
			double x, y, z, curvature, velocity, sect;
			std::string str;
			tf::Point wp;
			tf::Point curv;
			tf::Point speed;

			std::istringstream ss;

			std::getline(fs, str);
			ss.str(str);

			if(num_of_data == 5){
				ss >> x >> y >> curvature >> velocity >> sect;
			}
			else if(num_of_data == 6){
				ss >> x >> y >> z >> curvature >> velocity >> sect;
			}
			else if(num_of_data == 4){
				ss >> x >> y >> z >> curvature;
			}

			if(fs.eof())break;

			wp.setValue(x, y, z);
			wp_.push_back(wp);

			curv.setValue(0.0, 0.0, curvature);
			curvature_.push_back(curv);

			speed.setValue(0.0,0.0,velocity);
			ref_speed_.push_back(speed);
		}


		double total_dist = 0.0;
		for(int i=0;i<wp_.size()-1;i++){
			total_dist += sqrt(pow(wp_.at(i+1).x()-wp_.at(i).x(),2)+pow(wp_.at(i+1).y()-wp_.at(i).y(),2));
		}
		double avg_dist = total_dist/(double)(wp_.size()-1);

		printf("\n");
		printf("Total Waypoint size: %d\n",wp_.size());
		printf("Average distance between each waypoint: %.4f m\n",avg_dist);

		if (!wp_.size())
			return -2;

		// Everything OK
		return 0;
	}


	// Publish data
	void publish()
	{
		ros::Rate loop_rate(publish_rate_);

		while (node_->ok())
		{

			nav_msgs::PathPtr waypoint(new nav_msgs::Path);
			nav_msgs::PathPtr curvature(new nav_msgs::Path);
			nav_msgs::PathPtr ref_speed(new nav_msgs::Path);

			waypoint->header.frame_id = "odom";

			curvature->header.stamp = ros::Time::now();
			curvature->header.frame_id = "odom";

			ref_speed->header.stamp = ros::Time::now();
			ref_speed->header.frame_id = "odom";

			waypoint->poses.resize(wp_.size());
			curvature->poses.resize(curvature_.size());
			ref_speed->poses.resize(ref_speed_.size());

			double final_offset_x = 0;
			double final_offset_y = 0;

			if(!gps_origin_check_){
				origin_gps_x_ = waypoint->poses[0].pose.position.x;
				origin_gps_y_ = waypoint->poses[0].pose.position.y;
			}

			final_offset_x = origin_gps_x_ + lat_offset_x_ + ver_offset_x_;
			final_offset_y = origin_gps_y_ + lat_offset_y_ + ver_offset_y_;

			if(!wp_.empty()){
				for(int i=0; i<wp_.size();i++){
					waypoint->poses[i].pose.position.x = wp_[i].x() - final_offset_x;
					waypoint->poses[i].pose.position.y = wp_[i].y() - final_offset_y;
					waypoint->poses[i].pose.position.z = wp_[i].z() - wp_[0].z();
				}
			}
			if(!curvature_.empty()){
				for(int i=0; i<curvature_.size();i++){
					curvature->poses[i].pose.position.x = wp_[i].x() - final_offset_x;
					curvature->poses[i].pose.position.y = wp_[i].y() - final_offset_y;
					curvature->poses[i].pose.position.z = curvature_[i].z();
				}
			}

			//------Speed Setting
			if(!ref_speed_.empty()){
				for(int i=0; i<ref_speed_.size();i++){
					ref_speed->poses[i].pose.position.x = wp_[i].x() - final_offset_x;
					ref_speed->poses[i].pose.position.y = wp_[i].y() - final_offset_y;
					ref_speed->poses[i].pose.position.z = ref_speed_[i].z();
				}
			}


			if(!Reverse_waypoint_){
				waypoint_pub_.publish(waypoint);
				curvature_pub_.publish(curvature);
				ref_speed_pub_.publish(ref_speed);
			}
			else{
				nav_msgs::PathPtr waypoint_reverse(new nav_msgs::Path);
				nav_msgs::PathPtr curvature_reverse(new nav_msgs::Path);
				nav_msgs::PathPtr ref_speed_reverse(new nav_msgs::Path);

				ReversePath(waypoint,waypoint_reverse);
				ReversePath(curvature,curvature_reverse);
				ReversePath(ref_speed,ref_speed_reverse);

				waypoint_pub_.publish(waypoint_reverse);
				curvature_pub_.publish(curvature_reverse);
				ref_speed_pub_.publish(ref_speed_reverse);

			}

			ros::spinOnce();
			loop_rate.sleep();
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "road_information");

	RoadInformationNode rp_ros;
	if (rp_ros.init())
	{
		ROS_FATAL("RoadInformationNode initialization failed");
		return -1;
	}

	rp_ros.publish();
	return 0;
}

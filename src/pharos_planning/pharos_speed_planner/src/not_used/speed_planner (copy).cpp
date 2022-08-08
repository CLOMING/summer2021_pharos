#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//#include <pharos_msgs/MotionCommandStamped3.h>
#include <pharos_msgs/StateStamped2016.h>
#include <pharos_msgs/SpeedCommand.h>

#include <pharos_path_planner/RoadInfo.h>
#include <pharos_path_planner/ReferencePath.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include <pharos_path_planner/RoadInfo.h>
#include <pharos_path_planner/ReferencePath.h>

#define G 9.81
#define G_lateral 0.15*G

const double MinSpeedLimit = 3.0 // [km/h]

class CSpeedPlanner
{
public:

	std::vector<tf::Point> Wp_;
	std::vector<tf::Point> Ref_speed_;


	double max_speed_;
	double min_speed_;
	double speed_scale_;
    double decceleration_G_;
	double crio_speed_;
    float obstalce_dist_;

	int32_t publish_rate_;


	ros::NodeHandlePtr node_;
	ros::NodeHandlePtr pnode_;

	tf::TransformListener listener_;

	ros::Publisher speed_cmd_pub_;
	ros::Publisher ref_speed_profile_pub_;

	ros::Subscriber ref_roadinfo_sub_;	

	ros::Subscriber waypoint_sub_;
	ros::Subscriber ref_speed_sub_;
	ros::Subscriber gps_sub_;
	ros::Subscriber vehicle_sub_;
	ros::Subscriber current_wp_sub_;
    ros::Subscriber planner_speed_cmd_sub_;
    ros::Subscriber obstacle_dist_sub_;

    int current_wp_index_;
	bool wp_index_msg_flag_;
	bool wp_data_flag_;
	bool ref_speed_data_flag_;

	void ReferenceRoadInfoCallback(const pharos_path_planner::ReferencePathConstPtr &msg){

		nav_msgs::Path reference_speed;
		pharos_path_planner::ReferencePathPtr ref_roadinfo(new pharos_path_planner::ReferencePath);
		*ref_roadinfo = *msg;
		
		std::vector<pharos_path_planner::RoadInfo>::iterator iter;

		geometry_msgs::PoseStamped speed_pose;
		double speed;
		double min_speed = DBL_MAX;
		
		for(iter=ref_roadinfo->roadinfo.begin();iter!=ref_roadinfo->roadinfo.end();iter++){
			
			speed = sqrt(G_lateral/fabs(iter->curvature));
			speed *= 3.6; //Convert [m/s] to [km/h]
 
			if(speed > iter->speed_limit){
				speed = iter->speed_limit;
			}
			if(speed < min_speed){
				min_speed = speed;
			}
			if(speed < MinSpeedLimit){
				speed = MinSpeedLimit;
			}

			speed_pose.pose.position.x = iter->position.x;
			speed_pose.pose.position.y = iter->position.y;
			speed_pose.pose.position.z = speed;

			reference_speed.poses.push_back(speed_pose);
		}
		printf("Minimum Speed: %.3f km/h\n",min_speed);
		

		reference_speed.header.stamp = ros::Time::now();
		reference_speed.header.frame_id = "odom";
		ref_speed_profile_pub_.publish(reference_speed);
		
	}
    void ObstacleDistCallback(const std_msgs::Float32ConstPtr &msg){
    	return;
        obstalce_dist_ = msg->data;
    }


	void WpIndexCallback(const std_msgs::Int32ConstPtr& msg){
		return;
		current_wp_index_ = msg->data;
		wp_index_msg_flag_ = true;

		// if(!Without_WayPoint_){
		// 	if(!wp_data_flag_) return;
		// 	if(!ref_speed_data_flag_) return;
		// }
		speed_planning(current_wp_index_);
	}
	void WaypointCallback(const nav_msgs::PathConstPtr& wp) {
		return;

			tf::Point pt;
			double x,y;
			Wp_.clear();
			for(int i=0; i < wp->poses.size(); i++){
				x = wp->poses[i].pose.position.x;
				y = wp->poses[i].pose.position.y;
				pt.setValue(x,y,0.0);
				Wp_.push_back(pt);
			}
			wp_data_flag_ = true;
			//printf("[Speed_planner] Waypoints is received!\n");
	}
	void VehicleStateCallback(const pharos_msgs::StateStamped2016ConstPtr& msg){

		crio_speed_ = msg->state.velocity;
		//printf("vehicle state msg, velocity: %.3f\n",crio_speed_);
	}

	void RefSpeedCallback(const nav_msgs::PathConstPtr& speed){
		return;

			tf::Point pt;
			double x,y,s;
			Ref_speed_.clear();
			for(int i=0; i < speed->poses.size(); i++){
				x = speed->poses[i].pose.position.x;
				y = speed->poses[i].pose.position.y;
				s = speed->poses[i].pose.position.z;
				pt.setValue(x,y,s);
				Ref_speed_.push_back(pt);
			}
			ref_speed_data_flag_ = true;
			//printf("[Speed_planner] Reference Speed data is received!\n");
	}

	double Dist_along_with_WayPoint(int current_wp_index, int goal_wp_index){
		double dist_to_goal = 0.0;

		for(int i=current_wp_index;i<goal_wp_index;i++){
			dist_to_goal += Wp_[i].distance(Wp_[i+1]);
		}
		//printf("current_index: %d, goal_index: %d\n",current_wp_index_,goal_wp_index);
		return dist_to_goal;
	}

	inline double low_pass_filter(double previous_data, double measure_data){
		double estimated_data;
		double a = 0.5;
		estimated_data = a*previous_data + (1-a)*measure_data;
		return estimated_data;
	}

	void speed_planning(const int current_wp_index){
		//-- speed_limit_ : km/h
		//-- speed_limit_distance_ : m
		double speed_limit = 0.0;
		double speed_limit_distance = 0.0;
		int speed_limit_type = 1;


		//--Reference Speed Planning
		double speed_observing_distance = 0.0;
		double observed_dist = 0.0;
		double reference_goal_speed = 0.0;
		double reference_goal_distance = 0.0;
		tf::Point reference_speed_position;
		int reference_speed_position_index = 0;

		double min_reference_speed = DBL_MAX;

		double min_observing_dist = 10.0;
		//speed_observing_distance = observing_dist_scale_*(crio_speed_*3.6); // ex) vehicle speed: 70km/h -> scale*70m observing

        //acceleration based observation distance
        speed_observing_distance = pow(crio_speed_,2)/(2*decceleration_G_*G);

		if (speed_observing_distance < min_observing_dist){
			speed_observing_distance = min_observing_dist;
		}

		// if(Without_WayPoint_){
		// 	reference_goal_speed = 20.0;
		// 	reference_goal_distance = 0.0;
		// 	speed_limit_type = 1;
		// 	reference_goal_speed = 10*sin(0.1*current_wp_index*M_PI/180)+10;
		// }
		// else{
  //           speed_limit = Ref_speed_[current_wp_index].z();
		// }

		speed_limit = Ref_speed_[current_wp_index].z();	
		//speed limitation---------
		speed_limit *= speed_scale_;

		if(speed_limit > max_speed_){
			speed_limit = max_speed_;
		}
		if(speed_limit < min_speed_){
			speed_limit = min_speed_;
		}


        if(obstalce_dist_ != 1000){
            speed_limit = -crio_speed_/obstalce_dist_*0.1+crio_speed_;
            if(speed_limit<0)
                speed_limit=0;
        }

		pharos_msgs::SpeedCommand speed_cmd;

		speed_cmd.goal_speed = speed_limit;
		speed_cmd.goal_distance = speed_limit_distance;
		speed_cmd.speed_type = speed_limit_type;

		speed_cmd_pub_.publish(speed_cmd);
	}

	// Load parameters etc
	int init()
	{
		node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
		pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

		pnode_->param("publish_rate", publish_rate_, 100);

		pnode_->param("max_speed", max_speed_, 60.0);
		pnode_->param("min_speed", min_speed_, 20.0);
		pnode_->param("speed_scale", speed_scale_, 1.0);
        pnode_->param("decceleration_G", decceleration_G_, 0.3);

		
		speed_cmd_pub_ = node_->advertise<pharos_msgs::SpeedCommand>("speed_command/reference", 100);
		ref_speed_profile_pub_ = node_->advertise<nav_msgs::Path>("/path/ref_speed_profile", 1);


		ref_roadinfo_sub_ = node_->subscribe<pharos_path_planner::ReferencePath>("/reference_road_info", 10, &CSpeedPlanner::ReferenceRoadInfoCallback, this);

		vehicle_sub_ = node_->subscribe<pharos_msgs::StateStamped2016>("/vehicle/state2016", 100, &CSpeedPlanner::VehicleStateCallback, this);

		waypoint_sub_ = node_->subscribe<nav_msgs::Path>("/road_data/waypoint",10, &CSpeedPlanner::WaypointCallback, this);
		ref_speed_sub_ = node_->subscribe<nav_msgs::Path>("/road_data/ref_speed",10, &CSpeedPlanner::RefSpeedCallback, this);
		current_wp_sub_= node_->subscribe<std_msgs::Int32>("/current_wp_index",10, &CSpeedPlanner::WpIndexCallback, this);
        obstacle_dist_sub_ = node_->subscribe<std_msgs::Float32>("/NEAR_OBJECT_DISTANCE",10, &CSpeedPlanner::ObstacleDistCallback, this);

		//Global Variables Initialization

		// current_wp_index_ = 0;
		// wp_index_msg_flag_ = false;

		// wp_data_flag_ = false;
		// ref_speed_data_flag_ = false;
		// Wp_.clear();
		// Ref_speed_.clear();

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
	ros::init(argc, argv, "speed_planner");

	CSpeedPlanner sp_ros;
	if (sp_ros.init())
	{
		ROS_FATAL("CSpeedPlanner initialization failed");
		return -1;
	}

	sp_ros.publish();

	return 0;
}

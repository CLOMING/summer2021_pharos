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

#include <visualization_msgs/Marker.h> //shinwoo

#define G 9.81

nav_msgs::PathPtr reference_speed_(new nav_msgs::Path);
nav_msgs::PathPtr reference_speed_new_(new nav_msgs::Path);

std_msgs::Float32 dist_data;
std_msgs::Float32 virtualdist;

class CSpeedPlanner
{
public:

	double MaxSpeedLimit_;
	double MinSpeedLimit_;
	double SpeedScale_;
    double LonAcc_;
    double LatAcc_;
	double VehicleSpeed_;


    //float obstalce_dist_;
	unsigned int present_node_;//shinwoo
	visualization_msgs::Marker object_box_;//shinwoo
	float obstacle_dist_ = 1000;//shinwoo
	int acc_state = 0;//shinwoo
	float current_dist; //shinwoo
	float dcc_stop_ = 18;
	float dcc_start_ = 40;

	visualization_msgs::Marker VirtualObject;


	int32_t publish_rate_;


	ros::NodeHandlePtr node_;
	ros::NodeHandlePtr pnode_;

	tf::TransformListener listener_;

	ros::Publisher speed_cmd_pub_;
	ros::Publisher dist_pub_;
	ros::Publisher ref_speed_profile_pub_;
	ros::Publisher ref_speed_profile_pub2_;
	ros::Publisher ref_roadinfo_pub_;
	ros::Publisher virtual_object_pub_;

	ros::Subscriber ref_roadinfo_sub_;
	ros::Subscriber current_roadinfo_sub_;

	ros::Subscriber vehicle_sub_;
	ros::Subscriber current_wp_sub_;

    //ros::Subscriber obstacle_dist_sub_;
	ros::Subscriber current_node_sub_;//shinwoo
	ros::Subscriber object_box_sub_;//shinwoo
	ros::Subscriber local_path_sub_;//shinwoo

	//ros::Subscriber virtualobject_box_sub_;//shinwoo
	ros::Subscriber virtual_dist_sub_;//shinwoo

    unsigned int current_global_path_index_;
    unsigned int current_lane_index_;

	/* shinwoo */
	void CurrentNodeCallback(const std_msgs::Int32Ptr msg)
	{
	    present_node_ = msg->data;
	}
	/* original --------------- */
	void ObjectBoxCallback(const visualization_msgs::MarkerPtr &msg)
	{
		// std::cout << "0" << std::endl;
		object_box_ = *msg;
		// std::cout << "1" << std::endl;
		obstacle_dist_ = 1000;
		// std::cout << "2" << std::endl;
		// std::cout << object_box_.points[0].y << std::endl;
		// if (abs(object_box_.points[8].y) < 3 && object_box_.points[0].x < 20){ // for test
		// std::cout << "----------------" << std::endl;
		// std::cout << object_box_.points.empty() << std::endl;
		// std::cout << "----------------" << std::endl;

		if (object_box_.points.empty()){
			std::cout << "No Object Box!!!!!" << std::endl;
			// if(obstacle_dist_ < 40){
			// 	obstacle_dist_ = 40;
			// }
			// else{
			// 	obstacle_dist_ = 1000;
			// }
		}
		else{
			if (object_box_.points[0].y * object_box_.points[8].y < 0 && object_box_.points[0].x < dcc_start_ && object_box_.points[0].x > 0){ // car at the front
			// if (object_box_.points[0].x < 20){ // for test
				current_dist = object_box_.points[0].x;
		 	}
			if (current_dist < obstacle_dist_ && current_dist > 0.1) obstacle_dist_ = current_dist;
		}

	}
	
	/*--------------test-------------------*/
	// void VirtualDistanceCallback(const std_msgs::Float32 virtualdist)
	// {
	// 	obstacle_dist_ = virtualdist.data;
	// }

	/*-------------------------------------*/
	/* ------- */
	void ReferenceRoadInfoCallback(const pharos_path_planner::ReferencePathConstPtr &msg){

		pharos_path_planner::ReferencePathPtr ref_roadinfo(new pharos_path_planner::ReferencePath);
		*ref_roadinfo = *msg;



		//--- Lateral-Acceleration-based Speed Profiling

		double G_lateral = LatAcc_ * G;

		reference_speed_->poses.clear();

		std::vector<pharos_path_planner::RoadInfo>::iterator iter;
		for(iter=ref_roadinfo->roadinfo.begin();iter!=ref_roadinfo->roadinfo.end();++iter){
			double speed = sqrt(G_lateral/fabs(iter->curvature));
			speed *= 3.6; //Convert [m/s] to [km/h]

			// std::cout << "speed: " << speed << std::endl;

			if(speed > iter->speed_limit){
				speed = iter->speed_limit;
			}
			// if(speed < MinSpeedLimit_){
			// 	speed = MinSpeedLimit_;
			// }

			if(speed < iter->reference_speed){
				iter->reference_speed = speed;
			}

			// std::cout << "speed: " << speed << std::endl;
			// std::cout << "reference_speed: " << iter->reference_speed << std::endl;

			geometry_msgs::PoseStamped speed_pose;

			speed_pose.pose.position.x = iter->position.x;
			speed_pose.pose.position.y = iter->position.y;
			speed_pose.pose.position.z = iter->reference_speed;

			reference_speed_->poses.push_back(speed_pose);
		}


		//--- Longitudinal-Acceleration-based Speed Profiling

		double G_longitudinal = LonAcc_*G;

		// Acceleration Limit
		for(iter=ref_roadinfo->roadinfo.begin();iter!=ref_roadinfo->roadinfo.end()-1;++iter){

			double v1 = iter->reference_speed/3.6; //[m/s]
			double v2 = (iter+1)->reference_speed/3.6; //[m/s]

			tf::Point p1, p2;
			p1.setValue(iter->position.x,iter->position.y,0.0);
			p2.setValue((iter+1)->position.x,(iter+1)->position.y,0.0);

			double d = p1.distance(p2); // distance between two waypoints
			double dt = d/v1; // traveling time to next waypoint
			double a = (v2-v1)/dt;

			double new_speed;
			if(a > G_longitudinal){
				new_speed = v1 + G_longitudinal*dt;
				(iter+1)->reference_speed = new_speed*3.6; //[km/h]
			}
			// std::cout << "Acc limit reference_speed: " << iter->reference_speed << std::endl;
		}

		// Decceleration Limit
		for(iter=ref_roadinfo->roadinfo.end()-1;iter!=ref_roadinfo->roadinfo.begin();--iter){


			double v1 = iter->reference_speed/3.6; //[m/s]
			double v2 = (iter-1)->reference_speed/3.6; //[m/s]

			tf::Point p1, p2;
			p1.setValue(iter->position.x,iter->position.y,0.0);
			p2.setValue((iter-1)->position.x,(iter-1)->position.y,0.0);

			double d = p1.distance(p2); // distance between two waypoints
			double dt = d/v2; // traveling time to next waypoint
			double a = (v2-v1)/dt;

			double new_speed;
			if(a > G_longitudinal){
				new_speed = v1 + 0.6*G_longitudinal*dt; // original : multiply = 0.8*G_longitudinal
				(iter-1)->reference_speed = new_speed*3.6; //[km/h]
			}

			// if(iter == ref_roadinfo->roadinfo.end()-1){
			// 	printf("speed 1: %.4f\n,",v1);
			// 	printf("speed 2: %.4f\n,",v2);
			// 	printf("acc: %.4f\n,",a);
			// 	printf("speed 2 new: %.4f\n,",new_speed);
			// 	printf("x1: %.3f, y1: %.3f\n",p1.x(), p1.y());
			// 	printf("x2: %.3f, y2: %.3f\n",p2.x(), p2.y());
			// 	printf("dist: %.3f\n",d);
			// 	printf("dt: %.3f\n",dt);
			// }

			// std::cout << "Dcc limit reference_speed: " << iter->reference_speed << std::endl;
		}

		reference_speed_new_->poses.clear();
		for(iter=ref_roadinfo->roadinfo.begin();iter!=ref_roadinfo->roadinfo.end();++iter){
			geometry_msgs::PoseStamped speed_pose;

			speed_pose.pose.position.x = iter->position.x;
			speed_pose.pose.position.y = iter->position.y;
			speed_pose.pose.position.z = iter->reference_speed;

			reference_speed_new_->poses.push_back(speed_pose);
		}

		double slow_speed = 5.0;
		unsigned int i = reference_speed_new_->poses.size()-2;
		while(1){
			if(reference_speed_new_->poses.at(i).pose.position.z < slow_speed){
				reference_speed_new_->poses.at(i).pose.position.z = slow_speed;
			}
			i--;
			if(reference_speed_new_->poses.at(i).pose.position.z > slow_speed) break;
			if(i==0) break;
		}


		/* shinwoo ------------ ACC Here ---------------------------------------*/
		if (present_node_ == 12){
		// if (present_node_ == 10 || present_node_ == 11 || present_node_ == 6 || present_node_ == 7){
		// if (true){
			for(iter=ref_roadinfo->roadinfo.end()-1;iter!=ref_roadinfo->roadinfo.begin();--iter){

				if (true)
				{
					if (obstacle_dist_ < dcc_start_){
						iter->reference_speed = double(36)/(dcc_start_ - dcc_stop_) * (obstacle_dist_ - dcc_stop_);
						std::cout << "-----------ACC ON-----------" << std::endl;
						std::cout << "distance: " << obstacle_dist_ << std::endl;
						std::cout << "dcc_start_: " << dcc_start_ << std::endl;
						std::cout << "dcc_stop_: " << dcc_stop_ << std::endl;
						std::cout << "ACC speed: " << iter->reference_speed << std::endl;
					}
					if (iter->reference_speed < 0){
						std::cout << iter->reference_speed << std::endl;
							iter->reference_speed = 0;
						std::cout << "-----------ACC speed 0-----------" << std::endl;
						std::cout << iter->reference_speed << std::endl;
					}

				}
			}
		}
		/*----------------------------------------------------------------------*/

		/* shinwoo ------------ AEB Here ---------------------------------------*/
		// if (present_node_ == 12){
		// 	for(iter=ref_roadinfo->roadinfo.end()-1;iter!=ref_roadinfo->roadinfo.begin();--iter){

		// 		if (true)
		// 		{
		// 			if (obstacle_dist_ < dcc_start_){
		// 				std::cout << "-----------AEB ON-----------" << std::endl;
		// 				iter->reference_speed = 35/48 * (obstacle_dist_ - dcc_stop_);
		// 			}
		// 			if (iter->reference_speed < 0){
		// 				std::cout << "-----------AEB speed 0-----------" << std::endl;
		// 				iter->reference_speed = 0;
		// 			}
		// 		}
		// 	}
		// }
		/*----------------------------------------------------------------------*/
		// int iter_count = 0;
		// for(iter=ref_roadinfo->roadinfo.begin();iter!=ref_roadinfo->roadinfo.end();iter++){
		// 	iter_count++;
		// 	if (iter->reference_speed < 50 && iter->reference_speed >30){
		// 		std::cout << "Road " << iter_count << " reference_speed before pub: " << iter->reference_speed << std::endl;
		// 	}
		// }

		dist_data.data = obstacle_dist_;

		//--- Publish
		dist_pub_.publish(dist_data);
		ref_roadinfo_pub_.publish(ref_roadinfo);

		reference_speed_->header.stamp = ros::Time::now();
		reference_speed_->header.frame_id = "odom";
		ref_speed_profile_pub_.publish(reference_speed_);

		reference_speed_new_->header.stamp = ros::Time::now();
		reference_speed_new_->header.frame_id = "odom";
		ref_speed_profile_pub2_.publish(reference_speed_new_);
	}
	/*
    void ObstacleDistCallback(const std_msgs::Float32ConstPtr &msg){
        obstalce_dist_ = msg->data;
    }*/

	void VehicleStateCallback(const pharos_msgs::StateStamped2016ConstPtr& msg){
		VehicleSpeed_ = msg->state.velocity; //[m/s]

		/*-----virtual----가상 차량 위치 마커 ; 나중에 삭제하셈---*/
		object_box_.header.frame_id = "vehicle_frame";
		object_box_.header.stamp = msg->header.stamp;
		object_box_.ns = "object_box";
		object_box_.id = 0;
		object_box_.type = visualization_msgs::Marker::CUBE;
		object_box_.action = visualization_msgs::Marker::ADD;
        object_box_.pose.position.x = obstacle_dist_;
		object_box_.pose.position.y = 0;
		object_box_.pose.position.z = 0;
		object_box_.pose.orientation.x = 0.0;
		object_box_.pose.orientation.y = 0.0;
		object_box_.pose.orientation.z = 0.0;
		object_box_.pose.orientation.w = 1.0;
		object_box_.scale.x = 2;
		object_box_.scale.y = 2;
		object_box_.scale.z = 2;
        object_box_.color.r = 1.0;
        object_box_.color.g = 0.0;
        object_box_.color.b = 0.0;
        object_box_.color.a = 0.7;
		//only if using a MESH_RESOURCE marker type:
		virtual_object_pub_.publish(object_box_);
		/*------------------------------*/
	}

	void CurrentRoadInfoCallback(const pharos_path_planner::RoadInfoConstPtr& msg){
		unsigned int wp_idx = msg->wp_index;
		SpeedPlanning(wp_idx);
	}

	void SpeedPlanning(const unsigned int current_wp_index){

		if(reference_speed_->poses.empty()){
			ROS_ERROR("No reference road information");
			return;
		}

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

		speed_limit = reference_speed_->poses[current_wp_index].pose.position.z;

		//speed limitation---------
		speed_limit *= SpeedScale_;

		if(speed_limit > MaxSpeedLimit_){
			speed_limit = MaxSpeedLimit_;
		}
		// if(speed_limit < MinSpeedLimit_){
		// 	speed_limit = MinSpeedLimit_;
		// }

		/*
        if(obstalce_dist_ != 1000){
            speed_limit = -VehicleSpeed_/obstalce_dist_*0.1+VehicleSpeed_;
            speed_limit *= 3.6;
            if(speed_limit<0)
                speed_limit=0;
        }*/

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
		pnode_->param("max_speed_limit", MaxSpeedLimit_, 100.0); //[km/h]
		pnode_->param("min_speed_limit", MinSpeedLimit_, 0.0); //[km/h]
		pnode_->param("speed_scale", SpeedScale_, 1.0);
        pnode_->param("longitudinal_acc", LonAcc_, 0.7); //[m/s^2]
        pnode_->param("lateral_acc", LatAcc_, 0.15); //[m/s^2]

        pnode_->param<float>("dcc_stop", dcc_stop_, 18);
        pnode_->param<float>("dcc_start", dcc_start_, 40);

		speed_cmd_pub_ = node_->advertise<pharos_msgs::SpeedCommand>("speed_command/reference", 10);
		dist_pub_ = node_->advertise<std_msgs::Float32>("objectbox_dist", 10);

		ref_speed_profile_pub_ = node_->advertise<nav_msgs::Path>("/path/ref_speed_profile_origin", 1);
		ref_speed_profile_pub2_ = node_->advertise<nav_msgs::Path>("/path/ref_speed_profile", 1);
		ref_roadinfo_pub_ = node_->advertise<pharos_path_planner::ReferencePath>("/ref_road_info_with_speed_profile", 1);
		virtual_object_pub_ = node_->advertise<visualization_msgs::Marker>("/virtual_objectbox", 10);


		ref_roadinfo_sub_ = node_->subscribe("/reference_road_info/", 10, &CSpeedPlanner::ReferenceRoadInfoCallback, this);
		current_roadinfo_sub_ = node_->subscribe("/current_road_info", 10, &CSpeedPlanner::CurrentRoadInfoCallback, this);
		vehicle_sub_ = node_->subscribe("/vehicle/state2016", 100, &CSpeedPlanner::VehicleStateCallback, this);
        //obstacle_dist_sub_ = node_->subscribe("/NEAR_OBJECT_DISTANCE",10, &CSpeedPlanner::ObstacleDistCallback, this);
		current_node_sub_ = node_->subscribe("current_node",10, &CSpeedPlanner::CurrentNodeCallback, this);//shinwoo
		// object_box_sub_ = node_->subscribe("/PHAHROS/perceptopn/LiDAR/objectbox",10, &CSpeedPlanner::ObjectBoxCallback, this);//shinwoo
		object_box_sub_ = node_->subscribe("/lidar_pcl/obstacle/detectedBOX_Py",10, &CSpeedPlanner::ObjectBoxCallback, this);//shinwoo
		// virtual_dist_sub_ = node_->subscribe("/virtual_distance",10, &CSpeedPlanner::VirtualDistanceCallback, this);//shinwoo
		//virtualobject_box_sub_ = node_->subscribe("/virtual_object",10, &CSpeedPlanner::VirtualObjectBoxCallback, this);//shinwoo
		
		
		//Global Variables Initialization

		// current_wp_index_ = 0;

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

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <pharos_road_information/RoadNetworks.h>
#include <pharos_road_information/Road.h>
#include <pharos_road_information/Lane.h>
#include <pharos_road_information/Lanes.h>
#include <pharos_road_information/Waypoint.h>

#include <pharos_path_planner/RoadInfo.h>
#include <pharos_path_planner/ReferencePath.h>

#include <pharos_behavior_planner/DrivingState.h>
#include <pharos_behavior_planner/StopLane.h>

#define MAX_COST 99999 //Initial cost for edges

#define DistCost 101 // Distance-based path finding
#define TimeCost 102 // Time-bsed path finding

#define NumWpSearch 30

#define Road_Normal 1
#define Road_Junction 2
#define Road_Zone 3
#define Road_U_Turn 4

//Driving Situation
#define SITUATION_LANE_DRIVING 1
#define SITUATION_INTERSECTION 2
#define SITUATION_PASSENGER_PICKUP 3

//Driving Action - Lane driving
#define NORMAL_DRIVING 1
#define LANECHANGING 2
#define SLOW_DOWN 3
#define STOP 4

//Driving Action - Intersection
#define APPROACH 1
#define STOPPING 2
#define STOPPED 3
#define PASS 4



using namespace pharos_road_information;

nav_msgs::PathPtr final_path_(new nav_msgs::Path);
nav_msgs::PathPtr final_path_speed_limit_(new nav_msgs::Path);
nav_msgs::PathPtr final_path_curvature_(new nav_msgs::Path);

RoadNetworksPtr final_road_(new RoadNetworks);
LanesPtr final_lanes_(new Lanes);

pharos_path_planner::ReferencePath reference_road_info_;

pharos_behavior_planner::DrivingState DrivingState_;
pharos_behavior_planner::StopLane StopLane_;

class GlobalPathPlanning
{
public:

	std::string odom_topic_name_;
	std::vector<std::vector<double> > road_graph_;

	int32_t publish_rate_;

	ros::NodeHandlePtr node_;
	ros::NodeHandlePtr pnode_;

	ros::Publisher global_path_pub_;
	ros::Publisher global_path_speed_pub_;
	ros::Publisher global_path_curvature_pub_;
	ros::Publisher reference_road_info_pub_;
	ros::Publisher current_road_info_pub_;
	ros::Publisher current_pose_with_waypoint_pub_;
	ros::Publisher reference_lanes_pub_;
	ros::Publisher global_path_update_pub_;

	ros::Subscriber vehicle_odom_sub_;
	ros::Subscriber road_net_graph_sub_;
	ros::Subscriber destination_sub_;
	ros::Subscriber global_path_planning_req_sub_;

	ros::Subscriber DrivingState_sub_;
	ros::Subscriber StopLane_sub_;

	RoadNetworks Road_Networks_;

	unsigned int number_of_nodes_;
	unsigned int number_of_roads_;
	unsigned int number_of_lanes_;
	unsigned int current_wp_idx_;
	unsigned int current_road_;
	unsigned int current_lane_;

	std::vector<std::vector <Lane>  > lane_graph_; 
	std::vector<std::vector <double> > speed_limit_graph_;
	std::vector<std::vector <int> > road_type_graph_;
	std::vector<std::vector <int> > road_action_graph_;

	geometry_msgs::PoseStamped destination_pose_;
	geometry_msgs::PoseStamped start_pose_;

	geometry_msgs::PoseStamped vehicle_pose_;

	int global_path_planning_request_;

	bool manual_goal_setting_;
	bool new_destination_input_;
	bool new_global_path_generation_;

	bool graph_build_;
	bool block_lanes_;

	unsigned int source_node_ ;
	unsigned int destination_node_;

	struct Node
	{
		double cost; // Can be distance or traveling time(dist/vel).
		bool visited; // 'True' means already be visited(checked) and 'False' means not check yet.
		int parent_node; // Parent node of this node. It is updated when minimum cost for this is updated.
	};

	void GlobalPathPlanningRequestCallback(const std_msgs::Int32ConstPtr& msg){
		global_path_planning_request_ = msg->data;
	}
	void SourceAndDestinationCallback(){

	}

	void DrivingStateCallback(const pharos_behavior_planner::DrivingStateConstPtr &msg){
		DrivingState_ = *msg;
	}
	void StopLaneCallback(const pharos_behavior_planner::StopLaneConstPtr &msg){
		StopLane_ = *msg;
	}

	void DestinationCallback(const geometry_msgs::PoseStampedConstPtr& msg){
		destination_pose_ = *msg;
		// printf("x: %.3f, y: %.3f\n",destination_pose_.pose.position.x, destination_pose_.pose.position.y);
		manual_goal_setting_ = true;
		new_destination_input_ = true;
		global_path_planning_request_ = 1;
	}


	void CurrentVehiclePositionCallback(const nav_msgs::OdometryConstPtr& msg){
		start_pose_.header = msg->header;
		start_pose_.pose = msg->pose.pose;

		vehicle_pose_.header = msg->header;
		vehicle_pose_.pose = msg->pose.pose;
	}

	void RoadNetworkGraphCallback(const pharos_road_information::RoadNetworksConstPtr& msg){

		// if(!global_path_planning_request_) return;
		printf("road graphdddddd callbackddd\n");

		static unsigned int previous_end_node = 0;

		if(!graph_build_){
			
			number_of_roads_ = msg->roads.size();
			number_of_lanes_ = 0;

			for(unsigned int i=0;i<number_of_roads_;i++){
				number_of_lanes_ += msg->roads[i].lanes.size();
			}

			for(unsigned int i=0;i<number_of_roads_;i++){
				int number_of_lanes = msg->roads.at(i).lanes.size();
				for(unsigned int j=0;j<number_of_lanes;j++){
					if(msg->roads.at(i).lanes.at(j).end_node > number_of_nodes_){
						number_of_nodes_ = msg->roads.at(i).lanes.at(j).end_node;
					}
				}
			}
			number_of_nodes_++;

			

			road_graph_.resize(number_of_nodes_);
			std::vector<std::vector <double> >::iterator iter1;
			for(iter1=road_graph_.begin();iter1 != road_graph_.end();++iter1){
				iter1->resize(number_of_nodes_);
			}


			// Graph Build
			for(unsigned int i=0;i<number_of_roads_;i++){
				int number_of_lanes = msg->roads.at(i).lanes.size();
				for(unsigned int j=0;j<number_of_lanes;j++){
					Lane lane_info = msg->roads[i].lanes[j];
					int begin_node = lane_info.begin_node;
					int end_node = lane_info.end_node;

					// Cost setting
					road_graph_[begin_node][end_node] = msg->roads[i].distance; // Using distance for the cost
					// road_graph_[begin_node][end_node] = msg->roads[i].distance/msg->roads[i].speed_limit; // Using traveling time for the cost
				}
				if(number_of_lanes>1 && msg->roads[i].road_type == Road_Normal){
					// Set cost from (i) lane's end-node to (i+1) lanes begin node as almost zero
					for(unsigned int j=0;j<number_of_lanes-1;j++){
						int current_end_node = msg->roads[i].lanes[j].end_node;
						int next_begin_node = msg->roads[i].lanes[j+1].begin_node;
						// road_graph_[current_end_node][next_begin_node] = 1.0/msg->roads[i].distance; // None-zero small cost
						road_graph_[current_end_node][next_begin_node] = -1.0 * msg->roads.at(i).distance + 1.0/msg->roads[i].distance;
					}
					for(unsigned int j=1;j<number_of_lanes;j++){
						int current_end_node = msg->roads[i].lanes[j].end_node;
						int next_begin_node = msg->roads[i].lanes[j-1].begin_node;
						// road_graph_[current_end_node][next_begin_node] = 1.0/msg->roads[i].distance; // None-zero small cost
						road_graph_[current_end_node][next_begin_node] = -1.0 * msg->roads.at(i).distance + 1.0/msg->roads[i].distance;
					}
				}
			}


			lane_graph_.resize(number_of_nodes_);
			std::vector<std::vector <Lane> >::iterator iter2;
			for(iter2=lane_graph_.begin();iter2 != lane_graph_.end();++iter2){
				iter2->resize(number_of_nodes_);
			}

			for(unsigned int i=0;i<number_of_roads_;i++){
				int number_of_lanes = msg->roads.at(i).lanes.size();
				for(unsigned int j=0;j<number_of_lanes;j++){
					Lane lane_info = msg->roads[i].lanes[j];
					int begin_node = lane_info.begin_node;
					int end_node = lane_info.end_node;

					lane_graph_[begin_node][end_node] = lane_info;
				}
			}

			speed_limit_graph_.resize(number_of_nodes_);
			std::vector<std::vector <double> >::iterator iter_speed;
			for(iter_speed=speed_limit_graph_.begin();iter_speed != speed_limit_graph_.end();++iter_speed){
				iter_speed->resize(number_of_nodes_);
			}
			
			for(unsigned int i=0;i<number_of_roads_;i++){
				int number_of_lanes = msg->roads.at(i).lanes.size();
				for(unsigned int j=0;j<number_of_lanes;j++){
					Lane lane_info = msg->roads[i].lanes[j];
					int begin_node = lane_info.begin_node;
					int end_node = lane_info.end_node;

					speed_limit_graph_[begin_node][end_node] = msg->roads[i].speed_limit;
				}
			}

			
			road_type_graph_.resize(number_of_nodes_);
			std::vector<std::vector <int> >::iterator iter_type;
			for(iter_type=road_type_graph_.begin();iter_type != road_type_graph_.end();++iter_type){
				iter_type->resize(number_of_nodes_);
			}

			for(unsigned int i=0;i<number_of_roads_;i++){
				int number_of_lanes = msg->roads.at(i).lanes.size();
				for(unsigned int j=0;j<number_of_lanes;j++){
					Lane lane_info = msg->roads[i].lanes[j];
					int begin_node = lane_info.begin_node;
					int end_node = lane_info.end_node;

					road_type_graph_[begin_node][end_node] = msg->roads[i].road_type;
				}
			}

			road_action_graph_.resize(number_of_nodes_);
			std::vector<std::vector <int> >::iterator iter_action;
			for(iter_action=road_action_graph_.begin();iter_action != road_action_graph_.end();++iter_action){
				iter_action->resize(number_of_nodes_);
			}

			for(unsigned int i=0;i<number_of_roads_;i++){
				int number_of_lanes = msg->roads.at(i).lanes.size();
				for(unsigned int j=0;j<number_of_lanes;j++){
					Lane lane_info = msg->roads[i].lanes[j];
					int begin_node = lane_info.begin_node;
					int end_node = lane_info.end_node;

					road_action_graph_[begin_node][end_node] = msg->roads[i].action;
				}
			}

			graph_build_ = true;
		}
		
		//---- Dijkstra Graph Search
		std::vector<int> solution_nodes;

		if(graph_build_){
			printf("current_road: %d, current_lane: %d\n",current_road_, current_lane_);
			// if(current_road_ > 0 && current_lane_ > 0){
			// 	source_node_ = msg->roads.at(current_road_-1).lanes.at(current_lane_-1).begin_node;
			// }
			printf("source node : %d\n",source_node_);
		} 



		// Start Node Setting
		if(global_path_planning_request_ == 1){
			tf::Point p_start, p_wp;
			p_start.setValue(start_pose_.pose.position.x,start_pose_.pose.position.y,0.0);

			std::vector<Waypoint> waypoints;
			std::vector<Waypoint>::iterator wp_iter;

			double min_dist_start = DBL_MAX;

			unsigned int starting_road = 0;
			unsigned int starting_lane = 0;

			tf::Pose pose;
	        tf::poseMsgToTF(vehicle_pose_.pose, pose);
	        double Roll,Pitch, Yaw;
	        static double vehicle_heading;
	        pose.getBasis().getRPY(Roll, Pitch, Yaw);
	        vehicle_heading = Yaw;

			for(unsigned int i=0;i<number_of_roads_;i++){
				int number_of_lanes = msg->roads.at(i).lanes.size();
				for(unsigned int j=0;j<number_of_lanes;j++){
					waypoints = msg->roads.at(i).lanes.at(j).waypoints;
					unsigned int wp_idx = 0;
					for(wp_iter = waypoints.begin();wp_iter != waypoints.end()-1;++wp_iter){
						p_wp.setValue(wp_iter->x,wp_iter->y,0.0);
						double dist_start = p_start.distance(p_wp);

						double path_heading;

				        double x0 = wp_iter->x;
				        double y0 = wp_iter->y;
				        double x1 = (*(wp_iter+1)).x;
				        double y1 = (*(wp_iter+1)).y;

				        path_heading = atan2(y1-y0,x1-x0);
						
						double heading_err = path_heading - vehicle_heading;
			            if (heading_err > M_PI) {
			                heading_err = heading_err - 2 * M_PI;
			            }
			            else if (heading_err < -M_PI) {
			                heading_err = heading_err + 2 * M_PI;
			            }
			            

						if(dist_start < min_dist_start && fabs(heading_err) < M_PI/4){
							min_dist_start = dist_start;
							starting_road = i;
							starting_lane = j;
						}
						wp_idx++;
					}
				}
			}
			printf("source node update!\n");
			source_node_ = msg->roads.at(starting_road).lanes.at(starting_lane).begin_node;
		}
		// if(global_path_planning_request_ == 2){

		// }

		// Blcok Lane Changing
		if(global_path_planning_request_ == 2){ 
			block_lanes_ = true;
			source_node_ = msg->roads.at(current_road_-1).lanes.at(current_lane_-1).begin_node;
			int number_of_lanes = msg->roads.at(current_road_-1).lanes.size();
			printf("-----Planning again because of lane changing failure----\n");

			for(unsigned int i=0;i<number_of_lanes-1;i++){
				int current_end_node = msg->roads[current_road_-1].lanes[i].end_node;
				int next_begin_node = msg->roads[current_road_-1].lanes[i+1].begin_node;
				road_graph_[current_end_node][next_begin_node] = 0;
			}
			for(unsigned int i=1;i<number_of_lanes;i++){
				int current_end_node = msg->roads[current_road_-1].lanes[i].end_node;
				int next_begin_node = msg->roads[current_road_-1].lanes[i-1].begin_node;
				road_graph_[current_end_node][next_begin_node] = 0;
			}
		}

		// Finish Node setting
		static unsigned int finish_wp_index = 0;

		if(new_destination_input_){
			tf::Point p_goal, p_wp;
			p_goal.setValue(destination_pose_.pose.position.x,destination_pose_.pose.position.y,0.0);

			std::vector<Waypoint> waypoints;
			std::vector<Waypoint>::iterator wp_iter;

			double min_dist_goal = DBL_MAX;

			unsigned int finishing_road = 0;
			unsigned int finishing_lane = 0;

			for(unsigned int i=0;i<number_of_roads_;i++){
				int number_of_lanes = msg->roads.at(i).lanes.size();
				for(unsigned int j=0;j<number_of_lanes;j++){
					waypoints = msg->roads.at(i).lanes.at(j).waypoints;
					unsigned int wp_idx = 0;
					for(wp_iter = waypoints.begin();wp_iter != waypoints.end();++wp_iter){
						p_wp.setValue(wp_iter->x,wp_iter->y,0.0);
						double dist_goal = p_goal.distance(p_wp);
						
						if(dist_goal < min_dist_goal){
							min_dist_goal = dist_goal;
							finishing_road = i;
							finishing_lane = j;
							finish_wp_index = wp_idx;
						}
						wp_idx++;
					}
				}
			}
			destination_node_ = msg->roads.at(finishing_road).lanes.at(finishing_lane).end_node;
			new_destination_input_ = false;
		}

		DijkstraGraphSearch(source_node_,destination_node_,&road_graph_,&solution_nodes);

		std::reverse(solution_nodes.begin(),solution_nodes.end());
		printf("[Global Path]\n");
		for(unsigned int i=0;i<solution_nodes.size()-1;i++){
			printf("%d->",solution_nodes[i]);
		}
		printf("%d\n",solution_nodes.back());
		printf("\n");


		//--- Publish Final Global Path



		final_lanes_->lanes.clear();
		std::vector<int>::iterator iter_int;
		for(iter_int=solution_nodes.begin();iter_int != solution_nodes.end()-1;++iter_int){
			int begin_node;
			int end_node;

			begin_node = *iter_int;
			end_node = *(iter_int+1);
			if(road_graph_[begin_node][end_node] < 1.0)continue;

			Lane lane = lane_graph_[begin_node][end_node];
			final_lanes_->lanes.push_back(lane);
		}
		reference_lanes_pub_.publish(final_lanes_);

		final_path_->poses.clear();
		final_path_curvature_->poses.clear();
		final_path_speed_limit_->poses.clear();
		reference_road_info_.roadinfo.clear();

		reference_road_info_.traveling_time = 0.0;
		reference_road_info_.distance = 0.0;

		std::vector<int>::iterator iter3;

		unsigned int idx = 1;
		for(iter3=solution_nodes.begin();iter3 != solution_nodes.end()-1;++iter3){
			int begin_node;
			int end_node;

			begin_node = *iter3;
			end_node = *(iter3+1);

			std::vector<Waypoint> waypoints = lane_graph_[begin_node][end_node].waypoints;
			if(waypoints.empty()) continue;

			std::vector<Waypoint>::iterator iter;
			geometry_msgs::PoseStamped wp;
			geometry_msgs::PoseStamped wp_next;
			geometry_msgs::PoseStamped curv;
			geometry_msgs::PoseStamped speed_limit_pose;

			pharos_path_planner::RoadInfo road_info;
			double speed_limit = speed_limit_graph_[begin_node][end_node];
			unsigned int road_number = lane_graph_[begin_node][end_node].road_number;
			unsigned int lane_number = lane_graph_[begin_node][end_node].lane_number;
			int action = road_action_graph_[begin_node][end_node];
			int road_type = road_type_graph_[begin_node][end_node];
			unsigned int wp_idx = 0;
			unsigned int delete_wp = 4;

			for(iter = waypoints.begin()+delete_wp;iter != waypoints.end()-delete_wp;++iter){

				wp.pose.position.x = (*iter).x;
				wp.pose.position.y = (*iter).y;
				// wp.pose.position.z = (*iter).z;
				wp.pose.position.z = 0.0;

				if(iter == waypoints.end()-1){
					wp.pose.orientation = (final_path_->poses.back()).pose.orientation;
				}
				else{
					wp_next.pose.position.x = (*(iter+1)).x;
					wp_next.pose.position.y = (*(iter+1)).y;
					wp_next.pose.position.z = (*(iter+1)).z;

					geometry_msgs::Quaternion quat;
					tf::Vector3 vector3;
					vector3 = VectorBetweenTwoPoses(wp,wp_next);
					VectorToQuat(vector3,quat);
					wp.pose.orientation = quat;

					// printf("wp, x:%.3f, y:%.3f, z:%.3f\n",wp.pose.position.x,wp.pose.position.y,wp.pose.position.z);
					// printf("wp_next, x:%.3f, y:%.3f, z:%.3f\n",wp_next.pose.position.x,wp_next.pose.position.y,wp_next.pose.position.z);
					// printf("vector3, x:%.3f, y:%.3f, z:%.3f\n",vector3.x(),vector3.y(),vector3.z());
				}

				curv.pose.position.x = (*iter).x;
				curv.pose.position.y = (*iter).y;
				curv.pose.position.z = (*iter).curvature;

				speed_limit_pose.pose.position.x = (*iter).x;
				speed_limit_pose.pose.position.y = (*iter).y;
				speed_limit_pose.pose.position.z = speed_limit;

				final_path_->poses.push_back(wp);
				final_path_curvature_->poses.push_back(curv);
				final_path_speed_limit_->poses.push_back(speed_limit_pose);

				road_info.position.x = (*iter).x;
				road_info.position.y = (*iter).y;
				// road_info.position.z = (*iter).z;
				road_info.position.z = 0.0;
				road_info.curvature = (*iter).curvature;
				road_info.speed_limit = speed_limit;
				road_info.reference_speed = speed_limit;
				road_info.road_number = road_number;
				road_info.lane_number = lane_number;
				road_info.action = action;
				road_info.road_type = road_type;
				road_info.lane_index = idx;
				
				// Last Waypoint
				if(manual_goal_setting_ && end_node == solution_nodes.back()){
					if(wp_idx == finish_wp_index){
						printf("final waypoint\n");
						break;
					}
				}

				reference_road_info_.roadinfo.push_back(road_info);

				wp_idx++;
				idx++;
			}
		}

		printf("global_waypoint size: %d\n",final_path_->poses.size());
		printf("ref_roadinfo size: %d\n",reference_road_info_.roadinfo.size());


		if(DrivingState_.situation == SITUATION_INTERSECTION && DrivingState_.action == STOPPING){
			int i = DrivingState_.stop_wp_idx;
			printf("stop_wp_idx: %d\n",i);
			tf::Point p1,p2;
			double d = 0.0;
			while(1){
				reference_road_info_.roadinfo.at(i).speed_limit = 0.0;
				p1.setValue(reference_road_info_.roadinfo.at(i).position.x,reference_road_info_.roadinfo.at(i).position.y,0.0);
				i--;
				p2.setValue(reference_road_info_.roadinfo.at(i).position.x,reference_road_info_.roadinfo.at(i).position.y,0.0);
				d += p1.distance(p2);
				if(d>4.7) break;
				if(i == 0) break;
			}
		}

		reference_road_info_.roadinfo.back().reference_speed = 0.0;

		reference_road_info_pub_.publish(reference_road_info_);

		final_path_->header.stamp = ros::Time::now();
		final_path_->header.frame_id = "odom";
		global_path_pub_.publish(final_path_);


		final_path_curvature_->header.stamp = ros::Time::now();
		final_path_curvature_->header.frame_id = "odom";
		global_path_curvature_pub_.publish(final_path_curvature_);

		final_path_speed_limit_->header.stamp = ros::Time::now();
		final_path_speed_limit_->header.frame_id = "odom";
		global_path_speed_pub_.publish(final_path_speed_limit_);


		if(global_path_planning_request_ != 0){
			std_msgs::Bool global_path_update;
			global_path_update.data = true;
			global_path_update_pub_.publish(global_path_update);
		}


		static unsigned int old_idx = 0;
		if(new_global_path_generation_){
			printf("new_destination_input!\n");
			old_idx = 0;
			new_global_path_generation_ = false;
		}

		if(global_path_planning_request_ == 1 || global_path_planning_request_ == 2){
			printf("global path plannig is requested\n");
			old_idx = 0;
		}

		// unsigned int current_wp_idx = NearestWaypointIdx(final_path_, vehicle_pose_, old_idx);
		unsigned int current_wp_idx = 0;
		tf::Point p_vehicle, p_wp;
    	p_vehicle.setValue(vehicle_pose_.pose.position.x, vehicle_pose_.pose.position.y, 0.0);

    	double min_dist = DBL_MAX;
	    unsigned int search_start = old_idx;
	    unsigned int search_finish = old_idx + 50;
	    if(search_finish > reference_road_info_.roadinfo.size()-2){
	        search_finish = reference_road_info_.roadinfo.size()-1;
	    }
	    if(old_idx == 0) search_finish = reference_road_info_.roadinfo.size()-1;
	    for(unsigned int i=search_start;i<search_finish;i++){
	        p_wp.setValue(reference_road_info_.roadinfo.at(i).position.x, reference_road_info_.roadinfo.at(i).position.y,0.0);
	        int road_number = reference_road_info_.roadinfo.at(i).road_number;
	        int lane_number = reference_road_info_.roadinfo.at(i).lane_number;
	        int begin_node = msg->roads.at(road_number-1).lanes.at(lane_number-1).begin_node;
	        double dist = p_vehicle.distance(p_wp);
	        if(dist<min_dist){
	        	
					min_dist = dist;
	            	current_wp_idx = i;
	        	
	        }
	    }
	    // current_wp_idx = wp_idx;


		old_idx = current_wp_idx;

		pharos_path_planner::RoadInfo road_info;
		
		road_info = reference_road_info_.roadinfo.at(current_wp_idx);
		road_info.wp_index = current_wp_idx;
		current_road_info_pub_.publish(road_info);

		current_road_ = road_info.road_number;
		current_lane_ = road_info.lane_number;

        geometry_msgs::PoseStamped current_pose_with_waypoint;

        current_pose_with_waypoint.pose = final_path_->poses.at(current_wp_idx).pose;

        current_pose_with_waypoint.header.frame_id = vehicle_pose_.header.frame_id;
        current_pose_with_waypoint.header.stamp = vehicle_pose_.header.stamp;
        current_pose_with_waypoint_pub_.publish(current_pose_with_waypoint);

        static int old_road = current_road_;
        static int old_lane = current_lane_;



        if(block_lanes_ && road_info.road_type == 1 && old_road != current_road_){
        	ROS_INFO("old_road: %d, current_road : %d",old_road, current_road_);
        	printf("block lanes: %d, road_type: %d\n",block_lanes_,road_info.road_type);
        	if(current_road_ > 0 && current_lane_ > 0){
				source_node_ = msg->roads.at(current_road_-1).lanes.at(current_lane_-1).begin_node;
			}
			old_idx = 0;
        	graph_build_ = false;
        	block_lanes_ = false;
        }
        global_path_planning_request_ = 0;
        // printf("global path planning: %d\n",global_path_planning_request_);

        old_road = current_road_;
		old_lane = current_lane_;
		previous_end_node = msg->roads.at(current_road_-1).lanes.at(current_lane_-1).end_node;
		// new_global_path_generation_ = true;
	}

	void DijkstraGraphSearch(const unsigned int start_node, const unsigned int finish_node, std::vector<std::vector<double> > *Graph, std::vector<int>* solution_nodes){

		
		Node node_init_value;
		node_init_value.cost = MAX_COST;
		node_init_value.visited = false;
		node_init_value.parent_node = -1;

		std::vector<Node> nodes(Graph->size(),node_init_value);
		std::vector<int> neighbor_nodes;
		

		unsigned int current_node = start_node;
		nodes[current_node].cost = 0.0;
		nodes[current_node].visited = true;

		// Dijkstra Algorithm
		bool finish = false;
		while(!finish){

			// Add neighbor nodes
			neighbor_nodes.clear();
			std::vector<double>::iterator iter1;
			int j=0;
			for(iter1 = Graph->at(current_node).begin();iter1!=Graph->at(current_node).end();iter1++){
				if(*iter1 != 0 && !(nodes[j].visited)){
					neighbor_nodes.push_back(j);
				}
				j++;
			}

			std::vector<int>::iterator iter_open;
			for(iter_open = neighbor_nodes.begin();iter_open != neighbor_nodes.end(); iter_open++){
				double dist = Graph->at(current_node).at(*iter_open);
				if(nodes[current_node].cost+dist < nodes[*iter_open].cost){
					nodes[*iter_open].cost = nodes[current_node].cost+dist;
					nodes[*iter_open].parent_node = current_node;
				}	
			}

			// Find new current node which has minimum cost in unvisited nodes. 
			double min_dist = DBL_MAX;
			std::vector<Node>::iterator iter_node;
			unsigned int node_num = 0;
			for(iter_node = nodes.begin();iter_node != nodes.end(); iter_node++){
				if(!((*iter_node).visited) && (*iter_node).cost < min_dist){
					min_dist = (*iter_node).cost;
					current_node = node_num;
				}
				node_num++;
			}
			if(min_dist == MAX_COST){
				ROS_ERROR("Cannot Find the Path :(");
				break;
			}

			// Make current node visited
			nodes[current_node].visited = true;


			// Finish Condition
			if(current_node == finish_node){
				finish = true;
				ROS_INFO("Success to Find the Path!");
			}
		}

		// Final Solution Nodes
		std::vector<int> solution;

		if(finish)
		solution.push_back(finish_node);
		unsigned int c_node = finish_node;
		while(c_node != start_node){
			int parent_node = nodes[c_node].parent_node;
			solution.push_back(parent_node);
			c_node = parent_node;
			if(c_node == -1){
				ROS_ERROR("Error....");
				break;
			}
		}
		*solution_nodes = solution;
	}

    unsigned int NearestWaypointIdx(const nav_msgs::PathConstPtr &path,const geometry_msgs::PoseStamped vehicle_pose, const unsigned int old_idx){
        unsigned int idx = 0;

        tf::Point p1,p2;
        p1.setValue(vehicle_pose.pose.position.x,vehicle_pose.pose.position.y,0.0);
        double min_dist = DBL_MAX;

        unsigned long path_length = path->poses.size();
        unsigned long search_start_idx = 0;
        unsigned long search_finish_idx = 0;

        if(old_idx == 0){
            search_start_idx = 0;
            search_finish_idx = path_length-1;
        }
        else{
            search_start_idx = old_idx;
            search_finish_idx = search_start_idx + NumWpSearch;
        }

        tf::Pose pose;
        tf::poseMsgToTF(vehicle_pose.pose, pose);
        double Roll,Pitch, Yaw;
        static double vehicle_heading;
        pose.getBasis().getRPY(Roll, Pitch, Yaw);
        vehicle_heading = Yaw;

        for(unsigned int i=search_start_idx; i<search_finish_idx;i++){
            p2.setValue(path->poses.at(i%path_length).pose.position.x,path->poses.at(i%path_length).pose.position.y,0.0);
            double dist = p1.distance(p2);

            double heading_err = PathHeading(path,i%path_length)-vehicle_heading;
            if (heading_err > M_PI) {
                heading_err = heading_err - 2 * M_PI;
            }
            else if (heading_err < -M_PI) {
                heading_err = heading_err + 2 * M_PI;
            }
            if (dist < min_dist && fabs(heading_err) < M_PI/2){
            //if(dist < min_dist){
                min_dist = dist;
                idx = (unsigned int)(i%path_length);
            }
        }
        return idx;
    }

    inline double PathHeading(const nav_msgs::PathConstPtr &path, const unsigned int idx){
        double heading;
        unsigned long path_length = path->poses.size();

        double x0 = path->poses.at(idx).pose.position.x;
        double y0 = path->poses.at(idx).pose.position.y;
        double x1 = path->poses.at((idx+1)%path_length).pose.position.x;
        double y1 = path->poses.at((idx+1)%path_length).pose.position.y;

        heading = atan2(y1-y0,x1-x0);

        return heading;
    }

    void VectorToQuat(const tf::Vector3 vector3, geometry_msgs::Quaternion &pose_quaternion){
    	
    	double roll = 0.0;
    	double pitch = 0.0;
    	double yaw = 0.0;

    	// roll = atan2(vector3.z(),vector3.y());
    	// pitch = atan2(vector3.x(),vector3.z());
    	yaw = atan2(vector3.y(),vector3.x());

        tf::Quaternion q;
        q.setRPY(roll,pitch,yaw);
        tf::quaternionTFToMsg(q,pose_quaternion);
    }
    
    inline tf::Vector3 VectorBetweenTwoPoses(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2){

        tf::Vector3 vector3;

        double dx = pose2.pose.position.x - pose1.pose.position.x;
        double dy = pose2.pose.position.y - pose1.pose.position.y;
        double dz = pose2.pose.position.z - pose1.pose.position.z;

        vector3.setValue(dx,dy,dz);

        return vector3;
    }

	// Load parameters etc
	int init()
	{
		
		node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
		pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

		pnode_->param("publish_rate", publish_rate_, 5);
		pnode_->getParam("odom_topic_name",odom_topic_name_);

		global_path_pub_ = node_->advertise<nav_msgs::Path>(std::string("/path/reference_path"), 10);
		global_path_speed_pub_ =  node_->advertise<nav_msgs::Path>(std::string("/path/reference_speed"), 10);
		global_path_curvature_pub_ =  node_->advertise<nav_msgs::Path>(std::string("/path/reference_curvature"), 10);

        current_pose_with_waypoint_pub_ = node_->advertise<geometry_msgs::PoseStamped>("/pose_on_the_road", 10);

		reference_road_info_pub_ = node_->advertise<pharos_path_planner::ReferencePath>(std::string("/reference_road_info"), 10);
		current_road_info_pub_ = node_->advertise<pharos_path_planner::RoadInfo>(std::string("/current_road_info"), 10);

		reference_lanes_pub_ = node_->advertise<pharos_road_information::Lanes>("/reference_lanes",10);

		global_path_update_pub_ = node_->advertise<std_msgs::Bool>("global_path_update",10);

		road_net_graph_sub_ = node_->subscribe("/road_network_graph",5, &GlobalPathPlanning::RoadNetworkGraphCallback, this);
		vehicle_odom_sub_ = node_->subscribe("/odom/novatel",3, &GlobalPathPlanning::CurrentVehiclePositionCallback, this);
		destination_sub_ = node_->subscribe("/move_base_simple/goal",5, &GlobalPathPlanning::DestinationCallback, this);
		global_path_planning_req_sub_ = node_->subscribe("global_path_planning_request",5, &GlobalPathPlanning::GlobalPathPlanningRequestCallback, this);

		DrivingState_sub_ = node_->subscribe("/behavior/driving_state",5, &GlobalPathPlanning::DrivingStateCallback, this);
		StopLane_sub_ = node_->subscribe("/behavior/stoplane",5, &GlobalPathPlanning::StopLaneCallback, this);

		source_node_ = 1;
		// destination_node_ = 46;
		destination_node_ = 40;

		graph_build_ = false;
		manual_goal_setting_ = false;
		new_destination_input_ = false;
		new_global_path_generation_ = false;
		global_path_planning_request_ = 1;
		current_wp_idx_ = 0;
		current_road_ = 1;
		current_lane_ = 1;
		block_lanes_ = false;
		number_of_nodes_ = 0;


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
	ros::init(argc, argv, "GlobalPathPlanning_node");

	GlobalPathPlanning gpp_node;

	if (gpp_node.init())
	{
		ROS_FATAL("GlobalPathPlanning_node initialization failed");
		return -1;
	}

	gpp_node.publish();
	return 0;
}


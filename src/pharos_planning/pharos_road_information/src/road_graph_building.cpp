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

#include <fstream>


#include <std_msgs/Float64.h>
#include <pharos_msgs/Workzone_xy.h>
#include <pharos_msgs/Workzone_list.h>

#include <pharos_road_information/RoadNetworks.h>
#include <pharos_road_information/Road.h>
#include <pharos_road_information/Lane.h>
#include <pharos_road_information/Lanes.h>
#include <pharos_road_information/Waypoint.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


double Area1_x1 = 16.6+302455.135629;
double Area1_y1 = 151+4123701.212429;
double Area1_x2 = 0+302455.135629;
double Area1_y2 = 110+4123701.212429;
double Area1_x3 = 44+302455.135629;
double Area1_y3 = 87.9+4123701.212429;
double Area1_x4 = 31+302455.135629;
double Area1_y4 = 143+4123701.212429;

double Area2_x1 = 57.4+302455.135629;
double Area2_y1 = 129+4123701.212429;
double Area2_x2 = 45.3+302455.135629;
double Area2_y2 = 103+4123701.212429;
double Area2_x3 = 60.9+302455.135629;
double Area2_y3 = 94.5+4123701.212429;
double Area2_x4 = 76.4+302455.135629;
double Area2_y4 = 118+4123701.212429;

double Area3_x1 = 76.4+302455.135629;
double Area3_y1 = 97+4123701.212429;
double Area3_x2 = 53.9+302455.135629;
double Area3_y2 = 79.5+4123701.212429;
double Area3_x3 = 90.4+302455.135629;
double Area3_y3 = 60+4123701.212429;
double Area3_x4 = 113+302455.135629;
double Area3_y4 = 94.8+4123701.212429;

double Area4_x1 = 42.2+302455.135629;
double Area4_y1 = 165+4123701.212429;
double Area4_x2 = 31.9+302455.135629;
double Area4_y2 = 143+4123701.212429;
double Area4_x3 = 57.4+302455.135629;
double Area4_y3 = 129+4123701.212429;
double Area4_x4 = 68.1+302455.135629;
double Area4_y4 = 149+4123701.212429;

double Area5_x1 = 89.5+302455.135629;
double Area5_y1 = 140+4123701.212429;
double Area5_x2 = 76.4+302455.135629;
double Area5_y2 = 118+4123701.212429;
double Area5_x3 = 103+302455.135629;
double Area5_y3 = 101+4123701.212429;
double Area5_x4 = 115+302455.135629;
double Area5_y4 = 124+4123701.212429;

double Area6_x1 = 54.1+302455.135629;
double Area6_y1 = 217+4123701.212429;
double Area6_x2 = 29.3+302455.135629;
double Area6_y2 = 173+4123701.212429;
double Area6_x3 = 68.1+302455.135629;
double Area6_y3 = 177+4123701.212429;
double Area6_x4 = 89.6+302455.135629;
double Area6_y4 = 195+4123701.212429;

double Area7_x1 = 82.9+302455.135629;
double Area7_y1 = 176+4123701.212429;
double Area7_x2 = 68.1+302455.135629;
double Area7_y2 = 149+4123701.212429;
double Area7_x3 = 89.5+302455.135629;
double Area7_y3 = 140+4123701.212429;
double Area7_x4 = 101+302455.135629;
double Area7_y4 = 166+4123701.212429;

double Area8_x1 = 110+302455.135629;
double Area8_y1 = 183+4123701.212429;
double Area8_x2 = 89.5+302455.135629;
double Area8_y2 = 140+4123701.212429;
double Area8_x3 = 126+302455.135629;
double Area8_y3 = 118+4123701.212429;
double Area8_x4 = 150+302455.135629;
double Area8_y4 = 153+4123701.212429;



// #include <string>

using namespace pharos_road_information;

class RoadGraphBuild
{
public:


	int32_t publish_rate_;

	ros::NodeHandlePtr node_;
	ros::NodeHandlePtr pnode_;

	ros::Publisher road_net_graph_pub_;
    ros::Publisher workzone_point_pub;
    ros::Publisher global_path_planning_request_pub_;
    ros::Publisher wk_area_pub_;
    ros::Publisher workzone_bool;
    ros::Publisher click_bool_;

    ros::Subscriber workzone;
    ros::Subscriber click_point_sub_;
    ros::Subscriber mission_state_sub_;
	ros::ServiceServer service_;

	RoadNetworks Road_Networks_;
	RoadNetworks Workzone_Networks;

    std_msgs::Bool workzone_1_2_bool;
    std_msgs::Bool workzone_1_3_bool;
    std_msgs::Bool workzone_1_4_bool;
    std_msgs::Bool workzone_2_3_bool;
    std_msgs::Bool workzone_2_5_bool;
    std_msgs::Bool workzone_3_2_bool;
    std_msgs::Bool workzone_3_5_bool;
    std_msgs::Bool workzone_3_4_bool;
    std_msgs::Bool workzone_4_3_bool;
    std_msgs::Bool workzone_4_5_bool;

    std_msgs::Bool workzone_bool_;
    std_msgs::Bool click_bool;
    geometry_msgs::PointStamped clicked_point;
    bool clicked = false;

    std_msgs::Int32 mission_state_;


    std::vector<unsigned int> wk_road_number;
    std::vector<unsigned int> wk_lane_number;

    pharos_msgs::Workzone_list workzone_list;
    pharos_msgs::Workzone_list clicked_list;
    pharos_msgs::Workzone_xy clicked_xy;
    double origin_gps_x_;
	double origin_gps_y_;
	double origin_gps_z_;
	bool relative_gps_;
	bool workzone_graph_build = false;


	// Load parameters etc
	int init()
	{
		std::string wp_file_directory;

		node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
		pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

		pnode_->param("publish_rate", publish_rate_, 10);
		pnode_->param("waypoint_file_directory", wp_file_directory, std::string(""));
		road_net_graph_pub_ = node_->advertise<pharos_road_information::RoadNetworks>(std::string("/road_network_graph"), 3);
        global_path_planning_request_pub_ = node_->advertise<std_msgs::Int32>("global_path_planning_request",10);

		workzone = node_->subscribe("/wave/workzone_list",10,&RoadGraphBuild::workzone_callback,this);
        click_point_sub_ = node_->subscribe("/clicked_point",10,&RoadGraphBuild::ClickedPointCallback,this);
        mission_state_sub_ = node_->subscribe("mission_state",10,&RoadGraphBuild::MissionStateCallback,this);
        click_bool_ = node_->advertise<std_msgs::Bool>("road_graph/click_bool",10);

        workzone_bool = node_->advertise<std_msgs::Bool>("road_graph/workzone_bool",10);
        wk_area_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/workzone_area_visualization",10);
        relative_gps_ = false;
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
			ROS_INFO("Failed to get param /gps/origin/y");
			origin_gps_y_ = 0.0;
		}

		if (node_->getParam("gps/origin/z", origin_gps_z_)) {
			ROS_INFO("gps/origin/z : %f",origin_gps_z_);
		}
		else {
			ROS_INFO("Failed to get param /gps/origin/z");
			origin_gps_z_ = 0.0;
		}

		if (node_->getParam("gps_ekf/relative_gps", relative_gps_)) {
			ROS_INFO("gps_ekf/relative_gps : %d",relative_gps_);
		}
		else {
			ROS_INFO("Failed to get param /gps_ekf/relative_gps");
		}
		// if(!relative_gps_){
		// 	origin_gps_x_ = 0.0;
		// 	origin_gps_y_ = 0.0;
		// }


		int number_of_roads;
		int number_of_lanes;
        int number_of_wk_roads = 36;
        int number_of_wk_lanes = 36;


		// Loading Road Configurations
		pnode_->getParam("number_of_roads",number_of_roads);
		printf("number of roads %d \n" , number_of_roads);
        printf("number of wk_roads %d \n" , number_of_wk_roads);

        Road_Networks_.roads.resize(number_of_roads);
        Workzone_Networks.roads.resize(number_of_wk_roads);


		//for roads

            for(int i=0;i<number_of_roads;i++){
                int road_number = 0;
                int road_type = 0;
                int action = 0;
                double speed_limit = 0.0;
                double distance = 0.0;
                std::stringstream road_name;
                road_name << "road_" << i+1;

                pnode_->getParam(road_name.str()+"/road_number",road_number);  // 속성파일에 있는게 파라미터 서버로 올라가고 그걸 파라미터로 가져오는거
                pnode_->getParam(road_name.str()+"/road_type",road_type);
                pnode_->getParam(road_name.str()+"/action",action);
                pnode_->getParam(road_name.str()+"/distance",distance);
                pnode_->getParam(road_name.str()+"/speed_limit",speed_limit);

                // std::cout << "***********" <<road_number << std::endl;



                Road road_info;
                road_info.road_number = road_number;
                road_info.road_type = road_type;
                road_info.action = action;
                road_info.distance = distance;
                road_info.speed_limit = speed_limit;


                if(road_number > 1 && road_number < 37)
                {
                    Workzone_Networks.roads.at(road_number-1) = road_info;
                }

                Road_Networks_.roads.at(road_number-1) = road_info;
            }

            // Loading Lane Configurations and Waypoints
            pnode_->getParam("number_of_lanes",number_of_lanes);


            // for lanes
            for(int i=0;i<number_of_lanes;i++){
                int road_number = 0;
                int lane_number = 0;
                int begin_node = 0;
                int end_node = 0;
                std::string wp_filename = std::string("");

                std::stringstream lane_name;
                lane_name << "lane_" << i+1;

                //loading lane_config.yaml

                pnode_->getParam(lane_name.str()+"/road_number",road_number);
                pnode_->getParam(lane_name.str()+"/lane_number",lane_number);
                pnode_->getParam(lane_name.str()+"/begin_node",begin_node);
                pnode_->getParam(lane_name.str()+"/end_node",end_node);
                pnode_->getParam(lane_name.str()+"/waypoint_file",wp_filename);


                // Waypoint data loading
                std::string* wp_filename_ptr;
                wp_filename_ptr = &wp_filename;
                *wp_filename_ptr = wp_file_directory + wp_filename;

								// std::cout << wp_filename;

                std::vector<Waypoint> waypoints;

                int wp_load = LoadWaypoints(wp_filename_ptr,&waypoints);

//			printf("number of waypoints: %d\n",waypoints.size());
////            std::cout << waypoints.at(0).x << std::endl;
//            printf("curvature :: %lf \n",waypoints.at(0).curvature);
//            printf("xxx :: %lf \n",waypoints.at(0).x);
//            printf("yyy :: %lf \n",waypoints.at(0).y);
//            printf("zzz :: %lf \n",waypoints.at(0).z);

                Lane lane_info;
                lane_info.road_number = road_number;
                lane_info.lane_number = lane_number;
                lane_info.begin_node = begin_node;
                lane_info.end_node = end_node;
                lane_info.waypoints = waypoints;

				// std::cout << "----------------------------------" << std::endl;
				// std::cout << *wp_filename_ptr << std::endl;
				// std::cout << "----------------------------------" << std::endl;

                // lane_info.waypoints = waypoints;
                if(road_number == 0) continue;

                if(Road_Networks_.roads.at(road_number-1).lanes.empty()){
                    Road_Networks_.roads.at(road_number-1).lanes.push_back(lane_info);
                }
                else{
                    if(lane_number > Road_Networks_.roads.at(road_number-1).lanes.size()){
                        Road_Networks_.roads.at(road_number-1).lanes.push_back(lane_info);
                    }
                    else{
                        Road_Networks_.roads.at(road_number-1).lanes.insert(Road_Networks_.roads.at(road_number-1).lanes.begin()+lane_number-1,lane_info);
                    }
                }

                if(road_number > 1 && road_number < 37)
                {
                    Lane Wk_lane_info;
                    Wk_lane_info.road_number = road_number;
                    Wk_lane_info.lane_number = lane_number;
                    Wk_lane_info.begin_node = begin_node;
                    Wk_lane_info.end_node = end_node;
                    Wk_lane_info.waypoints = waypoints;

                    Workzone_Networks.roads.at(road_number-1).lanes.push_back(Wk_lane_info);
                }

            }



        workzone_graph_build = true;

		std::vector<Road>::iterator iter;
		std::vector<Lane>::iterator iter2;
		std::vector<Waypoint>::iterator iter3;




//        for(int i = 0; i<Road_Networks_.roads.size(); i++)
//        {
//        ROS_ERROR("road_networks road %d \n" , Road_Networks_.roads.at(i).road_number);
//        }

		//도로 길이 계산
		for(iter=Road_Networks_.roads.begin();iter!=Road_Networks_.roads.end();iter++){
			double road_distance = 0.0;
			for(iter2=(*iter).lanes.begin();iter2!=(*iter).lanes.end();iter2++){

			    double lane_slope = 0.0;
                int i=1;

				for(iter3=(*iter2).waypoints.begin();iter3!=(*iter2).waypoints.end()-1;iter3++){

					double x1 = (*iter3).x;
					double y1 = (*iter3).y;
					double x2 = (*(iter3+1)).x;
					double y2 = (*(iter3+1)).y;

					double dist = sqrt(pow(x2-x1,2) + pow(y2-y1,2));
					road_distance += dist;
//					if(iter->road_number == 3 && iter2->lane_number == 1)
//                    {
                        if(fabs((iter3)->curvature) > 0.4)
                        {
//                            ROS_ERROR("road num :: %d \n" , (*iter).road_number);
//                            ROS_ERROR("lane num :: %d \n" , (*iter2).lane_number);
//                            ROS_ERROR("idx num :: %d \n" , i);
//                            ROS_ERROR("curvature :: %lf \n" , (iter3)->curvature);
                        }

//                    }
                    i++;

				}
                //ROS_ERROR(" road graph lane node start :: %d \n" , (*iter2).begin_node);
			}
			road_distance = road_distance/(*iter).lanes.size();

			// if(iter->road_type == 1){
			// 	for(iter2=(*iter).lanes.begin();iter2!=(*iter).lanes.end();iter2++){
			// 		for(iter3=(*iter2).waypoints.begin();iter3!=(*iter2).waypoints.end()-1;iter3++){
			// 			double x1 = (*iter3).x;
			// 			double y1 = (*iter3).y;
			// 			double x2 = (*(iter3+1)).x;
			// 			double y2 = (*(iter3+1)).y;

			// 			double dist = sqrt(pow(x2-x1,2) + pow(y2-y1,2));
			// 			road_distance += dist;
			// 		}
			// 	}
			// 	road_distance = road_distance/(*iter).lanes.size();
			// }
			// else{
			// 	road_distance = 5.0;
			// }



			if(iter->lanes.size() == 1){
				road_distance *= 1.2;
			}

			// if(iter->road_type == 4){
			// 	road_distance = -11.0;
			// }

			(*iter).distance = road_distance;

			printf("road: %d, distance: %.3f\n",iter->road_number,road_distance);
		}



		return 0;
	}

	void MissionStateCallback(const std_msgs::Int32ConstPtr &msg)
    {
        mission_state_.data = msg->data;
    }

	void ClickedPointCallback(const geometry_msgs::PointStampedConstPtr &msg)
    {
        clicked_point = *msg;
        click_bool.data = true;
        if(clicked_list.workzone_xy.size() > 5)
        {
            clicked_list.workzone_xy.clear();
        }
        clicked_xy.workzone_x = clicked_point.point.x+origin_gps_x_;
        clicked_xy.workzone_y = clicked_point.point.y+origin_gps_y_;
        clicked_list.workzone_xy.push_back(clicked_xy);
        clicked = true;

        wk_road_number.clear();
        std::vector<Road>::iterator wk_iter;
        std::vector<Lane>::iterator wk_iter2;
        std::vector<Waypoint>::iterator wk_iter3;
        if(clicked)
        {
            workzone_list = clicked_list;
            clicked = false;
        }
        for(int i = 0; i< workzone_list.workzone_xy.size(); i++)
        {
            workzone_list.workzone_xy.at(i).workzone_x -= origin_gps_x_;
            workzone_list.workzone_xy.at(i).workzone_y -= origin_gps_y_;
            unsigned int wk_wp_idx = 0;
            unsigned int wk_road_number_ = 0;
            unsigned int wk_lane_number_ = 0;

            double min_dist = DBL_MAX;
            tf::Point p_work, p_wp;

            p_work.setValue( workzone_list.workzone_xy.at(i).workzone_x,workzone_list.workzone_xy.at(i).workzone_y, 0.0);

            for(wk_iter = Workzone_Networks.roads.begin(); wk_iter != Workzone_Networks.roads.end(); wk_iter++)
            {
                for(wk_iter2 = (*wk_iter).lanes.begin(); wk_iter2 != (*wk_iter).lanes.end(); wk_iter2++)
                {
                    int i = 0;
                    for(wk_iter3 = (*wk_iter2).waypoints.begin(); wk_iter3 != (*wk_iter2).waypoints.end(); wk_iter3++) {
                        p_wp.setValue((*wk_iter3).x, (*wk_iter3).y, 0.0);
                        double dist = p_work.distance(p_wp);
                        if (dist < min_dist)
                        {
                            wk_road_number_ = (*wk_iter2).road_number;
                            wk_lane_number_ = (*wk_iter2).lane_number;
                            min_dist = dist;
                            wk_wp_idx = i;
                        }
                        i++;
                    }
                }
            }

            //ROS_ERROR("wk_road number :: %d \n" , wk_road_number_);
            wk_road_number.push_back(wk_road_number_);

        }
        click_bool_.publish(click_bool);

    }

    void workzone_callback(const pharos_msgs::Workzone_listConstPtr &msg) {
//	    return;   //// this should be deleted
        workzone_list = *msg;
        workzone_bool_.data = true;

        if(workzone_graph_build = true)
        {
            //ROS_ERROR("workzone size :: %lu \n" , workzone_list.workzone_xy.size() );
            wk_road_number.clear();
            std::vector<Road>::iterator wk_iter;
            std::vector<Lane>::iterator wk_iter2;
            std::vector<Waypoint>::iterator wk_iter3;
            if(clicked)
            {
                workzone_list = clicked_list;
                clicked = false;
            }
            for(int i = 0; i< workzone_list.workzone_xy.size(); i++)
            {
                workzone_list.workzone_xy.at(i).workzone_x -= origin_gps_x_;
                workzone_list.workzone_xy.at(i).workzone_y -= origin_gps_y_;
                unsigned int wk_wp_idx = 0;
                unsigned int wk_road_number_ = 0;
                unsigned int wk_lane_number_ = 0;

                double min_dist = DBL_MAX;
                tf::Point p_work, p_wp;

                p_work.setValue( workzone_list.workzone_xy.at(i).workzone_x,workzone_list.workzone_xy.at(i).workzone_y, 0.0);

                for(wk_iter = Workzone_Networks.roads.begin(); wk_iter != Workzone_Networks.roads.end(); wk_iter++)
                {
                    for(wk_iter2 = (*wk_iter).lanes.begin(); wk_iter2 != (*wk_iter).lanes.end(); wk_iter2++)
                    {
                        int i = 0;
                        for(wk_iter3 = (*wk_iter2).waypoints.begin(); wk_iter3 != (*wk_iter2).waypoints.end(); wk_iter3++) {
                            p_wp.setValue((*wk_iter3).x, (*wk_iter3).y, 0.0);
                            double dist = p_work.distance(p_wp);
                            if (dist < min_dist)
                            {
                                wk_road_number_ = (*wk_iter2).road_number;
                                wk_lane_number_ = (*wk_iter2).lane_number;
                                min_dist = dist;
                                wk_wp_idx = i;
                            }
                        i++;
                    }
                }
            }

//            ROS_ERROR("wk_road number :: %d \n" , wk_road_number_);
            wk_road_number.push_back(wk_road_number_);

            }
            workzone_bool.publish(workzone_bool_);

        }

        visualization_msgs::MarkerArrayPtr wk_array (new visualization_msgs::MarkerArray);
        visualization_msgs::MarkerPtr wk_points (new visualization_msgs::Marker);


        wk_points->header.frame_id = "/odom";
        wk_points->header.stamp = ros::Time::now();
        wk_points->ns = "wk_points";
        wk_points->id = 0;
        wk_points->type = visualization_msgs::Marker::POINTS;
        wk_points->action = visualization_msgs::Marker::ADD;
        wk_points->scale.x = 2; //Line width
        wk_points->scale.y = 1;
        wk_points->scale.z = 0.0;
        wk_points->color.a = 1;
        wk_points->color.r = 1;
        wk_points->color.g = 1;
        wk_points->color.b = 0;


    }

	int LoadWaypoints(std::string* wp_file_name, std::vector<Waypoint>* Waypoints){

		// std::cout<< *wp_file_name << "123123123 " <<"\n";

		// File Loading
		std::fstream fs;
		fs.open(wp_file_name->c_str(), std::fstream::in);
		if (!fs.is_open())
		{
			ROS_FATAL("Cannot open Road data file: %s", wp_file_name->c_str());
			return -1;
		}

		// Reading and Parsing Data from Files
		std::vector<Waypoint> waypoints;
		int iter = 0;
		while(1)
		{
			iter++;
			double x, y, z, curvature, velocity, sect;
			std::string str;

			std::istringstream ss;

			std::getline(fs, str);
			ss.str(str);

			Waypoint wp;

			ss >> wp.x >> wp.y >> wp.z >> wp.curvature >> velocity >> sect;

			if(fs.eof())break;

			wp.x -= origin_gps_x_;
			wp.y -= origin_gps_y_;
			wp.z -= origin_gps_z_;

			waypoints.push_back(wp);
		}

		*Waypoints = waypoints;

		return 1;
	}

	// Publish data
	void publish()
	{
		ros::Rate loop_rate(publish_rate_);

		while (node_->ok())
		{
		    if(wk_road_number.empty())
            {
//                ROS_ERROR(" workzone is empty !! " );
            }else{
		        for(int i = 0; i<wk_road_number.size(); i++)
                {
                    Road_Networks_.roads[wk_road_number.at(i)-1].distance = 0;
//                    ROS_ERROR("number :: %d \n\n\n" , Road_Networks_.roads[wk_road_number.at(i)].road_number-1);

                }

            }

		    // if(mission_state_.data == 5 && mission_state_.data == 6)
        //     {
		    //     Road_Networks_.roads[51].speed_limit = 30;
		    //     Road_Networks_.roads[52].speed_limit = 30;
		    //     Road_Networks_.roads[53].speed_limit = 30;
        //     }
		    road_net_graph_pub_.publish(Road_Networks_);
				// std::cout << Road_Networks_ << std::endl;
//
//            ROS_ERROR("Mission state :: %d  " , mission_state_.data);
//            ROS_ERROR("Road_Network[53] :: %d  " ,Road_Networks_.roads[51].road_number );
//            ROS_ERROR("Road_Network[53]speed_limit :: %lf  " ,Road_Networks_.roads[51].speed_limit );
//            ROS_ERROR("Road_Network[53] :: %d  " ,Road_Networks_.roads[52].road_number );
//            ROS_ERROR("Road_Network[53]speed_limit :: %lf  " ,Road_Networks_.roads[52].speed_limit );
//            ROS_ERROR("Road_Network[53] :: %d  " ,Road_Networks_.roads[53].road_number );
//            ROS_ERROR("Road_Network[53]speed_limit :: %lf  " ,Road_Networks_.roads[53].speed_limit );


            //// debuging workzone cost
//		    for(int i = 0; i<35; i++)
//            {
//                std::cout << i+1 << " :: " << Road_Networks_.roads[i].distance << std::endl;
//            }


            ros::spinOnce();
			loop_rate.sleep();
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Road_Graph_Building");
//    ros::spin();

	RoadGraphBuild rgb_node;
	if (rgb_node.init())
	{
		ROS_FATAL("Road_Graph_Building initialization failed");
		return -1;
	}


	rgb_node.publish();

    return 0;
}

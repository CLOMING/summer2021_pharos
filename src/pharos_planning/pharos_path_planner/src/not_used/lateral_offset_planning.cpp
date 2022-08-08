#include <lateral_offset_planning.h>
#define map_resolution 0.15

CLatOffsetPlanning lat_plannig;

std::vector<tf::Point> CLatOffsetPlanning::trajectory_simulation(std::vector<tf::Point> *wp, std::vector<tf::Point> *curvature, int index,int current, double gps_x, double gps_y, double heading, double shifting, double lane_center, double velocity, double& moving_dist, int& collision_flag, double& max_curve){

	//ROS_INFO("trajectory simulation start");
	std::vector<tf::Point> trajectory;
	trajectory.clear();
	trajectory.resize(planning_sample);


	//	if(index == 3){
	//		for(int i=0;i<obstacle_map_.data.size();i++){
	//			if(obstacle_map_.data.at(i) != 0){
	//				ROS_INFO("map_data[%d]: %d",i, obstacle_map_.data.at(i));
	//			}
	//			//ROS_INFO("map_data[%d]: %d",i, map->data.at(i));
	//		}
	//	}

	int map_data_size = obstacle_map_.data.size();
	map_resolution_ = obstacle_map_.info.resolution;
//	int map_width = obstacle_map_.info.width;
//	int map_height = obstacle_map_.info.height;

	double angle_diff;
	double lat_off;
	double old_angle_diff;
	double old_lat_off;
	double angle_diff_d;
	double lat_off_d;
	double curv = 0.0;
	double theta;
	double theta_dot;

	double h_pose_x; //after heading compensation
	double h_pose_y;
	double h_pose_x_old;
	double h_pose_y_old;
	double pose_x;
	double pose_y;
	double pose_x_dot;
	double pose_y_dot;
	double pose_x_old;
	double pose_y_old;
	double steering;

	double planning_dist;

	double slope;

	double R;
	double beta;
	double Cx;
	double Cy;
	double delta_dist;

	double offset = 0.0;
	//double lateral_acc = abs(shifting)/lat_off_rate;
	double lateral_acc = 0.1;

	theta = heading;

	double velocity_kph = velocity * 3.6;

	//--- planning distance setting ---//

	float planning_dist_scale = 0.7;
	planning_dist = planning_dist_scale*velocity_kph;

	if(planning_dist > 40){
		planning_dist = 40.0;
	}
	else if(planning_dist < 10){
		planning_dist = 10.0;
	}


//	if (velocity_kph > 40.0){
//		planning_dist = 40.0;
//	}
//	else if (velocity_kph > 10.0){
//		planning_dist = (40.0 - 20.0)/(40.0-10.0) * (velocity_kph*3600/1000) + 40/3;
//	}
//	else{
//		planning_dist = 2 * (velocity_kph*3600/1000);
//		//planning_dist = 20;
//	}
//	if(planning_dist < 5){
//		planning_dist = 5.0;
//	}
//	planning_dist += 4.0;

	delta_dist = planning_dist/planning_sample;



	// minimum speed to generate path
	// if (velocity < 1.39){   // 5 km/h
	// 	velocity = 1.39;
	// }
	if (velocity < 1.39){   // 5 km/h
		velocity = 1.39;
	//velocity = 5.0;
	}

	int order = 0;

	int collision_bound_x;
	int collision_bound_y;
	//while(moving_dist < look_ahead){


	collision_flag = 0;
	moving_dist = 0;

	double x_d;
	double y_d;
	double x_dd;
	double y_dd;

	double look_x;
	double look_y;

	int lookahead_current = 0;
	int old_lookahead = 0;

	double ref_angle;
	static int old_idx = 0;
	lookahead_current = lat_plannig.NearestWaypointIndex(wp, gps_x,gps_y,theta,old_idx);
	old_idx = lookahead_current;
	//lookahead_current = lat_plannig.closestwaypoint2(wp, gps_x,gps_y);
	look_x = gps_x + 2.7*cos(theta);
	look_y = gps_y + 2.7*sin(theta);
	lat_off = lateraloffset(look_x, look_y, wp->at(lookahead_current).x(), wp->at(lookahead_current).y(), wp->at(lookahead_current+1).x(), wp->at(lookahead_current+1).y());
	double init_lat_offset = lat_off - lane_center;

	pose_x = gps_x;
	pose_y = gps_y;
	pose_x_old = pose_x;
	pose_y_old = pose_y;

	int grid_pose_x1=0;
	int grid_pose_y1=0;
	int grid_pose_x2=0;
	int grid_pose_y2=0;
	int old_idx2 = 0;
	for (int i=0;i<planning_sample;i++){
		//collision_flag =1;
		if (collision_flag != 1 && moving_dist < planning_dist){


			//// Predict with Vehicle Kinematics ////

			look_x = pose_x + 2.7*cos(theta);
			look_y = pose_y + 2.7*sin(theta);

			//lookahead_current =closestwaypoint2(old_lookahead,look_x,look_y,theta);
			//current =closestwaypoint2(old_current,look_x,look_y,theta);

			lookahead_current = lat_plannig.NearestWaypointIndex(wp, pose_x,pose_y,theta,old_idx2);
			old_idx2 = lookahead_current;
			//lookahead_current = lat_plannig.closestwaypoint2(wp, pose_x,pose_y);

			//current = ClosetWaypoint_order(old_current,pose_x,pose_y,theta); // closest way point

			angle_diff = angle_difference(lookahead_current, theta, wp,ref_angle);
			//			if (angle_diff> M_PI){
			//				angle_diff = angle_diff - 2*M_PI;
			//			}
			//			else if (angle_diff< -M_PI){
			//				angle_diff = angle_diff + 2*M_PI;
			//			}



			if(lookahead_current >= (wp->size()-2) ){
				lat_off = lateraloffset(look_x, look_y, wp->at(lookahead_current).x(), wp->at(lookahead_current).y(), wp->at(0).x(), wp->at(0).y());
				//ROS_INFO("test");
			}
			else{
				lat_off = lateraloffset(look_x, look_y, wp->at(lookahead_current).x(), wp->at(lookahead_current).y(), wp->at(lookahead_current+1).x(), wp->at(lookahead_current+1).y());
			}

			if (shifting > 0){
				offset += lateral_acc;
				if (offset > shifting){
					offset = shifting;
				}
			}

			if (shifting < 0){
				offset -= lateral_acc;
				if (offset < shifting){
					offset = shifting;
				}
			}

			lat_off += shifting;
			//lat_off += offset;
			if(curvature->size() != 0){
				curv = curvature->at(lookahead_current).z();
				//curv = 0.0;
			}
			else{
				curv = 0.0;
			}

			steering = steering_control(angle_diff, 0.0, lat_off, 0.0, curv, velocity);

			beta = delta_dist/wheel_base*tan(steering);

			//ROS_INFO("angle_diff: %f, lat_offset: %f",angle_diff, lat_off);
			//ROS_INFO("steering: %f, beta: %f, pose_x_dot: %f, pose_y_dot: %f", steering, beta, pose_x_dot, pose_y_dot);

			/*
			if (abs(beta)<0.001){
				pose_x_dot = delta_dist * cos(theta);
				pose_y_dot = delta_dist * sin(theta);

				theta += beta;
				pose_x += pose_x_dot;
				pose_y += pose_y_dot;
				//ROS_INFO("A");
			}
			else{

				R = wheel_base/beta;
				Cx = pose_x - delta_dist * sin(theta);
				Cy = pose_y + delta_dist * cos(theta);

				theta += beta;
				pose_x = Cx + sin(theta)*R;
				pose_y = Cy - cos(theta)*R;
				//ROS_INFO("B");
			}*/

			pose_x_dot = delta_dist * cos(theta);
			pose_y_dot = delta_dist * sin(theta);

			theta += beta;
			pose_x += pose_x_dot;
			pose_y += pose_y_dot;

			moving_dist += delta_dist;



			//trajectory_[index][order][1] = pose_x;
			//trajectory_[index][order][2] = pose_y;



			//// collision check ////


			if(obstacle_map_.data.size() > 0){

				int map_width = obstacle_map_.info.width;
				int map_height = obstacle_map_.info.height;


				int collision_range = 7;


				if (velocity*3600/1000 > 40.0){
					collision_range = 11;
				}
				else if (velocity*3600/1000 > 30.0){
					collision_range = 9;
				}
				else if (velocity*3600/1000 > 20.0){
					collision_range = 8;
				}
				else if (velocity*3600/1000 > 10.0){
					collision_range = 7;
				}


				//Rotate
//				h_pose_x = (pose_x - gps_x) * cos(-heading) - (pose_y - gps_y) * sin(-heading);
//				h_pose_y = (pose_x - gps_x) * sin(-heading) + (pose_y - gps_y) * cos(-heading);
//
//				h_pose_x_old = (pose_x_old - gps_x) * cos(-heading) - (pose_y_old - gps_y) * sin(-heading);
//				h_pose_y_old = (pose_x_old - gps_x) * sin(-heading) + (pose_y_old - gps_y) * cos(-heading);

				h_pose_x = (pose_x - gps_x) * cos(-0) - (pose_y - gps_y) * sin(-0);
				h_pose_y = (pose_x - gps_x) * sin(-0) + (pose_y - gps_y) * cos(-0);

				h_pose_x_old = (pose_x_old - gps_x) * cos(-0) - (pose_y_old - gps_y) * sin(-0);
				h_pose_y_old = (pose_x_old - gps_x) * sin(-0) + (pose_y_old - gps_y) * cos(-0);

				PointToGrid(pose_x_old - gps_x,pose_y_old - gps_y, grid_pose_x1,grid_pose_y1);
				//PointToGrid(pose_x - gps_x,pose_y - gps_y, grid_pose_x2,grid_pose_y2);

				PointToGrid(h_pose_x,h_pose_y, grid_pose_x2,grid_pose_y2);

				grid_pose_x1 += map_width/2-1;
				grid_pose_x2 += map_width/2-1;
				grid_pose_y1 += map_height/2-1;
				grid_pose_y2 += map_height/2-1;

				//printf("grid_x: %d, grid_y: %d\n", grid_pose_x2, grid_pose_y2);


				int map_idx = GridToIdx(map_height,grid_pose_x2,grid_pose_y2);
				//printf("map_idx: %d\n",map_idx);

//				if(obstacle_map_.data.at(map_idx) != 0){
//					collision_flag = 1;
//					//printf("collision!!!\n");
//				}



				if (pose_x_old != 0){
					if(i == 0){
						slope = 0;
					}
					else{
						slope = atan2(-(h_pose_x-h_pose_x_old),h_pose_y-h_pose_y_old);
					}

					for (int i=-collision_range;i<collision_range+1;i++){
						collision_bound_x = int(grid_pose_x2 + i*cos(slope));
						collision_bound_y = int(grid_pose_y2 + i*sin(slope));
						if ( collision_bound_x > map_width-1 || collision_bound_y > map_height-1 ){
							continue;
						}
						else if ( collision_bound_x < 0 || collision_bound_y < 0 ){
							continue;
						}
						//global_map.at<unsigned char>(collision_bound_x,collision_bound_y) = 200;
						int map_idx = GridToIdx(map_height,collision_bound_x,collision_bound_y);

						if(obstacle_map_.data.at(map_idx) != 0){
							collision_flag = 1;
							//printf("collision!!!\n");
						}
					}
				}
			}
			trajectory[i].setValue(pose_x,pose_y,0.0);
		}
		else{
			if (i > 0){
				//trajectory_[index][i][1] = trajectory_[index][i-1][1];
				//trajectory_[index][i][2] = trajectory_[index][i-1][2];
				trajectory[i].setValue(trajectory[i-1].x(),trajectory[i-1].y(),0.0);
			}

		}

		pose_x_old = pose_x;
		pose_y_old = pose_y;
		//order += 1;

		old_angle_diff = angle_diff;
		old_lat_off = lat_off;

	}
	// curvature //
	max_curve = 0;



	return trajectory;
}


double CLatOffsetPlanning::steering_control(double angle_diff, double angle_diff_d, double lat_off, double lat_off_d, double curvature, double velocity){

	double steering_angle;
	double kp_angle;
	double kp_lat;
	double kp_curv;
	double kd_angle;
	double kd_lat;

	double angle_control;
	double lat_control;
	double curv_control;

	kp_angle = 0.8;
	kd_angle = 0.2;
	kp_lat = 3.0;//1;
	kd_lat = 0.1;
	kp_curv = 2.4;

	angle_control = kp_angle * angle_diff + kd_angle * angle_diff_d;
	lat_control = atan2(kp_lat * lat_off + kd_lat * lat_off_d, velocity);
	curv_control = kp_curv*curvature;

	steering_angle = angle_control + lat_control + curv_control;

	double max_steering = 35.0;
	if(steering_angle > (max_steering*M_PI/180)){
		steering_angle = (max_steering*M_PI/180);
	}
	else if(steering_angle < (-max_steering*M_PI/180)){
		steering_angle = (-max_steering*M_PI/180);
	}

	return steering_angle;
}


double CLatOffsetPlanning::angle_difference(int current, double heading, std::vector<tf::Point> *waypoints, double& ref_angle){

	double r_path_angle;
	double angle_diff;

	if (current >= (waypoints->size()-2)){
		r_path_angle = atan2(waypoints->at(0).y() - waypoints->at(current).y(), waypoints->at(0).x() - waypoints->at(current).x());
	}
	else{
		r_path_angle = atan2(waypoints->at(current+1).y() - waypoints->at(current).y(), waypoints->at(current+1).x() - waypoints->at(current).x());
	}
	ref_angle = r_path_angle;

	angle_diff = r_path_angle - heading;

	if((angle_diff)>2*M_PI){
		while(angle_diff > 2*M_PI){
			angle_diff = angle_diff - 2*M_PI;
		}
	}
	else if((angle_diff)< -2*M_PI){
		while(angle_diff < -2*M_PI){
			angle_diff = angle_diff + 2*M_PI;
		}
	}


	if (angle_diff> M_PI){
		angle_diff = angle_diff - 2*M_PI;
	}
	else if (angle_diff< -M_PI){
		angle_diff = angle_diff + 2*M_PI;
	}

	return angle_diff;
}


inline double CLatOffsetPlanning::lateraloffset(double cur_x,double cur_y,double x1,double y1,double x2,double y2){

	double lat_off;

	lat_off = ((cur_x-x1)*(y2-y1)-(cur_y-y1)*(x2-x1))/(sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)));


	return lat_off;
}

double CLatOffsetPlanning::calc_curvature(double xd, double yd, double xdd, double ydd){

	double curv;
	double den;

	den = pow(xd*xd + yd*yd,1.5);
	if (den != 0){
		curv = (xd*ydd - yd*xdd)/den;
		//      if (abs(curv) > 1){
		//        curv = 0;
		//      }
	}
	return curv;
}


unsigned int CLatOffsetPlanning::NearestWaypointIndex(std::vector<tf::Point> *path, double gpsx, double gpsy, double heading, unsigned int old_index){

	tf::Point gps;

	double dist = 0.0;
	double min_dist = DBL_MAX;
	double min_dist1 = DBL_MAX;
	double min_dist2 = DBL_MAX;
	double path_heading = 0.0;
	double heading_err = 0.0;
	static int index = 0;
	static int index1 = 0;
	static int index2 = 0;

	gps.setX(gpsx);
	gps.setY(gpsy);

	int search_start_index = 0;
	int search_finish_index = 0;

	if(old_index == 0){
		search_start_index = 0;
		search_finish_index = (unsigned int)path->size() - 2;
		//printf("search_start:%d, search_finish: %d\n",search_start_index,search_finish_index );
	}
	else{
		search_start_index = old_index-10;
		if(search_start_index < 0){
			search_start_index = 0;
		}
		search_finish_index = old_index+20;// distance between 2 wp is around 20cm.. 100 indexes -> around 20m
	}
	// printf("search_start:%d, search_finish: %d\n",search_start_index,search_finish_index );

	if(search_finish_index > (unsigned int)(path->size())-2){
		search_finish_index = (unsigned int)(path->size())-2;

		for (unsigned int  j = search_start_index; j < (path->size()-1); j++){
			dist = gps.distance(path->at(j));
			if(j==(path->size())-3){
				path_heading = atan2(path->at(j).y() - path->at(j-1).y(), path->at(j).x() - path->at(j-1).x());
			}
			else{
				path_heading = atan2(path->at(j+1).y() - path->at(j).y(), path->at(j+1).x() - path->at(j).x());
			}
			heading_err = path_heading-heading;
			if (heading_err > M_PI) {
				heading_err = heading_err - 2 * M_PI;
			}
			else if (heading_err < -M_PI) {
				heading_err = heading_err + 2 * M_PI;
			}
			if (dist < min_dist && fabs(heading_err) < M_PI/3) {
				min_dist1 = dist;
				index1 = j;
			}

		}
		for (unsigned int j = 0; j < 20; j++){
			dist = gps.distance(path->at(j));
			if(j==(path->size())-3){
				path_heading = atan2(path->at(j).y() - path->at(j-1).y(), path->at(j).x() - path->at(j-1).x());
			}
			else{
				path_heading = atan2(path->at(j+1).y() - path->at(j).y(), path->at(j+1).x() - path->at(j).x());
			}
			heading_err = path_heading-heading;
			if (heading_err > M_PI) {
				heading_err = heading_err - 2 * M_PI;
			}
			else if (heading_err < -M_PI) {
				heading_err = heading_err + 2 * M_PI;
			}
			if (dist < min_dist && fabs(heading_err) < M_PI/3) {
				min_dist2 = dist;
				index2 = j;
			}
		}
		if (min_dist1<min_dist2){
			index = index1;
		}
		else{
			index = index2;
		}

	}
	else
	{
		for (unsigned int j = search_start_index; j < search_finish_index; j++) {
			dist = gps.distance(path->at(j));
			if(j==(path->size())-3){
				path_heading = atan2(path->at(j).y() - path->at(j-1).y(), path->at(j).x() - path->at(j-1).x());
			}
			else{
				path_heading = atan2(path->at(j+1).y() - path->at(j).y(), path->at(j+1).x() - path->at(j).x());
			}
			heading_err = path_heading-heading;
			if (heading_err > M_PI) {
				heading_err = heading_err - 2 * M_PI;
			}
			else if (heading_err < -M_PI) {
				heading_err = heading_err + 2 * M_PI;
			}
			if (dist < min_dist && fabs(heading_err) < M_PI/3) {
				min_dist = dist;
				index = j;
			}
		}
	}

	if (index > path->size()){
		index = (unsigned int)path->size()-2;
	}
	//printf("search[%d ~ %d], index = %d,size:%d\n",search_start_index,search_finish_index,index,int(Wp_.size()));
	//printf("%.4f, %.4f\n",Wp_[Wp_.size()-3].x(),Wp_[Wp_.size()-3].y());
	//printf("%.4f, %.4f\n",Wp_[Wp_.size()-1].x(),Wp_[Wp_.size()-1].y());
	return index;
}

int CLatOffsetPlanning::closestwaypoint2 (std::vector<tf::Point> *path, double gpsx, double gpsy){
	int order;
	double dist;
	double min_dist = DBL_MAX;
	int over_order;

	int length = path->size();

	for (int i=0; i<path->size();i++)
	{
		if (i > length-1){
			over_order = i-length;
			dist = sqrt(pow(gpsx-path->at(i).x(),2) + pow(gpsy-path->at(i).y(),2));
		}
		else{
			dist = sqrt(pow(gpsx-path->at(i).x(),2) + pow(gpsy-path->at(i).y(),2));
		}
		if (min_dist > dist)
		{
			min_dist = dist;
			order = i;
			if(order > length-1)
			{
				order = i-length;
			}
		}

	}
	return order;
}


inline void CLatOffsetPlanning::PointToGrid(const double point_x, const double point_y, int& grid_x, int& grid_y){
	grid_x = (int)(point_x/map_resolution);
	grid_y = (int)(point_y/map_resolution);
}
inline int CLatOffsetPlanning::GridToIdx(const int map_height, const int grid_x, const int grid_y){
	int map_idx = map_height*grid_y + grid_x;
	return map_idx;
}





#ifndef _LAT_OFFSET_PLANING_H_
#define _LAT_OFFSET_PLANING_H_

#include <path_planner.h>


#define wheel_base 2.7

//nav_msgs::OccupancyGrid obs_Map_;



class CLatOffsetPlanning{

public:

	double map_resolution_;

	std::vector<tf::Point> trajectory_simulation(std::vector<tf::Point> *wp, std::vector<tf::Point> *curvature, int index,int current, double pose_x, double pose_y, double heading, double shifting, double lane_center, double velocity, double& moving_dist, int& collision_flag, double& max_curve);
	double steering_control(double angle_diff, double angle_diff_d, double lat_off, double lat_off_d, double curvature, double velocity);


	double angle_difference(int current, double heading, std::vector<tf::Point> *waypoints,double& ref_angle);
	double lateraloffset(double cur_x,double cur_y,double x1,double y1,double x2,double y2);
	double calc_curvature(double xd, double yd, double xdd, double ydd);
	unsigned int NearestWaypointIndex(std::vector<tf::Point> *path, double gpsx, double gpsy, double heading, unsigned int old_index);
	int closestwaypoint2 (std::vector<tf::Point> *path, double gpsx, double gpsy);

	void PointToGrid(const double point_x, const double point_y, int& grid_x, int& grid_y);
	int GridToIdx(const int map_height, const int grid_x, const int grid_y);

};

#endif




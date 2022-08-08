#ifndef _PARKING_H_
#define _PARKING_H_

#include <path_planner.h>


class CParking{
public:


std::vector<tf::Point> parking_path_generating(const double parking_x, const double parking_y, const double parking_heading, nav_msgs::Path& parking_path);
void PositionToPoseStamped(const double x, const double y, const double heading, geometry_msgs::PoseStamped& pose_stamp);

};


#endif

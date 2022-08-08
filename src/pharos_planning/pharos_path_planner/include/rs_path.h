
#ifndef _RS_PATH_H_
#define _RS_PATH_H_

#include <path_planner.h>

#define d 0.5
#define turning_r  7.0


class CRS_PATH{
public:
	double Mod(double input, double divider);


	double LSrR(const double xs, const double ys, const double ts, const double xg, const double yg, const double tg, std::vector<tf::Point>& rs_path);
	double RSrR(const double xs, const double ys, const double ts, const double xg, const double yg, const double tg, std::vector<tf::Point>& rs_path);
	double LrSrR(const double xs, const double ys, const double ts, const double xg, const double yg, const double tg, std::vector<tf::Point>& rs_path);
	double RrSrR(const double xs, const double ys, const double ts, const double xg, const double yg, const double tg, std::vector<tf::Point>& rs_path);

};

#endif




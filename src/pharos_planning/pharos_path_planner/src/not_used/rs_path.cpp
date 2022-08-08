
#include <path_planner.h>


#include <rs_path.h>
#include <math.h>


CRS_PATH RS_PATHs;


double CRS_PATH::LSrR(const double xs, const double ys, const double ts, const double xg, const double yg, const double tg, std::vector<tf::Point>& rs_path){
	double path_cost;

	double x1 = xs - turning_r*sin(ts);
	double y1 = ys + turning_r*cos(ts);

	double x2 = xg + turning_r*sin(tg);
	double y2 = yg - turning_r*cos(tg);

	double X = x2-x1;
	double Y = y2-y1;

	double theta = atan2(Y,X);
	//theta = RS_PATHs.Mod(theta,2*M_PI);

	double u1 = sqrt(pow(X,2) + pow(Y,2));
	if(u1<2*turning_r){
		return -1;
	}
	double u = sqrt(pow(u1,2) - pow(2*turning_r,2));
	double phi = atan2(u,2*turning_r);

	double t = M_PI/2 + theta - phi - ts;
	t = RS_PATHs.Mod(t,2*M_PI);


	double v = M_PI/2 + theta - phi - tg;
	v = RS_PATHs.Mod(v,2*M_PI);
	v = 2*M_PI - v;
	v = RS_PATHs.Mod(v,2*M_PI);

	double dist = turning_r*t + u + turning_r*v; // Total distance of path
	double y = dist;
	path_cost = y;
	//ROS_INFO("t: %lf, u: %lf, v: %lf",t,u,v);

	double tt = t+ts-M_PI/2;
	double xt = x1 + turning_r*cos(tt);
	double yt = y1 + turning_r*sin(tt);

	double tv = ts + t;
	double xv = xt + u*cos(tv);
	double yv = yt + u*sin(tv);
	double xc = xs;
	double yc = ys;
	double tc = ts;

	std::vector<tf::Point> path;

	double t1 = 0.0;
	while(t1 < t){

		tf::Point pt;
		pt.setValue(xc,yc,0.0);
		path.push_back(pt);

		t1 = t1 + d*(1/turning_r);
		xc = x1 + turning_r*cos(ts + t1 - M_PI/2);
		yc = y1 + turning_r*sin(ts + t1 - M_PI/2);

	}

	double s_line = 0;
	xc = xt;
	yc = yt;
	tc = tv;
	while(s_line < u){

		tf::Point pt;
		pt.setValue(xc,yc,0.0);
		path.push_back(pt);

		xc = xc + d*cos(tc);
		yc = yc + d*sin(tc);

		s_line = sqrt(pow(xt-xc,2) + pow(yt-yc,2));
	}

	double arc2 = 0.0;
	double t2 = 0.0;
	xc = xv;
	yc = yv;
	tc = tv;
	while(t2 < v){

		tf::Point pt;
		pt.setValue(xc,yc,0.0);
		path.push_back(pt);

		t2 = t2 + d*(1/turning_r);
		xc = x2 + turning_r*cos(tv + M_PI/2 + t2);
		yc = y2 + turning_r*sin(tv + M_PI/2 + t2);
	}

	rs_path.resize(path.size());
	rs_path = path;

	return path_cost;
}

double CRS_PATH::RSrR(const double xs, const double ys, const double ts, const double xg, const double yg, const double tg, std::vector<tf::Point>& rs_path){
	double path_cost;


	double x1 = xs + turning_r*sin(ts);
	double y1 = ys - turning_r*cos(ts);

	double x2 = xg + turning_r*sin(tg);
	double y2 = yg - turning_r*cos(tg);

	double X = x2-x1;
	double Y = y2-y1;

	double theta = atan2(Y,X);
	theta = RS_PATHs.Mod(theta,2*M_PI);

	double u = sqrt(pow(X,2) + pow(Y,2));

	double t = ts - theta;
	t = RS_PATHs.Mod(t,2*M_PI);

	double v = tg - theta;
	v = RS_PATHs.Mod(v,2*M_PI);

	double dist = turning_r*t + u + turning_r*v; // Total distance of path
	double y = dist;
	path_cost = y;
	//ROS_INFO("t: %lf, u: %lf, v: %lf",t,u,v);

	double tt = ts + M_PI/2 - t;
	double xt = x1 + turning_r*cos(tt);
	double yt = y1 + turning_r*sin(tt);

	double tv = ts - t;
	double xv = xt + u*cos(tv);
	double yv = yt + u*sin(tv);

	double xc = xs;
	double yc = ys;
	double tc = ts;

	std::vector<tf::Point> path;

	double t1 = 0.0;
	while(t1 < t){

		tf::Point pt;
		pt.setValue(xc,yc,0.0);
		path.push_back(pt);

		t1 = t1 + d*(1/turning_r);
		xc = x1 + turning_r*cos(ts - t1 + M_PI/2);
		yc = y1 + turning_r*sin(ts - t1 + M_PI/2);

	}

	double s_line = 0;
	xc = xt;
	yc = yt;
	tc = tv;
	while(s_line < u){

		tf::Point pt;
		pt.setValue(xc,yc,0.0);
		path.push_back(pt);

		xc = xc + d*cos(tc);
		yc = yc + d*sin(tc);

		s_line = sqrt(pow(xt-xc,2) + pow(yt-yc,2));
	}

	double arc2 = 0.0;
	double t2 = 0.0;
	xc = xv;
	yc = yv;
	tc = tv;
	while(t2 < v){

		tf::Point pt;
		pt.setValue(xc,yc,0.0);
		path.push_back(pt);

		t2 = t2 + d*(1/turning_r);
		xc = x2 + turning_r*cos(tv + M_PI/2 + t2);
		yc = y2 + turning_r*sin(tv + M_PI/2 + t2);
	}

	rs_path.resize(path.size());
	rs_path = path;

	return path_cost;
}


double CRS_PATH::LrSrR(const double xs, const double ys, const double ts, const double xg, const double yg, const double tg, std::vector<tf::Point>& rs_path){
	double path_cost;

	//tg = RS_PATHs.Mod(tg,2*M_PI);

	double x1 = xs - turning_r*sin(ts);
	double y1 = ys + turning_r*cos(ts);

	double x2 = xg + turning_r*sin(tg);
	double y2 = yg - turning_r*cos(tg);

	double X = x2-x1;
	double Y = y2-y1;

	double theta = atan2(Y,X);
	theta = RS_PATHs.Mod(theta,2*M_PI);

	double u1 = sqrt(pow(X,2) + pow(Y,2));
	if(u1<2*turning_r){
		return -1;
	}
	double u = sqrt(pow(u1,2) - pow(2*turning_r,2));
	double phi = atan2(u,2*turning_r);

	double t = M_PI/2 + theta - phi - ts;
	double v = M_PI/2 + theta - phi - tg;
	t = RS_PATHs.Mod(t,2*M_PI);
	v = RS_PATHs.Mod(v,2*M_PI);

	t = t + 2*phi; //for reverse straight
	v = v + 2*phi;
	t = RS_PATHs.Mod(t,2*M_PI);
	v = RS_PATHs.Mod(v,2*M_PI);

	v = 2*M_PI - v; //for reverse turning

	double dist = turning_r*t + u + turning_r*v; // Total distance of path
	double y = dist;
	path_cost = y;
	//ROS_INFO("t: %lf, u: %lf, v: %lf",t,u,v);

	double tt = ts - M_PI/2 + t;
	double xt = x1 + turning_r*cos(tt);
	double yt = y1 + turning_r*sin(tt);

	double tv = ts + t;
	double xv = xt - u*cos(tv);
	double yv = yt - u*sin(tv);

	double xc = xs;
	double yc = ys;
	double tc = ts;

	std::vector<tf::Point> path;

	double t1 = 0.0;
	while(t1 < t){

		tf::Point pt;
		pt.setValue(xc,yc,0.0);
		path.push_back(pt);

		t1 = t1 + d*(1/turning_r);
		xc = x1 + turning_r*cos(ts + t1 - M_PI/2);
		yc = y1 + turning_r*sin(ts + t1 - M_PI/2);

	}

	double s_line = 0;
	xc = xt;
	yc = yt;
	tc = tv;
	while(s_line < u){

		tf::Point pt;
		pt.setValue(xc,yc,0.0);
		path.push_back(pt);

		xc = xc - d*cos(tc);
		yc = yc - d*sin(tc);

		s_line = sqrt(pow(xt-xc,2) + pow(yt-yc,2));
	}

	double arc2 = 0.0;
	double t2 = 0.0;
	xc = xv;
	yc = yv;
	tc = tv;
	while(t2 < v){

		tf::Point pt;
		pt.setValue(xc,yc,0.0);
		path.push_back(pt);

		t2 = t2 + d*(1/turning_r);
		xc = x2 + turning_r*cos(tv + M_PI/2 + t2);
		yc = y2 + turning_r*sin(tv + M_PI/2 + t2);
	}

	rs_path.resize(path.size());
	rs_path = path;

	return path_cost;
}


double CRS_PATH::RrSrR(const double xs, const double ys, const double ts, const double xg, const double yg, const double tg, std::vector<tf::Point>& rs_path){
	double path_cost;

	//tg = RS_PATHs.Mod(tg,2*M_PI);

	double x1 = xs + turning_r*sin(ts);
	double y1 = ys - turning_r*cos(ts);

	double x2 = xg + turning_r*sin(tg);
	double y2 = yg - turning_r*cos(tg);

	double X = x2-x1;
	double Y = y2-y1;

	double theta = atan2(Y,X);
	theta = RS_PATHs.Mod(theta,2*M_PI);

	double u = sqrt(pow(X,2) + pow(Y,2));

	double t = ts - theta;
	double v = -tg + theta;
	t = RS_PATHs.Mod(t,2*M_PI);
	v = RS_PATHs.Mod(v,2*M_PI);

	t = t + M_PI; //for reverse straight
	v = v + M_PI;
	t = RS_PATHs.Mod(t,2*M_PI);
	v = RS_PATHs.Mod(v,2*M_PI);

	v = 2*M_PI - v; // for reverse turning

	double dist = turning_r*t + u + turning_r*v; // Total distance of path
	double y = dist;
	path_cost = y;
	//ROS_INFO("t: %lf, u: %lf, v: %lf",t,u,v);

	double tt = ts + M_PI/2 - t;
	double xt = x1 + turning_r*cos(tt);
	double yt = y1 + turning_r*sin(tt);

	double tv = ts - t;
	double xv = xt - u*cos(tv);
	double yv = yt - u*sin(tv);

	double xc = xs;
	double yc = ys;
	double tc = ts;

	std::vector<tf::Point> path;

	double t1 = 0.0;
	while(t1 < t){

		tf::Point pt;
		pt.setValue(xc,yc,0.0);
		path.push_back(pt);

		t1 = t1 + d*(1/turning_r);
		xc = x1 + turning_r*cos(ts - t1 + M_PI/2);
		yc = y1 + turning_r*sin(ts - t1 + M_PI/2);

	}

	double s_line = 0;
	xc = xt;
	yc = yt;
	tc = tv;
	while(s_line < u){

		tf::Point pt;
		pt.setValue(xc,yc,0.0);
		path.push_back(pt);

		xc = xc - d*cos(tc);
		yc = yc - d*sin(tc);

		s_line = sqrt(pow(xt-xc,2) + pow(yt-yc,2));
	}

	double arc2 = 0.0;
	double t2 = 0.0;
	xc = xv;
	yc = yv;
	tc = tv;
	while(t2 < v){

		tf::Point pt;
		pt.setValue(xc,yc,0.0);
		path.push_back(pt);

		t2 = t2 + d*(1/turning_r);
		xc = x2 + turning_r*cos(tv + M_PI/2 + t2);
		yc = y2 + turning_r*sin(tv + M_PI/2 + t2);
	}

	rs_path.resize(path.size());
	rs_path = path;

	return path_cost;
}










double CRS_PATH::Mod(double input, double divider){
	double mod = 0.0;

	if(input > 0){
		if(input < divider){
			mod = input;
		}
		else{
			while(1){
				input -= divider;
				if(input < divider) break;
			}
		}
		mod = input;
	}
	else if(input < 0){
		while(1){
			input += divider;
			if(input > 0) break;
		}
		mod = input;
	}
	else{
		mod = 0;
	}

	return mod;
}

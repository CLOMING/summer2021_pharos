#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pharos_msgs/WaypointOffset.h>


#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>


#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

static int start_flag = 0;
static int lateral_compensation_flag = 0;
static int vertical_compensation_flag = 0;

double lateral_offset_x_ = 0.0;
double lateral_offset_y_ = 0.0;
double vertical_offset_x_ = 0.0;
double vertical_offset_y_ = 0.0;

double init_heading = 0.0;
double init_lateral_error_angle = 0.0;
static int modified_init_ = 1;

class WaypointCompensatorNode
{
public:



	std::vector<tf::Point> origin_wp_;
	std::vector<tf::Point> modified_wp_;

	int32_t publish_rate_;


	ros::NodeHandlePtr node_;
	ros::NodeHandlePtr pnode_;

	ros::NodeHandle n_;

	ros::Publisher waypoint_pub_;
	ros::Publisher offset_pub_;

	ros::Subscriber waypoint_sub_;
	ros::Subscriber gps_sub_;



	double origin_gps_x_;
	double origin_gps_y_;

	WaypointCompensatorNode() :
		publish_rate_(100)
	{
	}
	~WaypointCompensatorNode()
	{
	}


	double deg2rad(const double deg){
		double rad;
		rad = deg * M_PI/180.0;
		return rad;
	}

	void WaypointCallback(const nav_msgs::PathConstPtr& wp){

		// Get origin waypoint

		tf::Point pt1;
		double x1,y1;

		origin_wp_.clear();

		for(int i=0; i < wp->poses.size(); i++){
			x1 = wp->poses[i].pose.position.x;
			y1 = wp->poses[i].pose.position.y;
			pt1.setValue(x1,y1,0.0);
			origin_wp_.push_back(pt1);
		}



		// Init modified waypoint as origin waypoint
		if(modified_init_ == 1){
			tf::Point pt2;
			double x2,y2;

			//modified_wp_.clear();
			modified_wp_.resize(origin_wp_.size());

			for(int i=0; i < wp->poses.size(); i++){
				x2 = wp->poses[i].pose.position.x;
				y2 = wp->poses[i].pose.position.y;
				pt2.setValue(x2,y2,0.0);
				modified_wp_[i].setValue(x2,y2,0.0);
			}
			modified_init_ = 0;
		}


	}

	void GPSCallback(const nav_msgs::OdometryConstPtr& gps){

		pharos_msgs::WaypointOffset offset;

		if(origin_wp_.size() == 0){
			return;
		}


		// 0. Get current position information ( GPS data )

		tf::Point current_pt;

		current_pt.setValue(gps->pose.pose.position.x, gps->pose.pose.position.y, 0.0);


		tf::Pose pose;
		tf::poseMsgToTF(gps->pose.pose, pose);

		double Roll, Pitch, Yaw;
		double heading = 0;

		pose.getBasis().getRPY(Roll, Pitch, Yaw);
		heading = Yaw;

		if(start_flag == 0){
			init_heading = heading;
			start_flag = 1;
		}
		//printf("%f %f\n",init_heading*180.0/M_PI, heading*180.0/M_PI);


		// 1.1 lateral error calculation



		double min_lat_dist = DBL_MAX;
		double lat_dist = 0;
		int lat_closest_wp_index;

		for(int i=0; i < modified_wp_.size(); i++){
			lat_dist = modified_wp_[i].distance(current_pt);
			if(lat_dist < min_lat_dist){
				min_lat_dist = lat_dist;
				lat_closest_wp_index = i;
			}
		}


		double x1,x2,y1,y2,xc,yc;
		double xl,yl; //lateral error point on the waypoint.

		xc = current_pt.x();
		yc = current_pt.y();

		x1 = modified_wp_[lat_closest_wp_index].x();
		y1 = modified_wp_[lat_closest_wp_index].y();

		if(lat_closest_wp_index+1 == modified_wp_.size()){
			x2 = modified_wp_[0].x();
			y2 = modified_wp_[0].y();
		}
		else{
			x2 = modified_wp_[lat_closest_wp_index+1].x();
			y2 = modified_wp_[lat_closest_wp_index+1].y();
		}

		double lateral_error = 0.0;
		lateral_error = abs( ( (xc-x1)*(y2-y1)-(yc-y1)*(x2-x1) ) / sqrt( pow(x2-x1,2)+pow(y2-y1,2) ) );

		double a_lat;
		a_lat = (y2-y1)/(x2-x1);

		xl = (yc + (1/a_lat)*xc - y2 + a_lat*x2)/(a_lat + 1/a_lat);
		yl = -1/a_lat*xl + yc +1/a_lat*xc;

		double lateral_error_vector_x = 0.0;
		double lateral_error_vector_y = 0.0;
		double lateral_error_angle = 0.0;

		lateral_error_vector_x = xl - xc;
		lateral_error_vector_y = yl - yc;
		lateral_error_angle = atan2(lateral_error_vector_y, lateral_error_vector_x);


		// 1.2 lateral error compensation

		if(lateral_compensation_flag == 0){
			init_lateral_error_angle = lateral_error_angle;
			for(int i=0; i<modified_wp_.size();i++){
				modified_wp_[i].setValue(modified_wp_[i].x()-lateral_error_vector_x, modified_wp_[i].y()-lateral_error_vector_y, 0.0);
			}
			lateral_offset_x_ = lateral_error_vector_x;
			lateral_offset_y_ = lateral_error_vector_y;
			lateral_compensation_flag = 1;
			ROS_INFO("Lateral Compensation finished");

			offset.lateral_x = lateral_offset_x_;
			offset.lateral_y = lateral_offset_y_;
			offset.vertical_x = vertical_offset_x_;
			offset.vertical_y = vertical_offset_y_;

			offset_pub_.publish(offset);
		}

		// 2.1 vertical error calculation

		double a_lon, b_lon, c_lon; // a_lon*x + b_lon*y + c_lon = 0 -> line's equation

		double vertical_angle = init_lateral_error_angle + M_PI/2;

		a_lon = tan(vertical_angle); // vertical direction with lateral compensation direction
		b_lon = -1;
		c_lon = yc - a_lon*xc;

		double min_lon_dist = DBL_MAX;
		double lon_dist = 0;
		int lon_closest_wp_index = 0;

		double xi,yi;

		for(int i=0; i < modified_wp_.size(); i++){

			xi = modified_wp_[i].x();
			yi = modified_wp_[i].y();

			lon_dist = abs(a_lon*xi - yi + yc -a_lon*xc)/sqrt(pow(a_lon,2) + pow(b_lon,2));

			if(sqrt(pow(xc-xi,2)+pow(yc-yi,2)) < 5.0){
				if(lon_dist < min_lon_dist){
					min_lon_dist = lon_dist;
					lon_closest_wp_index = i;
				}
			}
		}
		double vertical_error;

		vertical_error = modified_wp_[lon_closest_wp_index].distance(current_pt);

		double vertical_error_vector_x = 0.0;
		double vertical_error_vector_y = 0.0;
		double vertical_error_angle = 0.0;

		vertical_error_vector_x = modified_wp_[lon_closest_wp_index].x() - xc;
		vertical_error_vector_y = modified_wp_[lon_closest_wp_index].y() - yc;


		// 2.2 vertical error compensation
		//printf("vertical: %d\n",vertical_compensation_flag);
//		if(vertical_compensation_flag == 0){
//
//			double angle_error;
//			angle_error = abs(init_heading - heading);
//			//printf("1. %.2f\n",angle_error);
//			if(angle_error > M_PI){
//				angle_error -= 2*M_PI;
//			}
//			else if(angle_error < -M_PI){
//				angle_error += 2*M_PI;
//			}
//			//printf("2. %.2f\n",angle_error);
//			angle_error = abs(angle_error);
//			//printf("3. %.2f\n",angle_error);
//			//printf("init heading: %.2f, current: %.2f, error: %.2f\n",init_heading*180/M_PI,heading*180/M_PI,angle_error);
//
//			if( (angle_error > deg2rad(30.0)) && (vertical_error > 0.5)){
//				for(int i=0; i<modified_wp_.size();i++){
//					modified_wp_[i].setValue(modified_wp_[i].x()-vertical_error_vector_x, modified_wp_[i].y()-vertical_error_vector_y, 0.0);
//				}
//				vertical_offset_x_ = vertical_error_vector_x;
//				vertical_offset_y_ = vertical_error_vector_y;
//				vertical_compensation_flag = 1;
//				ROS_INFO("Vertical Compensation finished");
//
//				offset.lateral_x = lateral_offset_x_;
//				offset.lateral_y = lateral_offset_y_;
//				offset.vertical_x = vertical_offset_x_;
//				offset.vertical_y = vertical_offset_y_;
//
//				offset_pub_.publish(offset);
//			}
//		}
//		else if( (vertical_compensation_flag == 1) && (vertical_error > 0.4) ){
//			for(int i=0; i<modified_wp_.size();i++){
//				modified_wp_[i].setValue(modified_wp_[i].x()-vertical_error_vector_x, modified_wp_[i].y()-vertical_error_vector_y, 0.0);
//			}
//
//			ROS_INFO("Vertical Compensation again");
//		}



	}







	// Load parameters etc
	int init()
	{
		node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
		pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

		pnode_->param("publish_rate", publish_rate_, 100);


		offset_pub_ = node_->advertise<pharos_msgs::WaypointOffset>(std::string("/waypoint_offset"), 100);
		//waypoint_pub_ = node_->advertise<nav_msgs::Path>(std::string("/road_data/compensated_waypoint"), 100);

		//if(sub_flag_ == 1){
			waypoint_sub_ = node_->subscribe<nav_msgs::Path>("/road_data/waypoint",10, &WaypointCompensatorNode::WaypointCallback, this);
			//sub_flag_ = 0;
		//}
		gps_sub_ = node_->subscribe<nav_msgs::Odometry>("/ekf_odom", 100, &WaypointCompensatorNode::GPSCallback, this);

		return 0;
	}


	// Publish data
	void publish()
	{


		ros::Rate loop_rate(publish_rate_);




		while (node_->ok())
		{



//			nav_msgs::Path waypoint;
//
//			waypoint.header.stamp = ros::Time::now();
//			waypoint.header.frame_id = "/odom";
//
//			waypoint.poses.resize(modified_wp_.size());
//
//			if(modified_wp_.size() > 0){
//				for(int i=0; i<modified_wp_.size();i++){
//					waypoint.poses[i].pose.position.x = modified_wp_[i].x();
//					waypoint.poses[i].pose.position.y = modified_wp_[i].y();
//					waypoint.poses[i].pose.position.z = 0.0;
//				}
//			}
//
//
//			waypoint_pub_.publish(waypoint);



			ros::spinOnce();
			loop_rate.sleep();
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_compensator");

	WaypointCompensatorNode rp_ros;
	if (rp_ros.init())
	{
		ROS_FATAL("WaypointCompensatorNode initialization failed");
		return -1;
	}

	rp_ros.publish();

	return 0;
}

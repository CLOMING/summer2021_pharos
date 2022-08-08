/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <pharos_msgs/StateStamped2016.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <novatel_gps_msgs/Inspva.h>
#include <ublox_msgs/NavPVT.h>

#include <tf/transform_listener.h>
#include <tf/tfMessage.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry>

using namespace gps_common;

static ros::Publisher novatel_odom_pub;
static ros::Publisher ublox_odom_pub;
static ros::Publisher ublox2_odom_pub;
static ros::Publisher vehicle_odom_pub;
static ros::Publisher vehicle_wgs_pub;

bool relative_gps;
bool relative_gps_init_;
bool vehicle = false;
bool ublox2_init = false;
bool dgps_heading_;

double origin_x_ = 0.0;
double origin_y_ = 0.0;
double origin_z_ = 0.0;
std::string zone_;

double heading_offset;

std::string frame_id, novatel_frame_id, ublox_frame_id, ublox2_frame_id, vehicle_frame;

nav_msgs::Odometry novatel_odom_;
nav_msgs::Odometry ublox_odom_;
nav_msgs::Odometry ublox2_odom_;
nav_msgs::Odometry vehicle_odom_;
nav_msgs::Odometry vehicle_odom;
nav_msgs::Odometry vehicle_wgs_;

double ublox_yaw = 0.0;
double ublox2_yaw = 0.0;
double antOffset = 0.0;
double vehicle_speed = 0.0;

tf::Quaternion heading_quat_;
tf::Quaternion heading_quat2_;
tf::StampedTransform transform;


Eigen::Vector3f getRPYfromRM(Eigen::Matrix3f rotation_matrix){
	Eigen::Vector3f rpy;
	rpy[0] = atan2f(rotation_matrix(2,1),rotation_matrix(2,2));
	rpy[1] = atan2f(-rotation_matrix(2,0),sqrt(pow(rotation_matrix(2,1),2)+pow(rotation_matrix(2,2),2)));
	rpy[2] = atan2f(rotation_matrix(1,0),rotation_matrix(0,0));

	return rpy;
}

void Relative_gps_init(double origin_x, double origin_y, double origin_z){

  if(origin_x==0||origin_y==0||origin_z==0){
    ROS_ERROR("gps recived without fix");
    return;
  }
  
  bool relative_gps_fix_;
  ros::param::get("~/relative_gps_fix", relative_gps_fix_);

  if( relative_gps_fix_ ){

    ros::param::get("~/fix_origin_x", origin_x);
    ros::param::get("~/fix_origin_y", origin_y);
    // ros::param::get("~/fix_origin_z", origin_z);

  }

  ros::param::set("gps/origin/x", origin_x);
  ros::param::set("gps/origin/y", origin_y);
  ros::param::set("gps/origin/z", origin_z);

  origin_x_ = origin_x;
  origin_y_ = origin_y;
  origin_z_ = origin_z;

  relative_gps_init_ = true;

  ROS_INFO("origin_x = %f, origin_y = %f, origin_z = %f",origin_x, origin_y, origin_z);

  return;
}

void Inspva_callback(const novatel_gps_msgs::InspvaConstPtr& fix) {

	if (fix->header.stamp == ros::Time(0)) {
		return;
	}

	double northing, easting;
	std::string zone;

	LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

	if (novatel_odom_pub) {
    	if(!relative_gps_init_) Relative_gps_init(easting, northing, fix->height);

		double roll,pitch,yaw;

			roll = fix->roll*M_PI/180;
			pitch = fix->pitch*-M_PI/180;
			yaw = fix->azimuth;
			yaw = yaw*M_PI/180.0;
			if(yaw < (M_PI/2.0)){
					yaw = yaw+(2*M_PI)-M_PI/2.0;
			}
			else{
					yaw -= M_PI/2.0;
			}
			yaw = 2*M_PI - yaw;

		geometry_msgs::Quaternion heading_quat;
		heading_quat_.setRPY(roll,pitch,yaw);
		tf::quaternionTFToMsg(heading_quat_,heading_quat);

		novatel_odom_.header.stamp = fix->header.stamp;

		if (frame_id.empty())
			novatel_odom_.header.frame_id = fix->header.frame_id;
		else
			novatel_odom_.header.frame_id = frame_id;

		novatel_odom_.child_frame_id = novatel_frame_id;

		if(relative_gps){
			novatel_odom_.pose.pose.position.x = easting - origin_x_;
			novatel_odom_.pose.pose.position.y = northing - origin_y_ ;
			novatel_odom_.pose.pose.position.z = fix->height - origin_z_ ;
		}
		else{
			novatel_odom_.pose.pose.position.x = easting;
			novatel_odom_.pose.pose.position.y = northing;
			novatel_odom_.pose.pose.position.z = fix->height;
		}


		novatel_odom_.pose.pose.orientation = heading_quat;
		novatel_odom_pub.publish(novatel_odom_);
		if(vehicle_frame=="novatel_gps"){
			vehicle=true;
			vehicle_odom = novatel_odom_;
		}
	}
}


void Ublox_navpvt_callback(const ublox_msgs::NavPVTConstPtr& fix) {
	double yaw;
	yaw = fix->heading;
	yaw = (yaw/100000.0-heading_offset)*M_PI/180.0;
	if(yaw < (M_PI/2.0)){
		yaw = yaw+(2*M_PI)-M_PI/2.0;
	}
	else{
		yaw -= M_PI/2.0;
	}
	yaw = 2*M_PI - yaw;
	heading_quat_.setRPY(0,0,yaw);
	ublox_yaw = yaw;
}


void Ublox2_navpvt_callback(const ublox_msgs::NavPVTConstPtr& fix) {
	double yaw;
	yaw = fix->heading;
	yaw = (yaw/100000.0)*M_PI/180.0;
	if(yaw < (M_PI/2.0)){
		yaw = yaw+(2*M_PI)-M_PI/2.0;
	}
	else{
		yaw -= M_PI/2.0;
	}
	yaw = 2*M_PI - yaw;
	heading_quat2_.setRPY(0,0,yaw);
	ublox2_yaw = yaw;
}


void Ublox_fix_callback(const sensor_msgs::NavSatFixConstPtr& fix) {

	if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
		ROS_WARN("No fix.");
		return;
	}
	if (fix->header.stamp == ros::Time(0)) {
		return;
	}

	double northing, easting;
	std::string zone;

	LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);
	zone_ = zone;

	if (ublox_odom_pub) {

    	if(!relative_gps_init_) Relative_gps_init(easting, northing, fix->altitude);

		ublox_odom_.header.stamp = fix->header.stamp;

		if (frame_id.empty())
			ublox_odom_.header.frame_id = fix->header.frame_id;
		else
			ublox_odom_.header.frame_id = frame_id;

		ublox_odom_.child_frame_id = ublox_frame_id;

		if(relative_gps){
			ublox_odom_.pose.pose.position.x = easting - origin_x_;
			ublox_odom_.pose.pose.position.y = northing - origin_y_ ;
			ublox_odom_.pose.pose.position.z = fix->altitude - origin_z_ ;
		}
		else{
			ublox_odom_.pose.pose.position.x = easting;
			ublox_odom_.pose.pose.position.y = northing;
			ublox_odom_.pose.pose.position.z = fix->altitude;
		}
		ublox_odom_.pose.pose.position.x -= antOffset*cos(ublox_yaw);
		ublox_odom_.pose.pose.position.y -= antOffset*sin(ublox_yaw);

		if(vehicle_speed==0 && ublox2_init)
		{
			double x_vec = ublox2_odom_.pose.pose.position.x-ublox_odom_.pose.pose.position.x;
			double y_vec = ublox2_odom_.pose.pose.position.y-ublox_odom_.pose.pose.position.y;

			heading_quat_.setRPY(0,0,atan2f(y_vec,x_vec));
		}

		geometry_msgs::Quaternion heading_quat;
		tf::quaternionTFToMsg(heading_quat_,heading_quat);
		ublox_odom_.pose.pose.orientation = heading_quat;

		// Use ENU covariance to build XYZRPY covariance
		boost::array<double, 36> covariance = {{
															 fix->position_covariance[0],
															 fix->position_covariance[1],
															 fix->position_covariance[2],
															 0, 0, 0,
															 fix->position_covariance[3],
															 fix->position_covariance[4],
															 fix->position_covariance[5],
															 0, 0, 0,
															 fix->position_covariance[6],
															 fix->position_covariance[7],
															 fix->position_covariance[7],
															 0, 0, 0,
															 0, 0, 0, fix->position_covariance[6], 0, 0,
															 0, 0, 0, 0, fix->position_covariance[7], 0,
															 0, 0, 0, 0, 0, fix->position_covariance[8]
															}};

		ublox_odom_.pose.covariance = covariance;
		ublox_odom_pub.publish(ublox_odom_);
		if(vehicle_frame == "ublox_ant"){
			vehicle=true;
			vehicle_odom = ublox_odom_;
		}
	}
}

void Ublox2_fix_callback(const sensor_msgs::NavSatFixConstPtr& fix) {
	if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
		ROS_WARN("ublox2 No fix.");
		return;
	}
	if (fix->header.stamp == ros::Time(0)) return;

	double northing, easting;
	std::string zone;

	LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

	if (ublox2_odom_pub) {

    	if(!relative_gps_init_) Relative_gps_init(easting, northing, fix->altitude);

		ublox2_odom_.header.stamp = fix->header.stamp;

		if (frame_id.empty())
			ublox2_odom_.header.frame_id = fix->header.frame_id;
		else
			ublox2_odom_.header.frame_id = frame_id;

		ublox2_odom_.child_frame_id = ublox2_frame_id;

		if(relative_gps){
			ublox2_odom_.pose.pose.position.x = easting - origin_x_;
			ublox2_odom_.pose.pose.position.y = northing - origin_y_ ;
			ublox2_odom_.pose.pose.position.z = fix->altitude - origin_z_ ;
		}
		else{
			ublox2_odom_.pose.pose.position.x = easting;
			ublox2_odom_.pose.pose.position.y = northing;
			ublox2_odom_.pose.pose.position.z = fix->altitude;
		}

		geometry_msgs::Quaternion heading_quat;
		tf::quaternionTFToMsg(heading_quat2_,heading_quat);
		ublox2_odom_.pose.pose.orientation = heading_quat;

		// Use ENU covariance to build XYZRPY covariance
		boost::array<double, 36> covariance = {{
															 fix->position_covariance[0],
															 fix->position_covariance[1],
															 fix->position_covariance[2],
															 0, 0, 0,
															 fix->position_covariance[3],
															 fix->position_covariance[4],
															 fix->position_covariance[5],
															 0, 0, 0,
															 fix->position_covariance[6],
															 fix->position_covariance[7],
															 fix->position_covariance[7],
															 0, 0, 0,
															 0, 0, 0, fix->position_covariance[6], 0, 0,
															 0, 0, 0, 0, fix->position_covariance[7], 0,
															 0, 0, 0, 0, 0, fix->position_covariance[8]
															}};

		ublox2_odom_.pose.covariance = covariance;
		ublox2_odom_pub.publish(ublox2_odom_);

		ublox2_init = true;
	}
}

void Mcl_callback(const nav_msgs::OdometryConstPtr& fix) {
	if(vehicle_frame == "mcl"){
		vehicle=true;
		vehicle_odom = *fix;
	}
}

void Ekf_callback(const nav_msgs::OdometryConstPtr& fix) {
	if(vehicle_frame == "ublox_ekf"){
		vehicle=true;
		vehicle_odom = *fix;
	}
	if(vehicle_frame == "mcl_ekf"){
		vehicle=true;
		vehicle_odom = *fix;
	}

	double Lat, Long;

	UTMtoLL(fix->pose.pose.position.y + origin_y_, fix->pose.pose.position.x + origin_x_, zone_, Lat, Long);

	vehicle_wgs_.pose.pose.position.x = Lat;
	vehicle_wgs_.pose.pose.position.y = Long;
	vehicle_wgs_.pose.pose.position.z = 0.0;

	vehicle_wgs_pub.publish(vehicle_wgs_);
}

void VehicleState_callback(const pharos_msgs::StateStamped2016Ptr& msg) {// pub /odom/vehicle
	vehicle_speed = msg->state.velocity;

	vehicle_odom_ = vehicle_odom;
	vehicle_odom_.child_frame_id = "vehicle_frame";

	if(vehicle) vehicle_odom_pub.publish(vehicle_odom_);
}

int main (int argc, char **argv) {
	ros::init(argc, argv, "pharos_utm_odometry_node");

	tf::TransformListener listener;
	try{
		listener.lookupTransform("ublox_ant", "vehicle_frame", ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	ros::NodeHandle priv_node("~");
	ros::NodeHandle node("");

	node.param<std::string>("frame_id", frame_id, "odom");
	node.param<std::string>("vehicle_frame_id", vehicle_frame, "ublox_ekf");

	node.param<std::string>("novatel_frame_id", novatel_frame_id, "novatel_gps");
	node.param<std::string>("ublox_frame_id", ublox_frame_id, "ublox_ant");
	node.param<std::string>("ublox2_frame_id", ublox2_frame_id, "ublox2_ant");
	node.param("dgps_heading", dgps_heading_, false);
	
	priv_node.param<double>("antOffset", antOffset, 0.0);
	priv_node.param("relative_gps", relative_gps, true);
	priv_node.param<double>("heading_offset", heading_offset, 0.0);

	novatel_odom_pub = node.advertise<nav_msgs::Odometry>("/odom/novatel", 100);
	ublox_odom_pub = node.advertise<nav_msgs::Odometry>("/odom/ublox", 100);
	ublox2_odom_pub = node.advertise<nav_msgs::Odometry>("/odom/ublox2", 100);
	vehicle_odom_pub = node.advertise<nav_msgs::Odometry>("/odom/vehicle", 100);
	vehicle_wgs_pub = node.advertise<nav_msgs::Odometry>("/wgs/vehicle", 100);

	ros::Subscriber inspva_sub = node.subscribe("/inspva", 10, Inspva_callback);
	ros::Subscriber fix_sub = node.subscribe("/ublox/fix", 10, Ublox_fix_callback);
	ros::Subscriber NavPVT_sub = node.subscribe("/ublox/navpvt", 10, Ublox_navpvt_callback);
	ros::Subscriber fix_sub2 = node.subscribe("/ublox2/fix", 10, Ublox2_fix_callback);
	ros::Subscriber NavPVT_sub2 = node.subscribe("/ublox2/navpvt", 10, Ublox2_navpvt_callback);
	ros::Subscriber mcl_sub = node.subscribe("/odom/mcl", 10, Mcl_callback);
	ros::Subscriber ekf_sub = node.subscribe("/odom/ekf", 10, Ekf_callback);
	ros::Subscriber VehicleState_sub = node.subscribe("/vehicle/state2016", 10, VehicleState_callback);

	ros::spin();
}
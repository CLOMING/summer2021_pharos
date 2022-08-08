/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <ublox_msgs/NavPVT.h>

#include <tf/transform_listener.h>
#include <tf/tfMessage.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry>

using namespace gps_common;

static ros::Publisher ublox_odom_pub;
static ros::Publisher ublox_ant_odom_pub;
static ros::Publisher vehicle_odom_pub;
static ros::Publisher vehicle_wgs_pub;

bool relative_gps_;
bool relative_gps_init_;
bool vehicle = false;

double origin_x_ = 0.0;
double origin_y_ = 0.0;
double origin_z_ = 0.0;
std::string zone_;

double heading_offset;

std::string frame_id, ublox_frame_id, vehicle_frame;

double ublox_yaw = 0.0;
double ant_offset_x_ = 0.0;
double ant_offset_y_ = 0.0;

tf::Quaternion heading_quat_;

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
  
  bool relative_gps_fix_ = false;
  ros::param::get("~/relative_gps_fix", relative_gps_fix_);

  if( relative_gps_fix_ ){

    ros::param::get("~/fix_origin_x", origin_x);
    ros::param::get("~/fix_origin_y", origin_y);
    // ros::param::get("~/fix_origin_z", origin_z);
    printf("relative gps fix: true\nx: %f y: %f\n",origin_x,origin_y);

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



void vehicle_frame_publish(nav_msgs::Odometry fix) {// pub /odom/vehicle
  static ros::Time prev_stamp;
  static nav_msgs::Odometry vehicle_odom;
  vehicle_odom = fix;
  vehicle_odom.child_frame_id = "vehicle_frame";

  if(fix.header.stamp != prev_stamp){
    vehicle_odom_pub.publish(vehicle_odom);
    prev_stamp = fix.header.stamp;
  }

  // Publish WGS84 Coordinate in latitude and longitude
  double Lat, Long;
  UTMtoLL(vehicle_odom.pose.pose.position.y + origin_y_, vehicle_odom.pose.pose.position.x + origin_x_, zone_, Lat, Long);

  static nav_msgs::Odometry vehicle_wgs;
  vehicle_wgs.header = fix.header;
  vehicle_wgs.pose.pose.position.x = Lat;
  vehicle_wgs.pose.pose.position.y = Long;
  vehicle_wgs.pose.pose.position.z = 0.0;

  vehicle_wgs_pub.publish(vehicle_wgs);
}

void Ublox_navpvt_callback(const ublox_msgs::NavPVTConstPtr& fix) {

  static tf::TransformListener listener;
  static tf::StampedTransform transform_;
  static bool init_retry = true;
  if(init_retry){
    try{
      listener.lookupTransform("ublox_ant", "vehicle_frame", ros::Time(0), transform_);
    }
    catch (tf::TransformException ex){
      init_retry = true;
      ros::Duration(1.0).sleep();
      return;
    }
  }
  init_retry = false;

  if (ublox_odom_pub) {
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


    double ublox_heading = 0.0;

    ublox_heading = (double)fix->position_covariance[1]; //unit: deg/1e-5
    ublox_heading = ublox_heading/100000.0;
    ublox_heading = ublox_heading*M_PI/180.0;

    if(ublox_heading < (M_PI/2.0)){

      ublox_heading = ublox_heading+(2*M_PI)-M_PI/2.0;

    }else{

      ublox_heading -= M_PI/2.0;

    }

    ublox_heading = 2*M_PI - ublox_heading;

//    geometry_msgs::Quaternion heading_quat;
//    heading_quat_.setRPY(0.0,0.0,ublox_heading);
//    tf::quaternionTFToMsg(heading_quat_,heading_quat);


    static nav_msgs::Odometry ublox_odom;
    static nav_msgs::Odometry ublox_ant_odom;
    ublox_odom.header.stamp = fix->header.stamp;

    if (frame_id.empty())
      ublox_odom.header.frame_id = fix->header.frame_id;
    else
      ublox_odom.header.frame_id = frame_id;


    ublox_odom.child_frame_id = ublox_frame_id;


    if(relative_gps_){

      ublox_odom.pose.pose.position.x = easting - origin_x_;
      ublox_odom.pose.pose.position.y = northing - origin_y_ ;
      ublox_odom.pose.pose.position.z = fix->altitude - origin_z_ ;

    }else{

      ublox_odom.pose.pose.position.x = easting;
      ublox_odom.pose.pose.position.y = northing;
      ublox_odom.pose.pose.position.z = fix->altitude;

    }

    geometry_msgs::Quaternion heading_quat;
    tf::quaternionTFToMsg(heading_quat_,heading_quat);
    ublox_odom.pose.pose.orientation = heading_quat;

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

    ublox_odom.pose.covariance = covariance;
  
    ublox_ant_odom = ublox_odom;
    ublox_ant_odom_pub.publish(ublox_ant_odom);

    //Calculate Antenna Offset
    ublox_odom.pose.pose.position.x -= ant_offset_x_*cos(ublox_yaw);
    ublox_odom.pose.pose.position.y -= ant_offset_x_*sin(ublox_yaw);
    ublox_odom.pose.pose.position.x -= ant_offset_y_*cos(ublox_yaw+M_PI/2);
    ublox_odom.pose.pose.position.y -= ant_offset_y_*sin(ublox_yaw+M_PI/2);
    static bool isFirst = true;
    if(isFirst){
      isFirst = false;
      return;
    }
    ublox_odom_pub.publish(ublox_odom);

    if(vehicle_frame == "gps") vehicle_frame_publish(ublox_odom);
    if(vehicle_frame == "ublox_gps") vehicle_frame_publish(ublox_odom);
  }
}

void Mcl_callback(const nav_msgs::OdometryConstPtr& fix) {

  if(vehicle_frame == "mcl") vehicle_frame_publish(*fix);

}

void Ekf_callback(const nav_msgs::OdometryConstPtr& fix) {

  if(vehicle_frame == "ekf") vehicle_frame_publish(*fix);
  if(vehicle_frame == "ublox_ekf") vehicle_frame_publish(*fix);
  if(vehicle_frame == "mcl_ekf") vehicle_frame_publish(*fix);

}

int main (int argc, char **argv) {
  ros::init(argc, argv, "pharos_utm_odometry_node");

  ros::NodeHandle priv_node("~");
  ros::NodeHandle node("");

  node.param<std::string>("frame_id", frame_id, "odom");
  node.param<std::string>("vehicle_frame_id", vehicle_frame, "ublox_ekf");

  node.param<std::string>("ublox_frame_id", ublox_frame_id, "ublox_gps");
  
  priv_node.param<double>("ant_offset_x", ant_offset_x_, 0.0);
  priv_node.param<double>("ant_offset_y", ant_offset_y_, 0.0);
  priv_node.param("relative_gps", relative_gps_, true);
  priv_node.param<double>("heading_offset", heading_offset, 0.0);

  ublox_odom_pub = node.advertise<nav_msgs::Odometry>("/odom/ublox", 100);
  ublox_ant_odom_pub = node.advertise<nav_msgs::Odometry>("/odom/ublox_ant", 100);
  vehicle_odom_pub = node.advertise<nav_msgs::Odometry>("/odom/vehicle", 100);
  vehicle_wgs_pub = node.advertise<nav_msgs::Odometry>("/wgs/vehicle", 100); //lat lon

  ros::Subscriber fix_sub = node.subscribe("/ublox/fix", 10, Ublox_fix_callback);
  ros::Subscriber NavPVT_sub = node.subscribe("/ublox/navpvt", 10, Ublox_navpvt_callback);
  ros::Subscriber mcl_sub = node.subscribe("/odom/mcl", 10, Mcl_callback);
  ros::Subscriber ekf_sub = node.subscribe("/odom/ekf", 10, Ekf_callback);

  ros::spin();
}
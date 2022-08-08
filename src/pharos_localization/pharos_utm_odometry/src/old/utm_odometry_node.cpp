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
#include <novatel_gps_msgs/Inspva.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h> // have to be deleted
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>

using namespace gps_common;

static ros::Publisher odom_pub;

double origin_x_ = 0.0;
double origin_y_ = 0.0;
double origin_z_ = 0.0;


std::string frame_id, child_frame_id;
double rot_cov;

void callback(const novatel_gps_msgs::InspvaConstPtr& fix) {
  // if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
  //   ROS_INFO("No fix.");
  //   return;
  // }

  if (fix->header.stamp == ros::Time(0)) {
    return;
  }

  double northing, easting;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

  if (odom_pub) {

    //relative coordinate setting
    if(origin_x_==0.0 && origin_y_==0.0){
        origin_x_ = easting;
        origin_y_ = northing;
        origin_z_ = fix->height;
    }


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
    tf::Quaternion ublox_qh;
    ublox_qh.setRPY(roll,pitch,yaw);
    tf::quaternionTFToMsg(ublox_qh,heading_quat);


    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    if (frame_id.empty())
      odom.header.frame_id = fix->header.frame_id;
    else
      odom.header.frame_id = frame_id;

    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = easting ;
    odom.pose.pose.position.y = northing ;
    odom.pose.pose.position.z = fix->height ;

    odom.pose.pose.orientation = heading_quat;

    odom_pub.publish(odom);

  }
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  priv_node.param<std::string>("frame_id", frame_id, "map");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "imu");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);

  odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);
  ros::Subscriber fix_sub = node.subscribe("/inspva", 10, callback);

  ros::spin();
}


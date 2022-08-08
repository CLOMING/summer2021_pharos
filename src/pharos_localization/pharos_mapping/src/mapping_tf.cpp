
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>

Eigen::Vector3f getRPYfromRM(Eigen::Matrix3f rotation_matrix){
    Eigen::Vector3f rpy;
    rpy[0] = atan2f(rotation_matrix(2,1),rotation_matrix(2,2));
    rpy[1] = atan2f(-rotation_matrix(2,0),sqrt(pow(rotation_matrix(2,1),2)+pow(rotation_matrix(2,2),2)));
    rpy[2] = atan2f(rotation_matrix(1,0),rotation_matrix(0,0));

    return rpy;
}

void PoseCB(geometry_msgs::PoseStampedConstPtr msg){
    static tf::TransformBroadcaster br;

    tf::Transform transform;
    tf::Quaternion quat;
    transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );

    quat.setX(msg->pose.orientation.x);
    quat.setY(msg->pose.orientation.y);
    quat.setZ(msg->pose.orientation.z);
    quat.setW(msg->pose.orientation.w);

    transform.setRotation(quat);

    br.sendTransform(tf::StampedTransform(transform,msg->header.stamp, msg->header.frame_id, "base_link"));

    Eigen::AngleAxisf r_rotation_x (0.0*M_PI/180,Eigen::Vector3f::UnitX ());
    Eigen::AngleAxisf r_rotation_y (0.0*M_PI/180,Eigen::Vector3f::UnitY ());
    Eigen::AngleAxisf r_rotation_z (0.0*M_PI/180,Eigen::Vector3f::UnitZ ());
    Eigen::Matrix3f r_rm;
    r_rm = r_rotation_z*r_rotation_y*r_rotation_x;
    Eigen::Vector3f r_velodyne_rpy = getRPYfromRM(r_rm);
    tf::Transform r_Velo_Transform;
    r_Velo_Transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
    tf::Quaternion r_velodyne_qt;
    r_velodyne_qt.setRPY(r_velodyne_rpy[0],r_velodyne_rpy[1],r_velodyne_rpy[2]);
    r_Velo_Transform.setRotation(r_velodyne_qt);
    br.sendTransform(tf::StampedTransform(r_Velo_Transform, msg->header.stamp, "base_link", "right_velodyne"));
}

int main(int argc, char** argv){
    ros::init(argc,argv,"mapping_tf_node");

    ros::NodeHandlePtr node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));

    ros::Subscriber pose_sub = node_->subscribe<geometry_msgs::PoseStamped>("current_pose",1,PoseCB);

    ros::spin();
    return 0;
}
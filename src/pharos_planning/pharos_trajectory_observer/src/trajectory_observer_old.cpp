#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pharos_msgs/MotionCommandStamped3.h>

#include <pharos_msgs/SpeedCommand.h>
#include <pharos_msgs/StateStamped2016.h>

#include <pharos_path_planner/RoadInfo.h>
#include <pharos_path_planner/ReferencePath.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#define NumWpSearch 30

#define Gear_P 1
#define Gear_R 2
#define Gear_N 3
#define Gear_D 4

nav_msgs::PathPtr ref_waypoint_(new nav_msgs::Path); 
nav_msgs::PathPtr ref_curvature_(new nav_msgs::Path);
nav_msgs::PathPtr ref_speed_(new nav_msgs::Path);

// pharos_path_planner::ReferencePathPtr ref_roadinfo_(new pharos_path_planner::ReferencePath);

class TrajectoryObserverNode
{
public:
    int32_t publish_rate_;
    ros::NodeHandlePtr node_;
    ros::NodeHandlePtr pnode_;

    double vehicle_x_;
    double vehicle_y_;
    double vehicle_z_;

    double lookahaed_distance_;

    double front_obstacle_dist_;

    ros::Publisher motion_cmd_pub_;
    ros::Publisher state_marker_pub_;
    
    ros::Subscriber planner_path_sub_;
    ros::Subscriber planner_speed_sub_;
    ros::Subscriber planner_curvature_sub_;

    ros::Subscriber vehicle_odom_sub_;
    ros::Subscriber vehicle_state_sub_;
    ros::Subscriber lookahead_dist_sub_;

    ros::Subscriber reference_path_sub_;
    ros::Subscriber reference_speed_sub_;
    ros::Subscriber reference_curvature_sub_;

    
    pharos_msgs::StateStamped2016 vehicle_state_;
    pharos_msgs::MotionCommandStamped3 motion_cmd_;


    bool planner_path_get_;
    bool planner_speed_get_;
    bool planner_curvature_get_;

    bool reference_path_get_;
    bool reference_speed_get_;
    bool reference_curvature_get_;

    bool reference_path_following_;

    struct SteeringControlParam{
        double heading_difference;
        double lateral_offset;
        double curvature;
    };

    void PlannerPathCallback(const nav_msgs::PathConstPtr& msg){
        if(reference_path_following_) return;
        *ref_waypoint_ = *msg;
        planner_path_get_ = true;
    }
    void PlannerSpeedCallback(const nav_msgs::PathConstPtr& msg){
        if(reference_path_following_) return;
        *ref_speed_ = *msg;
        planner_speed_get_ = true;
    }
    void PlannerCurvatureCallback(const nav_msgs::PathConstPtr& msg){
        if(reference_path_following_) return;
        *ref_curvature_ = *msg;
        planner_curvature_get_ = true;
    }
    
    void ReferecePathCallback(const nav_msgs::PathConstPtr& msg){
        ref_waypoint_->poses.clear();

        geometry_msgs::PoseStamped wp_pose;
        printf("reference path callback is running !!! \n");

        nav_msgs::PathPtr ref_path_ptr (new nav_msgs::Path);
        *ref_path_ptr = *msg;

        std::vector<geometry_msgs::PoseStamped>::iterator iter;
        for(iter=ref_path_ptr->poses.begin();iter!=ref_path_ptr->poses.end();++iter){

            wp_pose.pose.position.x = iter->pose.position.x;
            wp_pose.pose.position.y = iter->pose.position.y;
            wp_pose.pose.position.z = 0;

            ref_waypoint_->poses.push_back(wp_pose);

        }

        reference_path_get_ = true;

    }

    void RefereceSpeedCallback(const nav_msgs::PathConstPtr& msg){

        ref_speed_->poses.clear();

        geometry_msgs::PoseStamped speed_pose;


        nav_msgs::PathPtr ref_speed_ptr (new nav_msgs::Path);
        *ref_speed_ptr = *msg;

        std::vector<geometry_msgs::PoseStamped>::iterator iter;
        for(iter=ref_speed_ptr->poses.begin();iter!=ref_speed_ptr->poses.end();++iter){

            speed_pose.pose.position.x = iter->pose.position.x;
            speed_pose.pose.position.y = iter->pose.position.y;
            speed_pose.pose.position.z = iter->pose.position.z;

            ref_speed_->poses.push_back(speed_pose);

        }

        reference_speed_get_ = true;
    }

    void RefereceCurvatureCallback(const nav_msgs::PathConstPtr& msg){
        
        ref_curvature_->poses.clear();
        
        geometry_msgs::PoseStamped curv_pose;


        nav_msgs::PathPtr ref_curvature_ptr (new nav_msgs::Path);
        *ref_curvature_ptr = *msg;

        std::vector<geometry_msgs::PoseStamped>::iterator iter;
        for(iter=ref_curvature_ptr->poses.begin();iter!=ref_curvature_ptr->poses.end();++iter){

            curv_pose.pose.position.x = iter->pose.position.x;
            curv_pose.pose.position.y = iter->pose.position.y;
            curv_pose.pose.position.z = iter->pose.position.z;
            
            ref_curvature_->poses.push_back(curv_pose);
        }

        reference_curvature_get_ = true;
    }

    void VehicleStateCallback(const pharos_msgs::StateStamped2016ConstPtr& msg){
        vehicle_state_ = *msg;
        // VehicleStateMarkerPublish();
    }

    void LookaheadCallback(const std_msgs::Float32ConstPtr& msg){
        lookahaed_distance_ = msg->data;
    }

    void VehicleOdomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        //-- Timing Check
        ros::Time t0 = ros::Time::now();


        //-- Road Information data Receiving check
   
        // Rerference path following or path planner path following
        // if(reference_path_following_){
        //     if(!reference_road_info_get_){
        //         ROS_ERROR("No reference road information");
        //         return;
        //     }
        //     else{
        //     }
        // }
        // else{
        //     if(planner_path_get_ && planner_speed_get_ && planner_curvature_get_){

        //     }
        //     else{
        //         ROS_ERROR("No planner path and speed");
        //         return;
        //     }
        // }

        if(!reference_path_get_ || !reference_speed_get_ || !reference_curvature_get_){
            return;
        }

        if(reference_path_following_){
            printf("Reference Path Following mode\n");
        }
        else{
            printf("Local Path Planner Following mode\n");
        }

        geometry_msgs::PoseStampedPtr vehicle_pose(new geometry_msgs::PoseStamped);
        vehicle_pose->header = msg->header;
        vehicle_pose->pose = msg->pose.pose;

        vehicle_x_ = vehicle_pose->pose.position.x;
        vehicle_y_ = vehicle_pose->pose.position.y;
        vehicle_z_ = vehicle_pose->pose.position.z;

        //-- Current Vehicle Position & Orientation
        tf::Pose pose;
        tf::poseMsgToTF(vehicle_pose->pose, pose);

        double Roll,Pitch, Yaw;
        static double vehicle_heading;
        pose.getBasis().getRPY(Roll, Pitch, Yaw);
        vehicle_heading = Yaw;

        //-- Lookahead Point
        geometry_msgs::PoseStampedPtr vehicle_pose_lookahead(new geometry_msgs::PoseStamped);
        *vehicle_pose_lookahead = *vehicle_pose;
        vehicle_pose_lookahead->pose.position.x = vehicle_pose->pose.position.x + lookahaed_distance_*cos(vehicle_heading);
        vehicle_pose_lookahead->pose.position.y = vehicle_pose->pose.position.y + lookahaed_distance_*sin(vehicle_heading);

        //-- The Nearest Waypoint Searching
        unsigned int current_wp_idx = 0;
        unsigned int lookahead_wp_idx = 0; //forward

        static unsigned int old_lookahead_wp_idx = 0;
        static unsigned int old_wp_indx = 0;

        current_wp_idx = NearestWaypointIdx(ref_waypoint_,vehicle_pose,old_wp_indx);        
        lookahead_wp_idx = NearestWaypointIdx(ref_waypoint_,vehicle_pose_lookahead,old_lookahead_wp_idx);

        old_wp_indx = current_wp_idx;
        old_lookahead_wp_idx = lookahead_wp_idx;

        double ref_path_heading = PathHeading(ref_waypoint_,current_wp_idx);
        double ref_path_heading_lookahead = PathHeading(ref_waypoint_,lookahead_wp_idx);

        SteeringControlParam steer_control_origin;
        SteeringControlParam steer_control_lookahead;

        //-- Heading difference
        steer_control_origin.heading_difference = AngleDifference(ref_path_heading,vehicle_heading);
        steer_control_lookahead.heading_difference = AngleDifference(ref_path_heading_lookahead,vehicle_heading);

        //-- Lateral Offset
        steer_control_origin.lateral_offset = LateralOffset(ref_waypoint_,current_wp_idx,vehicle_pose);
        steer_control_lookahead.lateral_offset = LateralOffset(ref_waypoint_,lookahead_wp_idx,vehicle_pose_lookahead);

        //-- Curvature
        steer_control_origin.curvature = ref_curvature_->poses.at(current_wp_idx).pose.position.z;
        steer_control_lookahead.curvature = ref_curvature_->poses.at(lookahead_wp_idx).pose.position.z;

        //-- Speed
        double reference_speed = ref_speed_->poses.at(lookahead_wp_idx).pose.position.z;


        //---- Msg set and Publish
        motion_cmd_.header = msg->header;
        motion_cmd_.inclination = Pitch;
        motion_cmd_.velocity_limit = reference_speed;
        motion_cmd_.goal_distance = 0.0;
        motion_cmd_.lateral_offset_origin = steer_control_origin.lateral_offset;
        motion_cmd_.vehicle_heading = vehicle_heading;

        motion_cmd_.heading_difference = -steer_control_lookahead.heading_difference;
        motion_cmd_.lateral_offset = steer_control_lookahead.lateral_offset;
        motion_cmd_.curvature = steer_control_lookahead.curvature;
        motion_cmd_.wp_path_heading = ref_path_heading;

        motion_cmd_pub_.publish(motion_cmd_);
    }

    unsigned int NearestWaypointIdx(const nav_msgs::PathConstPtr &path,const geometry_msgs::PoseStampedConstPtr &vehicle_pose, const unsigned int old_idx){
        unsigned int idx = 0;

        tf::Point p1,p2;
        p1.setValue(vehicle_pose->pose.position.x,vehicle_pose->pose.position.y,0.0);
        double min_dist = DBL_MAX;

        unsigned long path_length = path->poses.size();
        unsigned long search_start_idx = 0;
        unsigned long search_finish_idx = 0;

        if(old_idx == 0){
            search_start_idx = 0;
            search_finish_idx = path_length-1;
        }
        else{
            search_start_idx = old_idx;
            search_finish_idx = search_start_idx + NumWpSearch;
        }

        tf::Pose pose;
        tf::poseMsgToTF(vehicle_pose->pose, pose);
        double Roll,Pitch, Yaw;
        static double vehicle_heading;
        pose.getBasis().getRPY(Roll, Pitch, Yaw);
        vehicle_heading = Yaw;

        for(unsigned int i=search_start_idx; i<search_finish_idx;i++){
            p2.setValue(path->poses.at(i%path_length).pose.position.x,path->poses.at(i%path_length).pose.position.y,0.0);
            double dist = p1.distance(p2);

            double heading_err = PathHeading(path,i%path_length)-vehicle_heading;
            if (heading_err > M_PI) {
                heading_err = heading_err - 2 * M_PI;
            }
            else if (heading_err < -M_PI) {
                heading_err = heading_err + 2 * M_PI;
            }
            if (dist < min_dist && fabs(heading_err) < M_PI/2){
            //if(dist < min_dist){
                min_dist = dist;
                idx = (unsigned int)(i%path_length);
            }
        }
        return idx;
    }

    inline double PathHeading(const nav_msgs::PathConstPtr &path, const unsigned int idx){
        double heading;
        unsigned long path_length = path->poses.size();

        double x0 = path->poses.at(idx).pose.position.x;
        double y0 = path->poses.at(idx).pose.position.y;
        double x1 = path->poses.at((idx+1)%path_length).pose.position.x;
        double y1 = path->poses.at((idx+1)%path_length).pose.position.y;

        heading = atan2(y1-y0,x1-x0);

        return heading;
    }

    double AngleDifference(const double path_heading, const double vehicle_heading){
        double angle_diff = path_heading - vehicle_heading;

        if (angle_diff > M_PI) {
            angle_diff -= 2*M_PI;
        }
        else if (angle_diff < -M_PI) {
            angle_diff += 2*M_PI;
        }
        return angle_diff;
    }

    inline double LateralOffset(const nav_msgs::PathConstPtr &path,
                                const unsigned int idx,
                                const geometry_msgs::PoseStampedConstPtr &vehicle_pose){

        double lateral_offset;

        double vehicle_x = vehicle_pose->pose.position.x;
        double vehicle_y = vehicle_pose->pose.position.y;

        unsigned long path_length = path->poses.size();
        double x1,y1,x2,y2;

        x1 = path->poses.at(idx).pose.position.x;
        y1 = path->poses.at(idx).pose.position.y;
        x2 = path->poses.at((idx+1)%path_length).pose.position.x;
        y2 = path->poses.at((idx+1)%path_length).pose.position.y;

        lateral_offset = ((vehicle_x-x1)*(y2-y1)-(vehicle_y-y1)*(x2-x1))/(sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)));
        return lateral_offset;
    }


    void VehicleStateMarkerPublish(void){

        visualization_msgs::MarkerArray VehicleStateMarkerArray;
        visualization_msgs::Marker start_point_marker;

        VehicleStateMarkerArray.markers.resize(3);

        for(int i=0;i<VehicleStateMarkerArray.markers.size();i++){
            VehicleStateMarkerArray.markers.at(i).header.stamp = ros::Time(0);
            VehicleStateMarkerArray.markers.at(i).header.frame_id = "odom";
            VehicleStateMarkerArray.markers.at(i).id = i;
            VehicleStateMarkerArray.markers.at(i).type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            VehicleStateMarkerArray.markers.at(i).action = visualization_msgs::Marker::ADD;
            VehicleStateMarkerArray.markers.at(i).pose.position.x = vehicle_x_;
            VehicleStateMarkerArray.markers.at(i).pose.position.y = vehicle_y_+(double)i+3.0;
            VehicleStateMarkerArray.markers.at(i).pose.position.z = vehicle_z_+(double)i+0.5;
            VehicleStateMarkerArray.markers.at(i).pose.orientation.w = 1.0;
            VehicleStateMarkerArray.markers.at(i).pose.orientation.x = 1.0;
            VehicleStateMarkerArray.markers.at(i).pose.orientation.y = 1.0;
            VehicleStateMarkerArray.markers.at(i).pose.orientation.z = 1.0;
            VehicleStateMarkerArray.markers.at(i).scale.x = 1.0;
            VehicleStateMarkerArray.markers.at(i).scale.y = 1.0;
            VehicleStateMarkerArray.markers.at(i).scale.z = 1.0;
            VehicleStateMarkerArray.markers.at(i).color.a = 1.0;
            VehicleStateMarkerArray.markers.at(i).color.r = 1.0;
            VehicleStateMarkerArray.markers.at(i).color.g = 1.0;
            VehicleStateMarkerArray.markers.at(i).color.b = 1.0;
        }

        std::stringstream str_speed;
        str_speed<<"Speed : "<<vehicle_state_.state.velocity*3.6<<" "<<"("<<motion_cmd_.velocity_limit<<", "<<motion_cmd_.goal_distance<<")";

        std::stringstream str_lat_offset;
        str_lat_offset<<"Lat_offset : "<<motion_cmd_.lateral_offset_origin;
        //str_lat_offset<<"Lat_offset : "<<motion_cmd_.lateral_offset;

        VehicleStateMarkerArray.markers.at(0).text = str_lat_offset.str();
        VehicleStateMarkerArray.markers.at(0).ns = "lateral_offset";
        VehicleStateMarkerArray.markers.at(1).text = str_speed.str();
        VehicleStateMarkerArray.markers.at(1).ns = "speed";

        state_marker_pub_.publish(VehicleStateMarkerArray);
    }


    // Load parameters etc
    int init(){
        publish_rate_ = 100;
        node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        pnode_->param("reference_path_following",reference_path_following_,true);

        motion_cmd_pub_ = node_->advertise<pharos_msgs::MotionCommandStamped3>("/vehicle/motion_command", 10);
        state_marker_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/state_marker", 1);

        vehicle_odom_sub_ = node_->subscribe("/odom/ublox", 5, &TrajectoryObserverNode::VehicleOdomCallback, this);
        vehicle_state_sub_ = node_->subscribe("/vehicle/state2016", 10, &TrajectoryObserverNode::VehicleStateCallback, this);
        lookahead_dist_sub_ = node_->subscribe("/vehicle/lookahead_dist", 10, &TrajectoryObserverNode::LookaheadCallback, this);

        reference_path_sub_ = node_->subscribe("/road_data/waypoint", 10, &TrajectoryObserverNode::ReferecePathCallback, this);
        reference_speed_sub_ = node_->subscribe("/road_data/ref_speed", 10, &TrajectoryObserverNode::RefereceSpeedCallback, this);
        reference_curvature_sub_ = node_->subscribe("/road_data/curvature", 10, &TrajectoryObserverNode::RefereceCurvatureCallback, this);

        // planner_path_sub_ = node_->subscribe("/final_path/path", 10, &TrajectoryObserverNode::PlannerPathCallback, this);
        // planner_speed_sub_ = node_->subscribe("/final_path/speed", 10, &TrajectoryObserverNode::PlannerSpeedCallback, this);
        // planner_curvature_sub_ = node_->subscribe("/final_path/curvature", 10, &TrajectoryObserverNode::PlannerCurvatureCallback, this);


        planner_path_sub_ = node_->subscribe("/path/reference_path", 10, &TrajectoryObserverNode::PlannerPathCallback, this);
        planner_speed_sub_ = node_->subscribe("/path/reference_speed", 10, &TrajectoryObserverNode::PlannerSpeedCallback, this);
        planner_curvature_sub_ = node_->subscribe("/path/reference_curvature", 10, &TrajectoryObserverNode::PlannerCurvatureCallback, this);

        lookahaed_distance_ = 2.7;

        planner_path_get_ = false;
        planner_speed_get_ = false;

        return 0;
    }

    // Publish data
    void publish(){
        ros::Rate loop_rate(publish_rate_);
        while (node_->ok()){
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_observer");

    TrajectoryObserverNode trajectory_observer_node;
    if (trajectory_observer_node.init())
    {
        ROS_FATAL("TrajectoryObserverNode initialization failed");
        return -1;
    }
    trajectory_observer_node.publish();

    return 0;
}

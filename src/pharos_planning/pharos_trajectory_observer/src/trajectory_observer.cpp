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
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include <pharos_msgs/GeographicalPath.h>
#include <pharos_msgs/AccelerationSet4Way_t.h>
#include <pharos_msgs/AntennaOffsetSet_t.h>
#include <pharos_msgs/BrakeSystemStatus_t.h>
#include <pharos_msgs/BSM.h>
#include <pharos_msgs/BSMcoreData.h>
#include <pharos_msgs/BSMpartII.h>
#include <pharos_msgs/ConnectsToList.h>
#include <pharos_msgs/FullPositionVector.h>
#include <pharos_msgs/IntersectionGeometryList.h>
#include <pharos_msgs/IntersectionReferenceID.h>
#include <pharos_msgs/IntersectionStateList_t.h>
#include <pharos_msgs/IntersectionReferenceID_t.h>
#include <pharos_msgs/LaneAttributes_t.h>
#include <pharos_msgs/LaneList_t.h>
#include <pharos_msgs/MAP.h>
#include <pharos_msgs/MovementEventList_t.h>
#include <pharos_msgs/MovementList_t.h>
#include <pharos_msgs/NodeAttributeSetXY.h>
#include <pharos_msgs/NodeListLL_t.h>
#include <pharos_msgs/NodeListXY_t.h>
#include <pharos_msgs/NodeOffsetPointXY_t.h>
#include <pharos_msgs/OffsetSystem_t.h>
#include <pharos_msgs/PathHistory_t.h>
#include <pharos_msgs/Position3D.h>
#include <pharos_msgs/Position3D_t.h>
#include <pharos_msgs/PositionalAccuracy_t.h>
#include <pharos_msgs/RTCM.h>
#include <pharos_msgs/RTCMheader.h>
#include <pharos_msgs/RTCMmessageList_t.h>
#include <pharos_msgs/SPAT.h>
#include <pharos_msgs/SpecialVehicleExtensions_t.h>
#include <pharos_msgs/SupplementalVehicleExtensions_t.h>
#include <pharos_msgs/TIM.h>
#include <pharos_msgs/TimeChangeDetails.h>
#include <pharos_msgs/TravelerDataFrame__content_PR.h>
#include <pharos_msgs/TravelerDataFrameList_t.h>
#include <pharos_msgs/VehicleSafetyExtensions_t.h>
#include <pharos_msgs/VehicleSize_t.h>

#include <pharos_msgs/Workzone_xy.h>
#include <pharos_msgs/Workzone_list.h>
#include <pharos_behavior_planner/LaneChangeState.h>

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

    bool stop_mission_state_;

    double lookahaed_distance_;

    double front_obstacle_dist_;

    ros::Publisher motion_cmd_pub_;
    ros::Publisher state_marker_pub_;
    ros::Publisher sc_zone_pub_;
    ros::Publisher speed_limit_marker_pub_;
    
    ros::Subscriber planner_path_sub_;
    ros::Subscriber planner_speed_sub_;
    ros::Subscriber planner_curvature_sub_;

    ros::Subscriber vehicle_odom_sub_;
    ros::Subscriber vehicle_state_sub_;
    ros::Subscriber lookahead_dist_sub_;
    ros::Subscriber ref_roadinfo_sub_;
    ros::Subscriber current_roadinfo_sub_;
    ros::Subscriber obstacle_dist_sub_;
    ros::Subscriber sczone_sub_;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double origin_z_ = 0.0;


    ros::Subscriber stop_mission_state_sub_;
    ros::Subscriber Emergency_state_sub_;
    ros::Subscriber mission_state_sub_;
    
    pharos_msgs::StateStamped2016 vehicle_state_;
    pharos_msgs::MotionCommandStamped3 motion_cmd_;
    pharos_msgs::Workzone_list sczone_list;

    unsigned int current_wp_idx_;
    bool current_road_info_get_;

    bool planner_path_get_;
    bool planner_speed_get_;
    bool planner_curvature_get_;
    bool reference_road_info_get_;
    bool reference_path_following_;
    bool speed_limit_point_bool = false;
    bool speed_limit_bool = false;
    std_msgs::Bool emergency_state_;
    std_msgs::Int32 mission_state_;
    double heading;
    double radius = 0;

    unsigned int current_road_number = 0;
    geometry_msgs::PoseStamped speed_limit_point;
    geometry_msgs::PoseStampedPtr speed_limit_point2;
    geometry_msgs::Point p1, p2, p3, p4;

    struct SteeringControlParam{
        double heading_difference ;
        double lateral_offset ;
        double curvature ;
    };

    void StopMissionSignalCallback(const std_msgs::BoolConstPtr& msg){
        stop_mission_state_ = msg->data;
        if(stop_mission_state_){
            printf("Force Stopping for mission!!!!!!!!\n");
        }
    }

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
    void EmergencyStateCallback(const std_msgs::BoolConstPtr& msg)
    {
        emergency_state_.data = msg->data;
    }
    void MissionStateCallback(const std_msgs::Int32ConstPtr &msg)
    {
        mission_state_.data = msg->data;
    }

    void PlannerCurvatureCallback(const nav_msgs::PathConstPtr& msg){
        if(reference_path_following_) return;
        *ref_curvature_ = *msg;
        planner_curvature_get_ = true;
    }
    void sczone_callback(const pharos_msgs::Workzone_listConstPtr &msg)
    {
        sczone_list = *msg;
        speed_limit_point_bool = false;

//        ROS_ERROR("sczonelist begin x :: %lf \n" , sczone_list.workzone_xy.at(0).workzone_x);
//        ROS_ERROR("sczonelist begin y :: %lf \n" , sczone_list.workzone_xy.at(0).workzone_y);
//        ROS_ERROR("sczonelist end x :: %lf \n" , sczone_list.workzone_xy.at(1).workzone_x);
//        ROS_ERROR("sczonelist end y :: %lf \n" , sczone_list.workzone_xy.at(1).workzone_y);

        geometry_msgs::PoseStamped speed_limit_end_point;
        speed_limit_end_point.pose.position.x = sczone_list.workzone_xy.at(1).workzone_x-origin_x_;
        speed_limit_end_point.pose.position.y = sczone_list.workzone_xy.at(1).workzone_y-origin_y_;

        speed_limit_point.pose.position.x = (sczone_list.workzone_xy.at(0).workzone_x+sczone_list.workzone_xy.at(1).workzone_x)/2 - origin_x_;
        speed_limit_point.pose.position.y = (sczone_list.workzone_xy.at(0).workzone_y+sczone_list.workzone_xy.at(1).workzone_y)/2 - origin_y_;
        speed_limit_point.pose.position.z = sczone_list.workzone_xy.at(0).ref_speed;

        radius = sqrt(pow(speed_limit_point.pose.position.x-speed_limit_end_point.pose.position.x,2) + pow(speed_limit_point.pose.position.y-speed_limit_end_point.pose.position.y,2));

        if(speed_limit_point.pose.position.x != 0 && speed_limit_point.pose.position.y != 0 && speed_limit_point.pose.position.z != 0 )
        {
            speed_limit_point_bool = true;  //// original :: true
            printf("speedlimit point :: x :: %lf  , y :: %lf \n" , speed_limit_point.pose.position.x , speed_limit_point.pose.position.y);

        } else{
            speed_limit_point_bool = false;
        }



    }

    void ReferenceRoadInfoCallback(const pharos_path_planner::ReferencePathConstPtr &msg){
        if(!reference_path_following_) return;
        if(reference_road_info_get_) return;

        ref_waypoint_->poses.clear();
        ref_curvature_->poses.clear();
        ref_speed_->poses.clear();

        geometry_msgs::PoseStamped wp_pose;
        geometry_msgs::PoseStamped curv_pose;
        geometry_msgs::PoseStamped speed_pose;


        pharos_path_planner::ReferencePathPtr ref_roadinfo (new pharos_path_planner::ReferencePath);
        *ref_roadinfo = *msg;

        std::vector<pharos_path_planner::RoadInfo>::iterator iter;
        for(iter=ref_roadinfo->roadinfo.begin();iter!=ref_roadinfo->roadinfo.end();++iter){


            wp_pose.pose.position.x = iter->position.x;
            wp_pose.pose.position.y = iter->position.y;
            wp_pose.pose.position.z = 0;

            curv_pose.pose.position.x = iter->position.x;
            curv_pose.pose.position.y = iter->position.y;
            curv_pose.pose.position.z = iter->curvature;
            
            speed_pose.pose.position.x = iter->position.x;
            speed_pose.pose.position.y = iter->position.y;
            speed_pose.pose.position.z = iter->reference_speed;

            ref_waypoint_->poses.push_back(wp_pose);
            ref_curvature_->poses.push_back(curv_pose);
            ref_speed_->poses.push_back(speed_pose);
        }


        reference_road_info_get_ = true;
    }

    void CurrentRoadInfoCallback(const pharos_path_planner::RoadInfoConstPtr &msg){
        current_wp_idx_ = msg->wp_index;
        current_road_number = msg->road_number;
        current_road_info_get_ = true;
    }

    void VehicleStateCallback(const pharos_msgs::StateStamped2016ConstPtr& msg){
        vehicle_state_ = *msg;
        VehicleStateMarkerPublish();
    }

    void LookaheadCallback(const std_msgs::Float32ConstPtr& msg){
        lookahaed_distance_ = msg->data;
    }

    void ObstacleDistCallback(const std_msgs::Float32ConstPtr& msg){
        front_obstacle_dist_ = msg->data;
    }

    void VehicleOdomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        //-- Timing Check
        ros::Time t0 = ros::Time::now();
//        ROS_ERROR(" vehicle_odom_callback start \n\n ");

        //-- Road Information data Receiving check
   
        if(!current_road_info_get_) return;

        nav_msgs::PathPtr reference_path (new nav_msgs::Path);

        // Rerference path following or path planner path following
        if(reference_path_following_){
            if(!reference_road_info_get_){
                ROS_ERROR("No reference road information");
                return;
            }
            else{
            }
        }
        else{
            if(planner_path_get_ && planner_speed_get_ && planner_curvature_get_){

            }
            else{
                
                ROS_ERROR("No planner path and speed");
                return;
            }
        }
        if(reference_path_following_){
            printf("Reference Path Following mode\n");
        }
        else{
//            printf("Local Path Planner Following mode\n");
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
        geometry_msgs::PoseStampedPtr vehicle_pose_lookahead_speed(new geometry_msgs::PoseStamped);

        *vehicle_pose_lookahead = *vehicle_pose;
        *vehicle_pose_lookahead_speed = *vehicle_pose;

        vehicle_pose_lookahead->pose.position.x = vehicle_pose->pose.position.x + lookahaed_distance_*cos(vehicle_heading);
        vehicle_pose_lookahead->pose.position.y = vehicle_pose->pose.position.y + lookahaed_distance_*sin(vehicle_heading);

        double lookahaed_distance_speed_ = lookahaed_distance_*1.65;
        vehicle_pose_lookahead_speed->pose.position.x = vehicle_pose->pose.position.x + lookahaed_distance_speed_*cos(vehicle_heading);
        vehicle_pose_lookahead_speed->pose.position.y = vehicle_pose->pose.position.y + lookahaed_distance_speed_*sin(vehicle_heading);


        //-- The Nearest Waypoint Searching
        unsigned int current_wp_idx = 0;
        unsigned int lookahead_wp_idx = 0; //forward
        unsigned int lookahead_speed_wp_idx = 0;

        static unsigned int old_lookahead_wp_idx = 0;
        static unsigned int old_lookahead_speed_wp_idx = 0;
        static unsigned int old_wp_indx = 0;

        if(ref_waypoint_->poses.empty() != 1)
        {
//            ROS_ERROR("gggg\n");
            current_wp_idx = NearestWaypointIdx(ref_waypoint_,vehicle_pose,old_wp_indx);     // this is matter
            // current_wp_idx = current_wp_idx_;
            lookahead_wp_idx = NearestWaypointIdx(ref_waypoint_,vehicle_pose_lookahead,old_lookahead_wp_idx);
            lookahead_speed_wp_idx = NearestWaypointIdx(ref_waypoint_,vehicle_pose_lookahead_speed,old_lookahead_speed_wp_idx);
            // printf("[wp_idx], from GP: %d, from TO: %d\n",current_wp_idx_, current_wp_idx);
        }else return;

        old_wp_indx = current_wp_idx;
        old_lookahead_wp_idx = lookahead_wp_idx;
        old_lookahead_speed_wp_idx = lookahead_speed_wp_idx;


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
        double reference_speed = ref_speed_->poses.at(lookahead_speed_wp_idx).pose.position.z;

//        if(mission_state_.data == 5 && mission_state_.data == 6)
//        {
//            reference_speed = 30;
//        }



        if(speed_limit_point_bool == true)
        {
//            ROS_ERROR("vehicle_x : %lf \n" , vehicle_x_);
//            ROS_ERROR("vehicle_y : %lf \n" , vehicle_y_);
//            ROS_ERROR("speedlimit x :: %lf \n" , speed_limit_point.pose.position.x);
//            ROS_ERROR("speedlimit y :: %lf \n" , speed_limit_point.pose.position.y);
            printf("radius :: %lf \n" , radius);
            double dist = sqrt(pow(speed_limit_point.pose.position.x-vehicle_x_,2) + pow(speed_limit_point.pose.position.y-vehicle_y_,2));
            printf("dist :: %lf \n" , dist);
            speed_limit_bool = false;
            if(radius > dist)
            {
                printf("we are in speed_limit_area !! \n\n\n");
                speed_limit_bool = true;
            }else{
                printf("we are not in speed_ limit area !!! \n\n\n");
                speed_limit_bool = false;
            }

        }




        if(speed_limit_bool == true && current_road_number > 46 )
        {
//            ROS_ERROR("speed_limit_bool : %d " , speed_limit_bool);
//            ROS_ERROR("current_road_number : %d " , current_road_number);
//            ROS_ERROR("reference_speed :: %lf \n" , reference_speed );
//            ROS_ERROR("speed limit :: %lf \n" , speed_limit_point.pose.position.z);
            if(reference_speed > speed_limit_point.pose.position.z)
            {
//                ROS_ERROR("reference_speed :: %lf \n" , reference_speed );
//                ROS_ERROR("speed limit :: %lf \n" , speed_limit_point.pose.position.z);
//                ROS_ERROR("reference speed is over than speed limit !! \n");
                reference_speed = speed_limit_point.pose.position.z-2;
            }
        }




        // if(front_obstacle_dist_ != 1000){
        //     speed_limit = -vehicle_state_.state.velocity/front_obstacle_dist_*0.1+vehicle_state_.state.velocity;
        //     if(speed_limit<0){
        //         speed_limit=0;
        //     }
        //     reference_speed = speed_limit*3.6;
        // }

        if(emergency_state_.data == true)
        {
            motion_cmd_.sudden_stop = 1;
        }else{
            motion_cmd_.sudden_stop = 0;
        }

        if(mission_state_.data == 9)
        {
            motion_cmd_.sudden_stop = 1;
        }

        //---- Msg set and Publish
        motion_cmd_.header = msg->header;
        motion_cmd_.inclination = Pitch;
        motion_cmd_.velocity_limit = reference_speed;
//        if(motion_cmd_.sudden_stop == 1)
//        {
//            motion_cmd_.velocity_limit = 0.0;
//        }
        motion_cmd_.goal_distance = 0.0;
        motion_cmd_.lateral_offset_origin = steer_control_origin.lateral_offset;
        motion_cmd_.vehicle_heading = vehicle_heading;

        motion_cmd_.heading_difference = -steer_control_lookahead.heading_difference;
        motion_cmd_.lateral_offset = steer_control_lookahead.lateral_offset;
        motion_cmd_.curvature = steer_control_lookahead.curvature;
        motion_cmd_.wp_path_heading = ref_path_heading;

        std_msgs::Float64 mission_state;
        mission_state.data = mission_state_.data;
        motion_cmd_.mission_state = current_road_number;


//        printf("motion_cmd sudden stop %lf \n",motion_cmd_.sudden_stop );

        // if(current_wp_idx > ref_waypoint_->poses.size()-5){
        //     motion_cmd_.velocity_limit = 0.0;
        //     printf("Stop!!, waypoint finish\n");
        // }

//        if(stop_mission_state_){
//            motion_cmd_.velocity_limit = 0.0;
//            printf("Stop!!, mission!!!!!\n");
//        }

        motion_cmd_pub_.publish(motion_cmd_);
//        ROS_ERROR(" vehicle_odom_callback end \n\n ");

    }

    unsigned int NearestWaypointIdx(const nav_msgs::PathConstPtr &path,const geometry_msgs::PoseStampedConstPtr &vehicle_pose, const unsigned int old_idx){
        unsigned int idx = 0;
//        ROS_ERROR(" NearestYwpointIdx start \n\n ");

        tf::Point p1,p2;
        p1.setValue(vehicle_pose->pose.position.x,vehicle_pose->pose.position.y,0.0);
        double min_dist = DBL_MAX;

        unsigned long path_length = path->poses.size();
        unsigned long search_start_idx = 0;
        unsigned long search_finish_idx = 0;


        if(path_length<2)
        {
            return 1;
        }

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

//        ROS_ERROR("path length = %lu \n" , path_length);
        if(path_length == 0) return 1;


        for(unsigned int i=search_start_idx; i<search_finish_idx;i++) {

            p2.setValue(path->poses.at(i%path_length).pose.position.x,path->poses.at(i%path_length).pose.position.y,0.0);

            double dist = p1.distance(p2);

            double heading_err = PathHeading(path,i%path_length)-vehicle_heading;
//            ROS_ERROR("//////////////// check this //////////////");
//            ROS_ERROR("path heading :: %lf " , PathHeading(path,i%path_length));
//            ROS_ERROR("vehicle_heading :: %lf ", vehicle_heading);
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
//        ROS_ERROR(" NearestYwpointIdx end \n\n ");

        return idx;
    }

    inline double PathHeading(const nav_msgs::PathConstPtr &path, const unsigned int idx){

        unsigned long path_length = path->poses.size();

        double x0;
        double y0;
        double x1;
        double y1;

        if(path_length > 2)
        {
            x0 = path->poses.at(idx).pose.position.x;
            y0 = path->poses.at(idx).pose.position.y;
            x1 = path->poses.at((idx+1)%path_length).pose.position.x;
            y1 = path->poses.at((idx+1)%path_length).pose.position.y;
            heading = atan2(y1-y0,x1-x0);
            return heading;
        }else
        {
            return heading;
        }
//        ROS_ERROR(" path heading end \n\n ");
//
//        return heading;
    }

    double AngleDifference(const double path_heading, const double vehicle_heading){
//        ROS_ERROR(" angle_diff start \n\n ");

        double angle_diff = path_heading - vehicle_heading;

        if (angle_diff > M_PI) {
            angle_diff -= 2*M_PI;
        }
        else if (angle_diff < -M_PI) {
            angle_diff += 2*M_PI;
        }
//        ROS_ERROR(" angle_diff end \n\n ");

        return angle_diff;
    }

    inline double LateralOffset(const nav_msgs::PathConstPtr &path,
                                const unsigned int idx,
                                const geometry_msgs::PoseStampedConstPtr &vehicle_pose){
//        ROS_ERROR(" lateralOffset start \n\n ");
//
//        ROS_ERROR("trajectory path size :: %lu \n\n "  ,  path->poses.size() );

        if(path->poses.size() < 3)
        {
            return 0;
        }
        double lateral_offset = 0;

        double vehicle_x = vehicle_pose->pose.position.x;
        double vehicle_y = vehicle_pose->pose.position.y;

        unsigned long path_length = path->poses.size();
        double x1,y1,x2,y2;

        x1 = path->poses.at(idx).pose.position.x;
        y1 = path->poses.at(idx).pose.position.y;
        x2 = path->poses.at((idx+1)%path_length).pose.position.x;
        y2 = path->poses.at((idx+1)%path_length).pose.position.y;

        lateral_offset = ((vehicle_x-x1)*(y2-y1)-(vehicle_y-y1)*(x2-x1))/(sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)));
//        ROS_ERROR(" lateralOffset end \n\n ");

        return lateral_offset;

    }


    void VehicleStateMarkerPublish(void){

//        ROS_ERROR(" vehiclestatemarkerpublish start \n\n ");

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
//        ROS_ERROR(" vehiclestatemarkerpublish end \n\n ");

    }


    // Load parameters etc
    int init(){
        publish_rate_ = 100;
        node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        ros::param::get("gps/origin/x",origin_x_);
        ros::param::get("gps/origin/y",origin_y_);
        ros::param::get("gps/origin/z",origin_z_);
        pnode_->param("reference_path_following",reference_path_following_,true);

        motion_cmd_pub_ = node_->advertise<pharos_msgs::MotionCommandStamped3>("/vehicle/motion_command", 100);
        state_marker_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/state_marker", 1);
        speed_limit_marker_pub_ = node_->advertise<visualization_msgs::Marker>("/speed_limit_marker",10);
        sc_zone_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/sc_zone_visualization", 1);

        vehicle_odom_sub_ = node_->subscribe("/odom/vehicle", 5, &TrajectoryObserverNode::VehicleOdomCallback, this);
        // vehicle_odom_sub_ = node_->subscribe("/odom/ublox", 5, &TrajectoryObserverNode::VehicleOdomCallback, this);
        vehicle_state_sub_ = node_->subscribe("/vehicle/state2016", 10, &TrajectoryObserverNode::VehicleStateCallback, this);
        lookahead_dist_sub_ = node_->subscribe("/vehicle/lookahead_dist", 10, &TrajectoryObserverNode::LookaheadCallback, this);
        ref_roadinfo_sub_ = node_->subscribe("/ref_road_info_with_speed_profile", 3, &TrajectoryObserverNode::ReferenceRoadInfoCallback, this);
        current_roadinfo_sub_ = node_->subscribe("/current_road_info", 10, &TrajectoryObserverNode::CurrentRoadInfoCallback, this);

        planner_path_sub_ = node_->subscribe("/final_path/path", 10, &TrajectoryObserverNode::PlannerPathCallback, this);
        planner_speed_sub_ = node_->subscribe("/final_path/speed", 10, &TrajectoryObserverNode::PlannerSpeedCallback, this);
        planner_curvature_sub_ = node_->subscribe("/final_path/curvature", 10, &TrajectoryObserverNode::PlannerCurvatureCallback, this);

        obstacle_dist_sub_ = node_->subscribe("/NEAR_OBJECT_DISTANCE",10, &TrajectoryObserverNode::ObstacleDistCallback, this);
        stop_mission_state_sub_ = node_->subscribe("/behavior/StopSignal",10, &TrajectoryObserverNode::StopMissionSignalCallback, this);
        Emergency_state_sub_ = node_->subscribe("emergency_state",10,&TrajectoryObserverNode::EmergencyStateCallback,this);
        mission_state_sub_ = node_->subscribe("mission_state",10,&TrajectoryObserverNode::MissionStateCallback,this);
        sczone_sub_ = node_->subscribe("/wave/sc_list",10,&TrajectoryObserverNode::sczone_callback,this);




        lookahaed_distance_ = 2.7;

        planner_path_get_ = false;
        planner_speed_get_ = false;
        reference_road_info_get_ = false;
        current_road_info_get_ = false;
        stop_mission_state_ = false;

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

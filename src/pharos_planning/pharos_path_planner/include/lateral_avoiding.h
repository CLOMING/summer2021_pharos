#ifndef _LATERAL_AVOIDING_
#define _LATERAL_AVOIDING_

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <tf/transform_datatypes.h>

//#include <local_path_planner.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <pharos_path_planner/RoadInfo.h>
#include <pharos_path_planner/ReferencePath.h>
#include "local_path_planner.h"

#define VehicleWheelBase 2.7f
#define MaxWheelAngle 30.0f

#define G 9.81
#define acc_G  0.1 * G
#define Init_dcc_G  0.05 * G
#define MAX_dcc_G 0.4*G  // original gain was 0.45
#define Estop 50

#define NumWpSearch 30

//extern PathPlanning3D PP3D;

class Lateral_Avoiding{
public:

    struct Candidate_Path{
        nav_msgs::Path path;
        nav_msgs::Path path_speed;
        nav_msgs::Path path_curvature;
        double path_offset;
        double velocity;
        double cost;
        double distance;
        bool collision;
    };

    struct Reference_Path{
        nav_msgs::Path waypoint;
        nav_msgs::Path curvature;
        nav_msgs::Path speed;
    };

    int Num_Candidate_Paths_;
    double Planning_Total_Time_;
    double Planning_Dt_;
    double Path_Lateral_offset_;
    double lookahaed_distance_;

    unsigned int planning_steps_;
//    pharos_behavior_planner::DrivingState DrivingState_;

    bool collision_expectation_;
    double dcc_G_;

    ros::NodeHandlePtr node_;
    ros::NodeHandlePtr pnode_;

    nav_msgs::PathPtr final_path_;
    nav_msgs::PathPtr final_path_speed_;
    nav_msgs::PathPtr final_path_curvature_;

    pharos_path_planner::ReferencePathPtr reference_road_;

    ros::Publisher candidate_path_line_marker_pub_;
    ros::Publisher candidate_path_point_marker_pub_;

    ros::Publisher lane_change_path_line_marker_pub_;
    ros::Publisher lane_change_path_point_marker_pub_;

    ros::Publisher force_avoid_path_line_marker_pub_;
    ros::Publisher force_avoid_path_point_marker_pub_;

    Reference_Path* reference_path_;

    std::string path_point_marker_name_;
    std::string path_line_marker_name_;

    visualization_msgs::MarkerArrayPtr path_line_array_;
    visualization_msgs::MarkerArrayPtr path_points_array_;

    double stop_count;

    int init(){

        collision_expectation_ = false;
        dcc_G_ = Init_dcc_G;
        
        reference_path_ = new Reference_Path;

        path_line_array_ = visualization_msgs::MarkerArrayPtr(new visualization_msgs::MarkerArray);
        path_points_array_ = visualization_msgs::MarkerArrayPtr(new visualization_msgs::MarkerArray);


        path_line_marker_name_ = std::string("/path/candidate_paths/line");
        path_point_marker_name_ = std::string("/path/candidate_paths/point");

        final_path_ = nav_msgs::PathPtr(new nav_msgs::Path);
        final_path_speed_ = nav_msgs::PathPtr(new nav_msgs::Path);
        final_path_curvature_ = nav_msgs::PathPtr(new nav_msgs::Path);

        reference_road_ = pharos_path_planner::ReferencePathPtr(new pharos_path_planner::ReferencePath);

        node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        pnode_->param("planning_time", Planning_Total_Time_, 3.0);
        pnode_->param("planning_dt", Planning_Dt_, 0.2);
        pnode_->param("num_of_candidate_paths", Num_Candidate_Paths_, 5);
        pnode_->param("offset_between_paths", Path_Lateral_offset_, 1.0);  

        printf("planning_time: %f\n",Planning_Total_Time_);
        printf("planning_dt: %f\n",Planning_Dt_);
        printf("num_of_candidate_paths: %d\n",Num_Candidate_Paths_);
        printf("offset_between_paths: %f\n",Path_Lateral_offset_);


        printf("Lateral_Avoiding init\n");      

        candidate_path_line_marker_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/path/candidate_paths/line", 10);
        candidate_path_point_marker_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/path/candidate_paths/point", 10);

        lane_change_path_line_marker_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/path/lane_change_path/line", 10);
        lane_change_path_point_marker_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/path/lane_change_path/point", 10);

        force_avoid_path_line_marker_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/path/force_avoiding/line", 10);
        force_avoid_path_point_marker_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/path/force_avoiding/point", 10);

        planning_steps_ = (int)(Planning_Total_Time_/Planning_Dt_)+1;

        return 0;
    };


    int LateralAvoidingPlanningSigmoid(const Reference_Path *reference_path,
                            const geometry_msgs::PoseStamped init_vehicle_pose,
                             const nav_msgs::OccupancyGridConstPtr& obstacle_map,
                             const tf::Point EgoVehiclePose,
                             const unsigned int init_current_wp_idx,
                             const double init_vehicle_speed,
                             unsigned int& path_id,
                             nav_msgs::PathPtr &final_path,
                             nav_msgs::OccupancyGridPtr &collision_check_map);

    int LateralAvoidingPlanning(const Reference_Path *reference_path,
                            const geometry_msgs::PoseStamped init_vehicle_pose,
                             const nav_msgs::OccupancyGridConstPtr& obstacle_map,
                             const tf::Point EgoVehiclePose,
                             const unsigned int init_current_wp_idx,
                             const double init_vehicle_speed,
                             const int planning_mode,
                             unsigned int& path_id,
                             nav_msgs::PathPtr &final_path,
                             nav_msgs::OccupancyGridPtr &collision_check_map,
                             const int Mission_State);

    unsigned int PathChoice(const std::vector<Candidate_Path> &candidate_paths,const unsigned int old_path_id){
        
        unsigned int final_path_id = 0;
        unsigned int temp_path_id = 0;
        static unsigned int old_temp_path_id = 0;

        static int judge_counting = 0;
        int judge_threshold = 10;

        // If old path has a collision then find new path
        if(candidate_paths.at(old_path_id).collision){
            judge_counting = 0;
            double min_cost = DBL_MAX;
            for(unsigned int i=0;i<candidate_paths.size();i++){
                if(candidate_paths.at(i).collision){
                    continue;
                }
                if(candidate_paths.at(i).cost < min_cost){
                    min_cost = candidate_paths.at(i).cost;
                    final_path_id = i;
                }
            }
        }

        // If old path doesn't has a collision then...
        else{
            double min_cost = DBL_MAX;

            //Find the min cost path id
            for(unsigned int i=0;i<candidate_paths.size();i++){
                if(candidate_paths.at(i).collision){
                    continue;
                }
                if(candidate_paths.at(i).cost < min_cost){
                    min_cost = candidate_paths.at(i).cost;
                    temp_path_id = i;
                }
            }

            if(temp_path_id == old_path_id){
                final_path_id = temp_path_id;
                judge_counting = 0;
            }
            else{
                if(judge_counting > judge_threshold){
                    final_path_id = temp_path_id;
                    judge_counting = 0;
                }
                else{
                    final_path_id = old_path_id;
                    if(temp_path_id == old_temp_path_id){
                        judge_counting++;
                    }
                    else{
                        old_temp_path_id = temp_path_id;
                        judge_counting = 0;
                    }
                }
            }
        }
        return final_path_id;
    };

    void VehiclePoseUpdate(const geometry_msgs::PoseStampedConstPtr &vehicle_pose,
                                           geometry_msgs::PoseStampedPtr &new_vehicle_pose,
                                           double& vehicle_speed,
                                           double& curvature,
                                           const double path_offset,
                                           const double dt,
                                           unsigned int& wp_idx,
                                           const int planning_mode){
        tf::Pose pose;
        tf::poseMsgToTF(vehicle_pose->pose, pose);
        double Roll,Pitch, Yaw;
        pose.getBasis().getRPY(Roll, Pitch, Yaw);
        double vehicle_heading = Yaw;

        geometry_msgs::PoseStampedPtr vehicle_pose_lookahead(new geometry_msgs::PoseStamped);
        *vehicle_pose_lookahead = *vehicle_pose;
        vehicle_pose_lookahead->pose.position.x = vehicle_pose->pose.position.x + lookahaed_distance_*cos(vehicle_heading);
        vehicle_pose_lookahead->pose.position.y = vehicle_pose->pose.position.y + lookahaed_distance_*sin(vehicle_heading);

        //Path Following


        //find the nearest waypoint
        unsigned int nearest_wp_idx = 0;
        unsigned int nearest_wp_idx_origin = 0;
        // nearest_wp_idx = NearestWaypointIdx(vehicle_pose_lookahead, wp_idx);
        nearest_wp_idx = NearestWaypointIdx(vehicle_pose_lookahead);
        nearest_wp_idx_origin = NearestWaypointIdx(vehicle_pose);

        // nearest_wp_idx = NearestWaypointIdx2(wp_idx,vehicle_pose_lookahead);
        // nearest_wp_idx_origin = NearestWaypointIdx2(wp_idx,vehicle_pose);
        wp_idx = nearest_wp_idx;
        //virtual steering

        //angle_difference
        double angle_difference = 0.0;
        angle_difference = AngleDifference(wp_idx,vehicle_pose_lookahead);

        //lateral_offset
        double lateral_offset = 0.0;
        lateral_offset = LateralOffset(wp_idx,vehicle_pose_lookahead) + path_offset;

        //curvature
        curvature = reference_path_->curvature.poses.at(nearest_wp_idx_origin).pose.position.z;
        double curvature_lookahead = reference_path_->curvature.poses.at(wp_idx).pose.position.z;
        //printf("curvature: %.5f\n",curvature);
        double wheel_angle = SteeringControl(angle_difference,lateral_offset,curvature,vehicle_speed, planning_mode);


        double current_vehicle_speed = vehicle_speed; //[m/s]
        double vehicle_speed_origin = reference_path_->speed.poses.at(wp_idx).pose.position.z;
        double desired_vehicle_speed = reference_path_->speed.poses.at(nearest_wp_idx_origin).pose.position.z;

//        ROS_ERROR("///////////////////////////////");
//        ROS_ERROR("vehicle_speed_origin : %lf " , vehicle_speed_origin);
//        ROS_ERROR("desired_vehicle_speed : %lf " , desired_vehicle_speed);

        if(planning_mode == 2){ //lane changing
            desired_vehicle_speed *= 1.0;
        }
        else if(planning_mode == 3){ //force avoiding
            desired_vehicle_speed *= 0.3;
        }

        if(collision_expectation_){
            // if(dcc_G_ > MAX_dcc_G){

            // }
            // else{

            // }
            vehicle_speed -= dcc_G_*dt;
            
        }
        else{
            if(desired_vehicle_speed > current_vehicle_speed){
                vehicle_speed += acc_G*dt;
                if(vehicle_speed > desired_vehicle_speed){
                    vehicle_speed = desired_vehicle_speed;
                }
            }
            else{
                vehicle_speed = desired_vehicle_speed;

            }
        }
        if(vehicle_speed < 0){
            vehicle_speed = 0.0;
        }

        //Motion Update
        BicycleModelMotionUpdate(vehicle_pose,new_vehicle_pose,wheel_angle,vehicle_speed,dt);
    };


    double SteeringControl(const double angle_diff, double lat_offset, const double curvature, const double velocity, const int planning_mode){
        double wheel_angle;

        double kp_angle;
        double kp_lat;
        double kp_curv;
        double kd_angle;
        double kd_lat;

        double angle_control;
        double lat_control;
        double curv_control;

        kp_angle = 0.8;
        kp_lat = 2.4;
        kp_curv = 0.3;
        double kp_lat_min = 0.5;

        double max_lat_offset = 0.75;
        double velocity_effect_scale = 1.0;

        if(planning_mode == 2){ // Lane Changing
            velocity_effect_scale = 4.0; // smooth path for lane chaning
            if(fabs(lat_offset) > max_lat_offset){
                if(lat_offset>0){
                    lat_offset = max_lat_offset;
                }
                else{
                    lat_offset = -max_lat_offset;
                }
            }
        }
        else if(planning_mode == 3){ // Force Avoiding
            velocity_effect_scale = 1.5; // sharp path for avoiding //// original gain was 2.5
         }
        else{
            velocity_effect_scale = 3.5;
        }

        angle_control = kp_angle * angle_diff;
        lat_control = atan2(kp_lat * lat_offset, velocity_effect_scale*velocity);
        curv_control = kp_curv * curvature;

        //printf("ad: %.3f, lo: %.3f, k: %.3f\n",angle_control,lat_control,curvature);
        wheel_angle = angle_control + lat_control + curv_control;

        if(wheel_angle > (MaxWheelAngle*M_PI/180)){
            wheel_angle = (MaxWheelAngle*M_PI/180);
        }
        else if(wheel_angle < (-MaxWheelAngle*M_PI/180)){
            wheel_angle = (-MaxWheelAngle*M_PI/180);
        }
        return wheel_angle;
    };

    unsigned int NearestWaypointIdx(const geometry_msgs::PoseStampedConstPtr &vehicle_pose){
        unsigned int idx = 0;
        tf::Point p1,p2;
        p1.setValue(vehicle_pose->pose.position.x,vehicle_pose->pose.position.y,0.0);
        double min_dist = DBL_MAX;

        std::vector<geometry_msgs::PoseStamped>::iterator iter;
        unsigned int i=0;
//        ROS_ERROR("reference_path in nearest_waypointidx path size :: %lu \n\n " , reference_path_->waypoint.poses.size());
        for(iter=reference_path_->waypoint.poses.begin();iter!=reference_path_->waypoint.poses.end()-2;++iter){
//            ROS_ERROR("inside of nearest for loop \n");
            p2.setValue(iter->pose.position.x,iter->pose.position.y,0.0);
            double dist = p1.distance(p2);
            if(dist < min_dist){
                min_dist = dist;
                idx = i;
            }
            i++;
        }
        return idx;
    };

//    //// NearestWaypointIdx2 for find Astar idx
//    unsigned int NearestWaypointIdx2(const geometry_msgs::PoseStampedConstPtr &vehicle_pose, const unsigned int old_wp_idx,const int search_num, const nav_msgs::PathPtr &a_star_path ){
//        unsigned int idx = 0;
//
//        tf::Point p1,p2;
//        p1.setValue(vehicle_pose->pose.position.x,vehicle_pose->pose.position.y,0.0);
//        double min_dist = DBL_MAX;
//
//        unsigned long path_length = a_star_path->poses.size();
//
//        unsigned int search_start_idx = old_wp_idx;
//        if(search_start_idx < 0){
//            search_start_idx += path_length;
//        }
//        unsigned int search_finish_idx = search_start_idx + search_num;
//
//        for(unsigned int i=search_start_idx; i<search_finish_idx;i++){
//            p2.setValue(a_star_path->poses.at(i%path_length).pose.position.x,a_star_path->poses.at(i%path_length).pose.position.y,0.0);
//            double dist = p1.distance(p2);
//            if(dist < min_dist){
//                min_dist = dist;
//                idx = (unsigned int)(i%path_length);
//            }
//        }
////        ROS_ERROR("Nearestwaypointidx2 distance :: %lf\n", min_dist);
//        return idx;
//    };

        // std::vector<geometry_msgs::PoseStamped>::iterator iter;
        // unsigned int i=0;
        // for(iter=reference_path_->waypoint.poses.begin();iter!=reference_path_->waypoint.poses.end();++iter){
        //     p2.setValue(iter->pose.position.x,iter->pose.position.y,0.0);
        //     double dist = p1.distance(p2);
        //     if(dist < min_dist){
        //         min_dist = dist;
        //         idx = i;
        //     }
        //     i++;
        // }


//    unsigned int NearestWaypointIdx2(const geometry_msgs::PoseStampedConstPtr &vehicle_pose, const unsigned int old_wp_idx,const int search_num){
//        unsigned int idx = 0;
//
//        tf::Point p1,p2;
//        p1.setValue(vehicle_pose->pose.position.x,vehicle_pose->pose.position.y,0.0);
//        double min_dist = DBL_MAX;
//
//
//
//        unsigned long path_length = reference_path_->waypoint.poses.size();
//
//        unsigned int search_start_idx = old_wp_idx;
//        if(search_start_idx < 0){
//            search_start_idx += path_length;
//        }
//        unsigned int search_finish_idx = search_start_idx + search_num;
//
//        for(unsigned int i=search_start_idx; i<search_finish_idx;i++){
//            p2.setValue(reference_path_->waypoint.poses.at(i%path_length).pose.position.x,reference_path_->waypoint.poses.at(i%path_length).pose.position.y,0.0);
//            double dist = p1.distance(p2);
//            if(dist < min_dist){
//                min_dist = dist;
//                idx = (unsigned int)(i%path_length);
//            }
//        }
//        return idx;
//    }

     double AngleDifference(const unsigned int wp_idx, const geometry_msgs::PoseStampedConstPtr &vehicle_pose){

        double angle_diff = 0.0;

        double r_path_heading;
        double vehicle_heading;

        unsigned long path_length = reference_path_->waypoint.poses.size();

        double x0 = reference_path_->waypoint.poses.at(wp_idx).pose.position.x;
        double y0 = reference_path_->waypoint.poses.at(wp_idx).pose.position.y;
        double x1 = reference_path_->waypoint.poses.at((wp_idx+1)%path_length).pose.position.x;
        double y1 = reference_path_->waypoint.poses.at((wp_idx+1)%path_length).pose.position.y;

        r_path_heading = atan2(y1-y0,x1-x0);

        tf::Pose pose;
        tf::poseMsgToTF(vehicle_pose->pose, pose);
        double Roll,Pitch, Yaw;
        pose.getBasis().getRPY(Roll, Pitch, Yaw);
        vehicle_heading = Yaw;

        angle_diff = r_path_heading - vehicle_heading;

        if (angle_diff > M_PI) {
            angle_diff = angle_diff - 2 * M_PI;
        }
        else if (angle_diff < -M_PI) {
            angle_diff = angle_diff + 2 * M_PI;
        }
        return angle_diff;
    }

//    double AngleDifference2(const unsigned int wp_idx, const geometry_msgs::PoseStampedConstPtr &vehicle_pose, const nav_msgs::PathPtr &a_star_path){
//
//        double angle_diff = 0.0;
//
//        double r_path_heading;
//        double vehicle_heading;
//
//        unsigned long path_length = a_star_path->poses.size();
//
//        double x0 = a_star_path->poses.at(wp_idx).pose.position.x;
//        double y0 = a_star_path->poses.at(wp_idx).pose.position.y;
//        double x1 = a_star_path->poses.at((wp_idx+1)%path_length).pose.position.x;
//        double y1 = a_star_path->poses.at((wp_idx+1)%path_length).pose.position.y;
//
//        r_path_heading = atan2(y1-y0,x1-x0);
//
//        tf::Pose pose;
//        tf::poseMsgToTF(vehicle_pose->pose, pose);
//        double Roll,Pitch, Yaw;
//        pose.getBasis().getRPY(Roll, Pitch, Yaw);
//        vehicle_heading = Yaw;
//
//        angle_diff = r_path_heading - vehicle_heading;
//
//        if (angle_diff > M_PI) {
//            angle_diff = angle_diff - 2 * M_PI;
//        }
//        else if (angle_diff < -M_PI) {
//            angle_diff = angle_diff + 2 * M_PI;
//        }
//        return angle_diff;
//    }

    inline double LateralOffset(const unsigned int wp_idx, const geometry_msgs::PoseStampedConstPtr &vehicle_pose){
        double lateral_offset;

        double vehicle_x = vehicle_pose->pose.position.x;
        double vehicle_y = vehicle_pose->pose.position.y;

        unsigned long path_length = reference_path_->waypoint.poses.size();

        double x1,y1,x2,y2;

        x1 = reference_path_->waypoint.poses.at(wp_idx).pose.position.x;
        y1 = reference_path_->waypoint.poses.at(wp_idx).pose.position.y;
        x2 = reference_path_->waypoint.poses.at((wp_idx+1)%path_length).pose.position.x;
        y2 = reference_path_->waypoint.poses.at((wp_idx+1)%path_length).pose.position.y;

        lateral_offset = ((vehicle_x-x1)*(y2-y1)-(vehicle_y-y1)*(x2-x1))/(sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)));

        return lateral_offset;
    };

//    inline double LateralOffset2(const unsigned int wp_idx, const geometry_msgs::PoseStampedConstPtr &vehicle_pose, const nav_msgs::PathPtr &a_star_path){
//        double lateral_offset;
//
//        double vehicle_x = vehicle_pose->pose.position.x;
//        double vehicle_y = vehicle_pose->pose.position.y;
//
//        unsigned long path_length = a_star_path->poses.size();
//
//        double x1,y1,x2,y2;
//
//        x1 = a_star_path->poses.at(wp_idx).pose.position.x;
//        y1 = a_star_path->poses.at(wp_idx).pose.position.y;
//        x2 = a_star_path->poses.at((wp_idx+1)%path_length).pose.position.x;
//        y2 = a_star_path->poses.at((wp_idx+1)%path_length).pose.position.y;
//
//        lateral_offset = ((vehicle_x-x1)*(y2-y1)-(vehicle_y-y1)*(x2-x1))/(sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)));
//
//        return lateral_offset;
//    };

    void BicycleModelMotionUpdate(const geometry_msgs::PoseStampedConstPtr &in_pose,
                                  geometry_msgs::PoseStampedPtr &out_pose,
                                  const double wheel_angle,
                                  const double vehicle_speed,
                                  const double dt){
        tf::Pose pose;
        tf::poseMsgToTF(in_pose->pose, pose);
        double Roll,Pitch, Yaw;
        pose.getBasis().getRPY(Roll, Pitch, Yaw);
        double vehicle_heading = Yaw;

        double dx = vehicle_speed*dt * cos(vehicle_heading);
        double dy = vehicle_speed*dt * sin(vehicle_heading);
        double beta = vehicle_speed*dt/VehicleWheelBase*tan(wheel_angle);
        vehicle_heading += beta;
        //printf("speed: %.3f, dt: %.3f\n",vehicle_speed,dt);
        //New pose
        geometry_msgs::Quaternion quat_heading;
        tf::Quaternion q;
        q.setRPY(0,0,vehicle_heading);
        tf::quaternionTFToMsg(q,quat_heading);

        *out_pose = *in_pose;
        out_pose->pose.position.x += dx;
        out_pose->pose.position.y += dy;
        out_pose->pose.orientation = quat_heading;
    };

//    void BicycleModelMotionUpdate2(const geometry_msgs::PoseStampedConstPtr &in_pose,
//                                  geometry_msgs::PoseStampedPtr &out_pose,
//                                  const double wheel_angle,
//                                  const double vehicle_speed,
//                                  const double dt){
//        tf::Pose pose;
//        tf::poseMsgToTF(in_pose->pose, pose);
//        double Roll,Pitch, Yaw;
//        pose.getBasis().getRPY(Roll, Pitch, Yaw);
//        double vehicle_heading = Yaw;
//
//        double beta = vehicle_speed*dt/VehicleWheelBase*tan(wheel_angle);
//        vehicle_heading += beta;
//        double dx = vehicle_speed*dt * cos(vehicle_heading);
//        double dy = vehicle_speed*dt * sin(vehicle_heading);
//
//
//        //New pose
//        geometry_msgs::Quaternion quat_heading;
//        tf::Quaternion q;
//        q.setRPY(0,0,vehicle_heading);
//        tf::quaternionTFToMsg(q,quat_heading);
//
//        *out_pose = *in_pose;
//        out_pose->pose.position.x += dx;
//        out_pose->pose.position.y += dy;
//        out_pose->pose.orientation = quat_heading;
//    };

    void Candidate_Path_Visualize(const std::vector<Candidate_Path> &candidate_paths){

        printf("Path visualization\n");
        visualization_msgs::MarkerArrayPtr path_line_array (new visualization_msgs::MarkerArray);
        visualization_msgs::MarkerArrayPtr path_points_array (new visualization_msgs::MarkerArray);
        //visualization_msgs::MarkerPtr path_points (new visualization_msgs::Marker);

        path_line_array_->markers.clear();
        path_points_array_->markers.clear();

        for(unsigned int i=0;i<candidate_paths.size();i++){
            visualization_msgs::MarkerPtr path_line (new visualization_msgs::Marker);
            visualization_msgs::MarkerPtr path_points (new visualization_msgs::Marker);

            path_line->header.frame_id = "/odom";
            path_line->header.stamp = ros::Time(0);
            path_line->ns = "candidate_path";
            path_line->id = i;
            path_line->type = visualization_msgs::Marker::LINE_STRIP;
            path_line->action = visualization_msgs::Marker::ADD;
            path_line->pose.orientation.x = 0.0;
            path_line->pose.orientation.y = 0.0;
            path_line->pose.orientation.z = 0.0;
            path_line->pose.orientation.w = 1.0;
            path_line->scale.x = 0.02; //Line width
            path_line->scale.y = 0.0;
            path_line->scale.z = 0.0;
            path_line->color.a = 1.0;
            path_line->color.r = 1.0;
            path_line->color.g = 1.0;
            path_line->color.b = 0.0;

            path_points->header.frame_id = "/odom";
            path_points->header.stamp = ros::Time(0);
            path_points->ns = "candidate_path_points";
            path_points->id = i;
            path_points->type = visualization_msgs::Marker::POINTS;
            path_points->action = visualization_msgs::Marker::ADD;
            path_points->pose.orientation.x = 0.0;
            path_points->pose.orientation.y = 0.0;
            path_points->pose.orientation.z = 0.0;
            path_points->pose.orientation.w = 1.0;
            path_points->scale.x = 0.2; //point width
            path_points->scale.y = 0.2; //point height
            path_points->scale.z = 0.0;
            path_points->color.a = 0.8;
            path_points->color.r = 0.1;
            path_points->color.g = 0.1;
            path_points->color.b = 1.0;

            for(unsigned int j=0;j<candidate_paths.at(i).path.poses.size();j++){
                geometry_msgs::Point p;
                p.x = candidate_paths.at(i).path.poses.at(j).pose.position.x;
                p.y = candidate_paths.at(i).path.poses.at(j).pose.position.y;
                p.z = candidate_paths.at(i).path.poses.at(j).pose.position.z;
                path_line->points.push_back(p);
                path_points->points.push_back(p);
            }
            path_line_array_->markers.push_back(*path_line);
            path_points_array_->markers.push_back(*path_points);
        }
        // candidate_path_line_marker_pub_.publish(path_line_array);
        // candidate_path_point_marker_pub_.publish(path_points_array);


    };
};



#endif



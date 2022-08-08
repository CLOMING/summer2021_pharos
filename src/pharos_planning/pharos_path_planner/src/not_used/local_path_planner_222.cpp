#include <local_path_planner.h>
#include <collision_check.h>
#include <lateral_avoiding.h>



// pharos_obstacle_detection::TrackedVehicles_Info2Ptr moving_vehicles_(new pharos_obstacle_detection::TrackedVehicles_Info2);


ros::NodeHandlePtr node_;
ros::NodeHandlePtr pnode_;

extern PathPlanning3D PP3D;
VehicleCollisionCheck VehicleCollisionCheck;

Lateral_Avoiding LateralAvoiding;

pharos_path_planner::ReferencePathPtr ref_roadinfo_(new pharos_path_planner::ReferencePath);


Lateral_Avoiding::Reference_Path Ref_path_info_;
std::vector<Lateral_Avoiding::Reference_Path> SideLanes_;

pharos_path_planner::RoadInfo CurrentRoadInfo_;
pharos_road_information::LanesPtr ReferenceLanes_(new pharos_road_information::Lanes);
pharos_behavior_planner::LaneChangeStatePtr LaneChangeState_(new pharos_behavior_planner::LaneChangeState);

nav_msgs::OccupancyGridPtr final_obstacle_map_ (new nav_msgs::OccupancyGrid);

unsigned int exit_lane_number_ = 0;

void PathPlanning3D::TrackedVehiclesCallback(const geometry_msgs::PoseArrayConstPtr& msg){

    TrackedObject_List_.objects.clear();

    pharos_msgs::ObjectInfo object_info;
    for(unsigned int i=0;i<msg->poses.size();i++){
        double x = msg->poses.at(i).position.x;
        double y = msg->poses.at(i).position.y;
        double z = 0.0;

        object_info.pose.x = x;
        object_info.pose.y = y;
        object_info.pose.z = z;

        TrackedObject_List_.objects.push_back(object_info);
    }
    if(!map_get_) return;
    nav_msgs::OccupancyGridPtr init_map_ptr_ (new nav_msgs::OccupancyGrid);
    *init_map_ptr_ = init_obstacle_map_;
    PredictiveMap(init_map_ptr_,vehicle_pose_);
    predicted_map_pub_.publish(FutureMaps_.at(0));
    *final_obstacle_map_ = FutureMaps_.at(0);
    // printf("Tracked object callback\n");
    tracked_map_generation_ = true;



    // TrackedObject_List_ = *msg;
    // if(!map_get_) return;
    // nav_msgs::OccupancyGridPtr init_map_ptr_ (new nav_msgs::OccupancyGrid);
    // *init_map_ptr_ = init_obstacle_map_;
    // PredictiveMap(init_map_ptr_,vehicle_pose_);
    // predicted_map_pub_.publish(FutureMaps_.at(0));
    // *final_obstacle_map_ = FutureMaps_.at(0);
    // // printf("Tracked object callback\n");
    // tracked_map_generation_ = true;

}

void PathPlanning3D::DrivingStateCallback(const pharos_behavior_planner::DrivingStateConstPtr& msg){
    DrivingState_ = *msg;
}
void PathPlanning3D::GlobalPathUpdateCallback(const std_msgs::BoolConstPtr& msg){
    global_path_update_ = true;
}
void PathPlanning3D::ReferenceLanesCallback(const pharos_road_information::LanesConstPtr& msg){
    *ReferenceLanes_ = *msg;
}
void PathPlanning3D::LaneChangeStateCallback(const pharos_behavior_planner::LaneChangeStateConstPtr& msg){
    *LaneChangeState_ = *msg;
}

void PathPlanning3D::RoadNetworkGraphCallback(const pharos_road_information::RoadNetworksConstPtr& msg){
    if(road_networks_get_) return;
    *road_networks_ = *msg;

    // -- side lane check
    if(!wp_idx_get_) return;

    // unsigned int current_road_id = CurrentRoadInfo_.road_number;
    // unsigned int current_lane_id = CurrentRoadInfo_.lane_number;
    // static unsigned int old_road_id = current_road_id;

    // unsigned int number_of_lanes = msg->roads.at(current_road_id-1).lanes.size();
    
    // pharos_road_information::Road current_road;
    // pharos_road_information::Road next_road;

    // current_road = msg->roads.at(current_road_id-1);

    // next_road = msg->roads.at(current_road_id);



    // // if(old_road_id != current_road_id){
    //     SideLanes_.clear();
    //     if(current_road.road_type == 1 && number_of_lanes > 1){

    //         // Put Other lanes information for Lane Changing
    //         Lateral_Avoiding::Reference_Path side_lane;
    //         for(unsigned int i=0; i<number_of_lanes; i++){
    //             if((i+1) == current_lane_id) continue;

    //             geometry_msgs::PoseStamped wp_pose;
    //             geometry_msgs::PoseStamped curv_pose;
    //             geometry_msgs::PoseStamped speed_pose;

    //             pharos_road_information::Lane lane = current_road.lanes.at(i);
    //             std::vector<pharos_road_information::Waypoint>::iterator iter;
    //             for(iter = lane.waypoints.begin(); iter!=lane.waypoints.end();++iter){
    //                 wp_pose.pose.position.x = iter->x;
    //                 wp_pose.pose.position.y = iter->y;
    //                 wp_pose.pose.position.z = iter->z;

    //                 curv_pose.pose.position.x = iter->x;
    //                 curv_pose.pose.position.y = iter->y;
    //                 curv_pose.pose.position.z = iter->curvature;

    //                 speed_pose.pose.position.x = iter->x;
    //                 speed_pose.pose.position.y = iter->y;
    //                 speed_pose.pose.position.z = current_road.speed_limit; // Speed Planning is Required!!!!

    //                 side_lane.waypoint.poses.push_back(wp_pose);
    //                 side_lane.curvature.poses.push_back(curv_pose);
    //                 side_lane.speed.poses.push_back(speed_pose);
    //             }
    //             SideLanes_.push_back(side_lane);
    //         }
    //     }
    // // }
    // // printf("current_road: %d\n",current_road_id);
    // // printf("number of side lanes: %d\n",SideLanes_.size());

 
    // old_road_id = current_road_id;
    road_networks_get_ = true;
}


void PathPlanning3D::ReferenceRoadInfoCallback(const pharos_path_planner::ReferencePathConstPtr &msg){
    *ref_roadinfo_ = *msg;
}

void PathPlanning3D::CurrentRoadInfoCallback(const pharos_path_planner::RoadInfoConstPtr &msg){

}


void PathPlanning3D::VehicleOdomCallback(const nav_msgs::OdometryConstPtr &msg){
    vehicle_pose_.header = msg->header;
    vehicle_pose_.pose = msg->pose.pose;
    vehicle_odom_get_ = true;


}

void PathPlanning3D::VehicleStateCallback(const pharos_msgs::StateStamped2016ConstPtr &msg){
    vehicle_speed_ = msg->state.velocity; //[m/s]
    vehicle_speed_get_ = true;
}

void PathPlanning3D::LookaheadCallback(const std_msgs::Float32ConstPtr& msg){
    lookahaed_distance_ = msg->data;
    LateralAvoiding.lookahaed_distance_ = msg->data;
}

void PathPlanning3D::TrackedObjectCallback(const pharos_msgs::ObjectInfoArrayConstPtr &msg){
    // TrackedVehicle_List_.clear();
    // for(unsigned int i=0; i<msg->objects.size();i++){
    //     geometry_msgs::PoseStamped object_pose;
    //     object_pose.pose.position.x = msg->objects.at(i).pose.x;
    //     object_pose.pose.position.y = msg->objects.at(i).pose.y;
    //     object_pose.pose.position.z = msg->objects.at(i).pose.z;
    // }

    // TrackedObject_List_.objects.clear();
    TrackedObject_List_ = *msg;
    if(!map_get_) return;
    nav_msgs::OccupancyGridPtr init_map_ptr_ (new nav_msgs::OccupancyGrid);
    *init_map_ptr_ = init_obstacle_map_;
    PredictiveMap(init_map_ptr_,vehicle_pose_);
    predicted_map_pub_.publish(FutureMaps_.at(0));
    *final_obstacle_map_ = FutureMaps_.at(0);
    // printf("Tracked object callback\n");
    tracked_map_generation_ = true;
}

void PathPlanning3D::ObstacleMapCallback(const nav_msgs::OccupancyGridConstPtr &msg){
    ros::Time t0 = ros::Time::now();
    ros::Rate r(20);

    init_obstacle_map_ = *msg;
    map_get_ =true;
    if(!tracked_map_generation_) return;
    int planning_mode = 1; //planning mode, 1: normal driving, 2: lane changing, 3: force avoiding

    if(ReferenceLanes_->lanes.empty() || ref_roadinfo_->roadinfo.empty()){
        //reference path (global waypoint) request
        
        std_msgs::Int32 global_path_planning_request_msg;
        global_path_planning_request_msg.data = 1;
        global_path_planning_request_pub_.publish(global_path_planning_request_msg);
        return;
    }


    if(!vehicle_odom_get_ || !vehicle_speed_get_) return;

    printf("\n");

    unsigned int init_current_wp_idx;


    LateralAvoiding.reference_road_->roadinfo.clear();

    LateralAvoiding.reference_path_->waypoint.poses.clear();
    LateralAvoiding.reference_path_->curvature.poses.clear();
    LateralAvoiding.reference_path_->speed.poses.clear();
    
    Ref_path_info_.waypoint.poses.clear();
    Ref_path_info_.curvature.poses.clear();
    Ref_path_info_.speed.poses.clear();

    double search_dist = 0.0;
    unsigned int wp_idx = 0;

    pharos_path_planner::RoadInfo roadinfo1,roadinfo2;
    pharos_path_planner::ReferencePath ref_path;

    geometry_msgs::PoseStamped wp_pose;
    geometry_msgs::PoseStamped curv_pose;
    geometry_msgs::PoseStamped speed_pose;

    std::vector<pharos_path_planner::RoadInfo>::iterator iter;
    tf::Point p_vehicle, p_wp;
    p_vehicle.setValue(vehicle_pose_.pose.position.x, vehicle_pose_.pose.position.y, 0.0);
    double min_dist = DBL_MAX;
    // unsigned int i = 0;
    
    // for(iter=ref_roadinfo_->roadinfo.begin();iter!=ref_roadinfo_->roadinfo.end();++iter){
    //     p_wp.setValue(iter->position.x, iter->position.y,0.0);
    //     double dist = p_vehicle.distance(p_wp);
    //     if(dist<min_dist){
    //         min_dist = dist;
    //         wp_idx = i;
    //     }
    //     i++;
    // }
    // init_current_wp_idx = wp_idx;


    static unsigned int old_wp_idx = 0;
    static unsigned int old_ref_roadinfo_size = ref_roadinfo_->roadinfo.size();

    if(ref_roadinfo_->roadinfo.size() != old_ref_roadinfo_size) old_wp_idx = 0;

    unsigned int search_start = old_wp_idx;
    unsigned int search_finish = old_wp_idx + 50;
    if(search_finish > ref_roadinfo_->roadinfo.size()-2){
        search_finish = ref_roadinfo_->roadinfo.size()-1;
    }
    if(old_wp_idx == 0) search_finish = ref_roadinfo_->roadinfo.size()-1;
    for(unsigned int i=search_start;i<search_finish;i++){
        p_wp.setValue(ref_roadinfo_->roadinfo.at(i).position.x, ref_roadinfo_->roadinfo.at(i).position.y,0.0);
        double dist = p_vehicle.distance(p_wp);
        if(dist<min_dist){
            min_dist = dist;
            wp_idx = i;
        }
    }
    init_current_wp_idx = wp_idx;
    old_wp_idx = init_current_wp_idx;
    old_ref_roadinfo_size = ref_roadinfo_->roadinfo.size();
    printf("wp_idx: %d\n",wp_idx);


    pharos_path_planner::RoadInfo current_road_info;
    current_road_info = ref_roadinfo_->roadinfo.at(init_current_wp_idx);


    while(search_dist < 40.0){ // extract front 50m waypoints

        if(wp_idx > ref_roadinfo_->roadinfo.size()-2) break;

        roadinfo1 = ref_roadinfo_->roadinfo.at(wp_idx);
        roadinfo2 = ref_roadinfo_->roadinfo.at(wp_idx+1);

        wp_pose.pose.position.x = roadinfo1.position.x;
        wp_pose.pose.position.y = roadinfo1.position.y;
        wp_pose.pose.position.z = roadinfo1.position.z;

        curv_pose.pose.position.x = roadinfo1.position.x;
        curv_pose.pose.position.y = roadinfo1.position.y;
        curv_pose.pose.position.z = roadinfo1.curvature;
        
        speed_pose.pose.position.x = roadinfo1.position.x;
        speed_pose.pose.position.y = roadinfo1.position.y;
        speed_pose.pose.position.z = roadinfo1.reference_speed/3.6;
        
        LateralAvoiding.reference_path_->waypoint.poses.push_back(wp_pose);
        LateralAvoiding.reference_path_->curvature.poses.push_back(curv_pose);
        LateralAvoiding.reference_path_->speed.poses.push_back(speed_pose);
    
        Ref_path_info_.waypoint.poses.push_back(wp_pose);
        Ref_path_info_.curvature.poses.push_back(curv_pose);
        Ref_path_info_.speed.poses.push_back(speed_pose);
    
        tf::Point p1,p2;
        p1.setValue(roadinfo1.position.x,roadinfo1.position.y,0.0);
        p2.setValue(roadinfo2.position.x,roadinfo2.position.y,0.0);

        if(roadinfo1.road_number == roadinfo2.road_number && roadinfo1.lane_number != roadinfo2.lane_number){

        }
        else{
            search_dist += p1.distance(p2);
        }
        wp_idx++;
    }

    double remain_distance = 0.0;
    double remain_time = 0.0;
    wp_idx = init_current_wp_idx;
    while(1){

        if(wp_idx > ref_roadinfo_->roadinfo.size()-2) break;

        roadinfo1 = ref_roadinfo_->roadinfo.at(wp_idx);
        roadinfo2 = ref_roadinfo_->roadinfo.at(wp_idx+1);

        if(roadinfo1.road_number != roadinfo2.road_number || roadinfo1.lane_number != roadinfo2.lane_number){
            break;
        }

        tf::Point p1,p2;
        p1.setValue(roadinfo1.position.x,roadinfo1.position.y,0.0);
        p2.setValue(roadinfo2.position.x,roadinfo2.position.y,0.0);
        double d = p1.distance(p2);
        double t = d/(roadinfo1.reference_speed/3.6);
        remain_distance += d;
        remain_time += t;
        wp_idx++;
    }
    printf("Remain distance of current road: %.3f\n", remain_distance);
    printf("Remain time of current road: %.3f\n", remain_time);



    nav_msgs::OccupancyGridPtr obstacle_map(new nav_msgs::OccupancyGrid);
    obstacle_map->header = msg->header;
    obstacle_map->info = msg->info;
    obstacle_map->data = msg->data;


    static nav_msgs::PathPtr old_path(new nav_msgs::Path);
    nav_msgs::PathPtr old_path_front(new nav_msgs::Path);

    nav_msgs::OccupancyGridPtr accumulated_map(new nav_msgs::OccupancyGrid);

    *accumulated_map = *obstacle_map;
    *obstacle_map = *final_obstacle_map_;

    geometry_msgs::PoseStamped init_vehicle_pose = vehicle_pose_;
    double init_vehicle_speed = vehicle_speed_;
    double goal_speed = ref_goal_speed_;
    double acc = 0.0;


    // Path Planning

    geometry_msgs::PoseStampedPtr lookahead_vehicle_pose(new geometry_msgs::PoseStamped);
    *lookahead_vehicle_pose = init_vehicle_pose;

    //static double vehicle_heading = QuatToEulerHeading(init_vehicle_pose);


    tf::Pose pose;
    tf::poseMsgToTF(init_vehicle_pose.pose, pose);
    double Roll,Pitch, Yaw;
    static double vehicle_heading;
    pose.getBasis().getRPY(Roll, Pitch, Yaw);
    vehicle_heading = Yaw;

    double lookahead_margin = 0.5;
    lookahead_vehicle_pose->pose.position.x = init_vehicle_pose.pose.position.x + (lookahaed_distance_+lookahead_margin)*cos(vehicle_heading);
    lookahead_vehicle_pose->pose.position.y = init_vehicle_pose.pose.position.y + (lookahaed_distance_+lookahead_margin)*sin(vehicle_heading);

    tf::Point EgoVehiclePose;
    EgoVehiclePose.setValue(init_vehicle_pose.pose.position.x,init_vehicle_pose.pose.position.y,0.0);
    // VehicleCollisionCheck.CollisionCheck(lookahead_vehicle_pose,EgoVehiclePose,obstacle_map,accumulated_map);
    //predicted_map_pub_.publish(accumulated_map);

    // static int Planner_Failure_Counting = 0;
    // if(Planner_Failure_Counting > Re_Trial_){
    //     printf("Planning Again From the Vehicle Origin\n");
    // }
    // else{
    //     unsigned int old_path_lookahead_idx = 0;
    //     int old_path_current_idx = 0;
    //     if((old_path->poses.size()>2)){
    //         geometry_msgs::PoseStampedPtr current_vehicle_pose(new geometry_msgs::PoseStamped);
    //         *current_vehicle_pose = init_vehicle_pose;
    //         old_path_current_idx = NearestWaypointIdx3(old_path,current_vehicle_pose,0);
    //         old_path_lookahead_idx = NearestWaypointIdx3(old_path,lookahead_vehicle_pose,0);
    //         //printf("c_idx: %d, lkahd_idx: %d\n",old_path_current_idx,old_path_lookahead_idx);
    //         if(old_path_lookahead_idx>(old_path->poses.size()-2)) old_path_lookahead_idx =old_path->poses.size()-2;
    //         old_path_current_idx -= 7;
    //         if(old_path_current_idx<0) old_path_current_idx = 0;
    //         for(unsigned int i=old_path_current_idx;i<old_path_lookahead_idx+1;i++){
    //             //for(unsigned int i=0;i<old_path_current_idx+1;i++){
    //             old_path_front->poses.push_back(old_path->poses.at(i));
    //         }
    //         init_vehicle_pose = *lookahead_vehicle_pose;
    //         init_vehicle_pose.pose = old_path->poses.at(old_path_lookahead_idx+1).pose;
    //     }
    //     if(old_path_current_idx > old_path->poses.size()/2){
    //         init_vehicle_pose = *lookahead_vehicle_pose;
    //         old_path_front->poses.clear();
    //     }
    // }

    Lateral_Avoiding::Reference_Path FinalLocalPath;

    nav_msgs::PathPtr lateral_avoiding_path(new nav_msgs::Path);

    static unsigned int old_path_id = 0;
    int lateral_avoiding_result = 0;
    printf("Lateral Avoiding planning start\n");
    *(LateralAvoiding.reference_path_) = Ref_path_info_;

    double max_lat_offset = 0.0;
    planning_mode = 1;
    lateral_avoiding_result = LateralAvoiding.LateralAvoidingPlanning(LateralAvoiding.reference_path_,
                                                            init_vehicle_pose,
                                                            obstacle_map,
                                                            EgoVehiclePose,
                                                            init_current_wp_idx,
                                                            init_vehicle_speed,
                                                            planning_mode,
                                                            old_path_id,
                                                            lateral_avoiding_path,
                                                            accumulated_map);


    LateralAvoiding.candidate_path_line_marker_pub_.publish(LateralAvoiding.path_line_array_);
    LateralAvoiding.candidate_path_point_marker_pub_.publish(LateralAvoiding.path_points_array_);
    //predicted_map_pub_.publish(accumulated_map);

    Lateral_Avoiding::Reference_Path LateralAvoiding_Ref_Path;
    LateralAvoiding_Ref_Path.waypoint = *LateralAvoiding.final_path_;
    LateralAvoiding_Ref_Path.curvature = *LateralAvoiding.final_path_curvature_;
    LateralAvoiding_Ref_Path.speed = *LateralAvoiding.final_path_speed_;
    

    static double stopping_time = 0.0;
    ros::Time T0 = ros::Time::now();
    static ros::Time T1 = T0;
    double dt = (T0-T1).toSec();
    T1 = T0;
    
    if(lateral_avoiding_result == -1){
        for(unsigned int i=0;i<LateralAvoiding_Ref_Path.speed.poses.size();i++){
            LateralAvoiding_Ref_Path.speed.poses.at(i).pose.position.z = 0.0;
        }
    }
    else if(lateral_avoiding_result == 2){

        bool SideBack = true;
        SideBack = LaneChangeState_->left;


        double path_dist = 0;
        for(unsigned int i=0;i<LateralAvoiding_Ref_Path.speed.poses.size()-1;i++){
            tf::Point p1,p2;
            double d = p1.distance(p2);
            path_dist += d;
        }
        if(path_dist < 5.0){
            for(unsigned int j=0;j<LateralAvoiding_Ref_Path.speed.poses.size();j++){
                LateralAvoiding_Ref_Path.speed.poses.at(j).pose.position.z = 0.0;
            }
        }
        if(DrivingState_.action != 2 && vehicle_speed_ < 0.001){
            stopping_time += dt;
            if(ForceAvoiding_){
                
            }
            else{
                printf("stopping time: %.4f\n",stopping_time);
                if(stopping_time > force_avoiding_waiting_time_ && SideBack){
                    ForceAvoiding_ = true;
                    stopping_time = 0.0;
                }
            }
            
        }
    }
    else if(lateral_avoiding_result == 1){
        ForceAvoiding_ = false;
        stopping_time = 0.0;
    }
    else if(lateral_avoiding_result == 3){
        //if(current_road_info.road_type == 1){
            double path_dist = 0;
            for(unsigned int i=0;i<LateralAvoiding_Ref_Path.speed.poses.size()-1;i++){
                tf::Point p1,p2;
                double d = p1.distance(p2);
                path_dist += d;
            }
            if(path_dist < 3.0){
                for(unsigned int j=0;j<LateralAvoiding_Ref_Path.speed.poses.size();j++){
                    LateralAvoiding_Ref_Path.speed.poses.at(j).pose.position.z = 0.0;
                }
            }
        //}
    }
    // ForceAvoiding_ = true;

    static unsigned int lateral_offset_check = 0;
    int origin_num_candidate_paths = LateralAvoiding.Num_Candidate_Paths_;
    double origin_path_lateral_offset = LateralAvoiding.Path_Lateral_offset_;
    if(ForceAvoiding_){
        printf("Force Avoiding!\n");

        int force_avoiding_result = 0;
        static int avoiding_planning_count = 0;
        // while(force_avoiding_result != 1){
        //     force_avoiding_result = LateralAvoiding.LateralAvoidingPlanning(LateralAvoiding.reference_path_,
        //                                                         init_vehicle_pose,
        //                                                         obstacle_map,
        //                                                         EgoVehiclePose,
        //                                                         init_current_wp_idx,
        //                                                         init_vehicle_speed,
        //                                                         max_lat_offset,
        //                                                         planning_mode,
        //                                                         old_path_id,
        //                                                         lateral_avoiding_path,
        //                                                         accumulated_map);
            
        //     LateralAvoiding.force_avoid_path_line_marker_pub_.publish(LateralAvoiding.path_line_array_);
        //     LateralAvoiding.force_avoid_path_point_marker_pub_.publish(LateralAvoiding.path_points_array_);
            
        //     avoiding_planning_count++;
        //     if(avoiding_planning_count > 5){
        //         printf("Cannot avoid.. Road is blocked\n");
        //         break;
        //     }
        // }

        max_lat_offset = 0.0;
        planning_mode = 3;
        force_avoiding_result = LateralAvoiding.LateralAvoidingPlanning(LateralAvoiding.reference_path_,
                                                            init_vehicle_pose,
                                                            obstacle_map,
                                                            EgoVehiclePose,
                                                            init_current_wp_idx,
                                                            init_vehicle_speed,
                                                            planning_mode,
                                                            old_path_id,
                                                            lateral_avoiding_path,
                                                            accumulated_map);
        
        LateralAvoiding.force_avoid_path_line_marker_pub_.publish(LateralAvoiding.path_line_array_);
        LateralAvoiding.force_avoid_path_point_marker_pub_.publish(LateralAvoiding.path_points_array_);
        

        // if(force_avoiding_result == 1){

        //     printf("Force Avoiding path publish\n");
        //     FinalLocalPath.waypoint = *LateralAvoiding.final_path_;
        //     FinalLocalPath.curvature = *LateralAvoiding.final_path_curvature_;
        //     FinalLocalPath.speed = *LateralAvoiding.final_path_speed_;

        //     FinalLocalPath.waypoint.header.frame_id = "odom";
        //     FinalLocalPath.waypoint.header.stamp = obstacle_map->header.stamp;
        //     FinalLocalPath.speed.header.frame_id = "odom";
        //     FinalLocalPath.speed.header.stamp = obstacle_map->header.stamp;
        //     FinalLocalPath.curvature.header.frame_id = "odom";
        //     FinalLocalPath.curvature.header.stamp = obstacle_map->header.stamp;

        //     final_path_pub_.publish(FinalLocalPath.waypoint);
        //     planner_speed_pub_.publish(FinalLocalPath.speed);
        //     planner_curvature_pub_.publish(FinalLocalPath.curvature);

        //     return;
        // }
        // else{
        // }

        printf("Force Avoiding path publish\n");
        FinalLocalPath.waypoint = *LateralAvoiding.final_path_;
        FinalLocalPath.curvature = *LateralAvoiding.final_path_curvature_;
        FinalLocalPath.speed = *LateralAvoiding.final_path_speed_;

        FinalLocalPath.waypoint.header.frame_id = "odom";
        FinalLocalPath.waypoint.header.stamp = obstacle_map->header.stamp;
        FinalLocalPath.speed.header.frame_id = "odom";
        FinalLocalPath.speed.header.stamp = obstacle_map->header.stamp;
        FinalLocalPath.curvature.header.frame_id = "odom";
        FinalLocalPath.curvature.header.stamp = obstacle_map->header.stamp;

        final_path_pub_.publish(FinalLocalPath.waypoint);
        planner_speed_pub_.publish(FinalLocalPath.speed);
        planner_curvature_pub_.publish(FinalLocalPath.curvature);

        return;
        // printf("Force Avoiding path publish\n");
        
        

        // if(lateral_offset_check == 0 && LateralOffset_ > 1.0){
        //     lateral_offset_check++;
        // }
        // if(lateral_offset_check == 1 && LateralOffset_ < 0.5){
        //     lateral_offset_check = 0;
        //     ForceAvoiding_ = false;
        //     stopping_time = 0.0;
        // }
        
    }
    else{
        LateralAvoiding.Num_Candidate_Paths_ =  origin_num_candidate_paths;
        LateralAvoiding.Path_Lateral_offset_ = origin_path_lateral_offset;


    }

    printf("Lateral Avoiding planning finish\n");
    FinalLocalPath = LateralAvoiding_Ref_Path;


    //- Lane changing path planning


    //-- Desired lane selecting
    pharos_road_information::Road CurrentRoad;

    pharos_road_information::LanePtr CurrentLane(new pharos_road_information::Lane);
    pharos_road_information::LanePtr ExitLane(new pharos_road_information::Lane);

    std::vector<pharos_road_information::Lane> SideLanes;

    unsigned int current_road_id = current_road_info.road_number;
    unsigned int current_lane_id = current_road_info.lane_number;

    CurrentRoad = road_networks_->roads.at(current_road_id-1);
    *CurrentLane = CurrentRoad.lanes.at(current_lane_id-1);

    unsigned int number_of_lanes = CurrentRoad.lanes.size();
    for(unsigned int i=0;i<number_of_lanes;i++){
        if(CurrentRoad.lanes.at(i).lane_number == CurrentLane->lane_number) continue;
        SideLanes.push_back(CurrentRoad.lanes.at(i));
    }
    
    
    *ExitLane = *CurrentLane;

    // Exit Lane Searching
    unsigned int idx_of_current_lane = 0;
    for(unsigned int i=0;i<ReferenceLanes_->lanes.size();i++){
        if(ReferenceLanes_->lanes.at(i).road_number == current_road_id){
            idx_of_current_lane = i;
            break;
        }
    }
    unsigned int lane_idx = idx_of_current_lane;
    while(current_road_id == ReferenceLanes_->lanes.at(lane_idx).road_number){
        *ExitLane = ReferenceLanes_->lanes.at(lane_idx);
        lane_idx++;
        if(lane_idx == ReferenceLanes_->lanes.size()) break;
    }
    
    printf("Road: %d, lane: %d, exit lane: %d\n",current_road_id,current_lane_id,ExitLane->lane_number);
    printf("Current lane, road: %d, lane: %d\n",CurrentLane->road_number, CurrentLane->lane_number);
    // printf("Next lane,    road: %d, lane: %d\n",ReferenceLanes_->lanes.at(idx_of_current_lane+1).road_number, ReferenceLanes_->lanes.at(idx_of_current_lane+1).lane_number);


    int lane_changing_planning_result = 0;

    nav_msgs::PathPtr lane_changing_path(new nav_msgs::Path);
    
    
    Lateral_Avoiding::Reference_Path LaneChanging_Ref_Path;
    if(number_of_lanes>1){

        // Lane Changing Case 1
        if(CurrentLane->lane_number != ExitLane->lane_number){

            Lateral_Avoiding::Reference_Path side_lane;
            
            geometry_msgs::PoseStamped wp_pose;
            geometry_msgs::PoseStamped curv_pose;
            geometry_msgs::PoseStamped speed_pose;

            if(abs(CurrentLane->lane_number - ExitLane->lane_number)>1){
                unsigned int desired_side_lane_number = 0;

                if(CurrentLane->lane_number > ExitLane->lane_number){
                    desired_side_lane_number = CurrentLane->lane_number - 1;
                }
                else{
                    desired_side_lane_number = CurrentLane->lane_number + 1;
                }

                unsigned int desired_side_lane_idx = 0;
                for(unsigned int i=0; i<SideLanes.size();i++){
                    if(SideLanes.at(i).lane_number == desired_side_lane_number){
                        desired_side_lane_idx = i;
                    }
                }

                std::vector<pharos_road_information::Waypoint>::iterator iter;
                for(iter = SideLanes.at(desired_side_lane_idx).waypoints.begin(); iter!=SideLanes.at(desired_side_lane_idx).waypoints.end();++iter){
                    wp_pose.pose.position.x = iter->x;
                    wp_pose.pose.position.y = iter->y;
                    wp_pose.pose.position.z = iter->z;

                    curv_pose.pose.position.x = iter->x;
                    curv_pose.pose.position.y = iter->y;
                    curv_pose.pose.position.z = iter->curvature;

                    speed_pose.pose.position.x = iter->x;
                    speed_pose.pose.position.y = iter->y;
                    speed_pose.pose.position.z = CurrentRoad.speed_limit/3.6; // Speed Planning is Required!!!!

                    side_lane.waypoint.poses.push_back(wp_pose);
                    side_lane.curvature.poses.push_back(curv_pose);
                    side_lane.speed.poses.push_back(speed_pose);
                }
            }

            else{
                std::vector<pharos_road_information::Waypoint>::iterator iter;
                for(iter = ExitLane->waypoints.begin(); iter!=ExitLane->waypoints.end();++iter){
                    wp_pose.pose.position.x = iter->x;
                    wp_pose.pose.position.y = iter->y;
                    wp_pose.pose.position.z = iter->z;

                    curv_pose.pose.position.x = iter->x;
                    curv_pose.pose.position.y = iter->y;
                    curv_pose.pose.position.z = iter->curvature;

                    speed_pose.pose.position.x = iter->x;
                    speed_pose.pose.position.y = iter->y;
                    speed_pose.pose.position.z = CurrentRoad.speed_limit/3.6; // Speed Planning is Required!!!!

                    side_lane.waypoint.poses.push_back(wp_pose);
                    side_lane.curvature.poses.push_back(curv_pose);
                    side_lane.speed.poses.push_back(speed_pose);
                }
            }

            *(LateralAvoiding.reference_path_) = side_lane;
            planning_mode = 2;
            lane_changing_planning_result = LateralAvoiding.LateralAvoidingPlanning(LateralAvoiding.reference_path_,
                                                                                    init_vehicle_pose,
                                                                                    obstacle_map,
                                                                                    EgoVehiclePose,
                                                                                    init_current_wp_idx,
                                                                                    init_vehicle_speed,
                                                                                    planning_mode,
                                                                                    old_path_id,
                                                                                    lane_changing_path,
                                                                                    accumulated_map);

            if(lane_changing_planning_result == 0){
                ROS_ERROR("lane chaning result is 0 !!!");
            }
            
            LateralAvoiding.lane_change_path_line_marker_pub_.publish(LateralAvoiding.path_line_array_);
            LateralAvoiding.lane_change_path_point_marker_pub_.publish(LateralAvoiding.path_points_array_);

            LaneChanging_Ref_Path.waypoint = *LateralAvoiding.final_path_;
            LaneChanging_Ref_Path.curvature = *LateralAvoiding.final_path_curvature_;
            LaneChanging_Ref_Path.speed = *LateralAvoiding.final_path_speed_;
        }

        // Lane Changing Case 2
        // if(lateral_avoiding_result != 1 && vehicle_speed_*3.6 < current_road_info.reference_speed*0.3){
        // // if(lateral_avoiding_result != 1 && vehicle_speed_*3.6 < current_road_info.reference_speed*0.6){
        //     if(remain_distance > 5.0){
        //         printf("Current Lane is blocked, Lane changng\n");
        //         Lateral_Avoiding::Reference_Path side_lane;
                
        //         unsigned int desired_side_lane_idx = 0;
        //         unsigned int desired_side_lane_number = 0;

        //         if(CurrentLane->lane_number > 1){
        //             desired_side_lane_number = CurrentLane->lane_number - 1;
        //         }
        //         else{
        //             desired_side_lane_number = CurrentLane->lane_number + 1;
        //         }
        //         for(unsigned int i=0; i<SideLanes.size();i++){
        //             if(SideLanes.at(i).lane_number == desired_side_lane_number){
        //                 desired_side_lane_idx = i;
        //             }
        //         }
        //         printf("111\n");
        //         std::vector<pharos_road_information::Waypoint>::iterator iter;
        //         printf("lane_size: %d, desired lane: %d\n",SideLanes.size(),desired_side_lane_idx);
        //         for(iter = SideLanes.at(desired_side_lane_idx).waypoints.begin(); iter!=SideLanes.at(desired_side_lane_idx).waypoints.end();++iter){
        //             wp_pose.pose.position.x = iter->x;
        //             wp_pose.pose.position.y = iter->y;
        //             wp_pose.pose.position.z = iter->z;

        //             curv_pose.pose.position.x = iter->x;
        //             curv_pose.pose.position.y = iter->y;
        //             curv_pose.pose.position.z = iter->curvature;

        //             speed_pose.pose.position.x = iter->x;
        //             speed_pose.pose.position.y = iter->y;
        //             speed_pose.pose.position.z = CurrentRoad.speed_limit/3.6; // Speed Planning is Required!!!!

        //             side_lane.waypoint.poses.push_back(wp_pose);
        //             side_lane.curvature.poses.push_back(curv_pose);
        //             side_lane.speed.poses.push_back(speed_pose);
        //         }
        //         *(LateralAvoiding.reference_path_) = side_lane;
        //         printf("222\n");
        //         max_lat_offset = 1.5;
        //         lane_changing_planning_result = LateralAvoiding.LateralAvoidingPlanning(LateralAvoiding.reference_path_,
        //                                                                                 init_vehicle_pose,
        //                                                                                 obstacle_map,
        //                                                                                 EgoVehiclePose,
        //                                                                                 init_current_wp_idx,
        //                                                                                 init_vehicle_speed,
        //                                                                                 max_lat_offset,
        //                                                                                 old_path_id,
        //                                                                                 lane_changing_path,
        //                                                                                 accumulated_map);

        //         if(lane_changing_planning_result == 0){
        //             ROS_ERROR("lane chaning result is 0 !!!");
        //         }
                
        //         LateralAvoiding.lane_change_path_line_marker_pub_.publish(LateralAvoiding.path_line_array_);
        //         LateralAvoiding.lane_change_path_point_marker_pub_.publish(LateralAvoiding.path_points_array_);

        //         LaneChanging_Ref_Path.waypoint = *LateralAvoiding.final_path_;
        //         LaneChanging_Ref_Path.curvature = *LateralAvoiding.final_path_curvature_;
        //         LaneChanging_Ref_Path.speed = *LateralAvoiding.final_path_speed_;
        //     }
        // }
    }
    else{

    }

    static unsigned int lane_changing_ready_count = 0;
    double lane_chaning_finish_rate = 0.5; // lane changing is finished when 

    if(lane_changing_planning_result != 0){

        double lat_offset_current_lane = 0;
        double lat_offset_desired_lane = 0;
        bool SideBack = true;
        if(CurrentLane->lane_number == 1)
            SideBack = LaneChangeState_->right;
        else if(CurrentLane->lane_number == 2)
            SideBack = LaneChangeState_->left;

        if(lane_change_state_ == 0){
            if(lane_changing_planning_result == 1){
                lane_change_state_ = LANE_CHANGING_READY;
            }
            else if(lane_changing_planning_result == 3){
                std_msgs::Int32 global_path_planning_request_msg;
                global_path_planning_request_msg.data = 2; // 2: side lane block
                global_path_planning_request_pub_.publish(global_path_planning_request_msg);
                printf("Cannot change the lane. Remain road is too short!\n");
            }
            else{

            }
        }
        else if(lane_change_state_ == LANE_CHANGING_READY && SideBack){
            if(lane_changing_planning_result == 1){
                lane_changing_ready_count++;
                if(lane_changing_ready_count > 5){
                    lane_change_state_ = LANE_CHANGING_START;
                    lane_changing_ready_count = 0;
                }
            }
            else if(lane_changing_planning_result == 3){
                std_msgs::Int32 global_path_planning_request_msg;
                global_path_planning_request_msg.data = 2; // 2: side lane block
                global_path_planning_request_pub_.publish(global_path_planning_request_msg);
                printf("Cannot change the lane. Remain road is too short!\n");

            }
            else{

            }
        }
        else if(lane_change_state_ == LANE_CHANGING_START){
            if(!SideBack){
                lane_change_state_ = LANE_CHANGING_READY;
                printf("Stop lane changing,,,because of side back vehicle\n");
            }
            else{
                lat_offset_current_lane = fabs(LateralOffsetwithWaypoints(CurrentLane,init_vehicle_pose));
                lat_offset_desired_lane = fabs(LateralOffsetwithWaypoints(ExitLane,init_vehicle_pose));

                lane_approach_ = lat_offset_desired_lane/(lat_offset_desired_lane+lat_offset_current_lane);

                printf("lane_approach: %.3f\n", lane_approach_);
                if(lane_changing_planning_result == 1){
                    FinalLocalPath = LaneChanging_Ref_Path;
                        
                    if(lane_approach_ < 0.65){
                        lane_change_state_ = LANE_CHANGING_FINISH;
                        std_msgs::Int32 global_path_planning_request_msg;
                        global_path_planning_request_msg.data = 1;
                        global_path_planning_request_pub_.publish(global_path_planning_request_msg);
                    }
                }
                else if(lane_changing_planning_result == 2){
                    lane_change_state_ = LANE_CHANGING_READY;
                }
                else if(lane_changing_planning_result == 3){
                    if(lane_approach_ > 0.6){
                        std_msgs::Int32 global_path_planning_request_msg;
                        global_path_planning_request_msg.data = 2; // 2: side lane block
                        global_path_planning_request_pub_.publish(global_path_planning_request_msg);
                        printf("Cannot change the lane. Remain road is too short!\n");
                    }
                    else{
                        FinalLocalPath = LaneChanging_Ref_Path;

                        if(lane_approach_ < 0.65){
                            lane_change_state_ = LANE_CHANGING_FINISH;
                            std_msgs::Int32 global_path_planning_request_msg;
                            global_path_planning_request_msg.data = 1;
                            global_path_planning_request_pub_.publish(global_path_planning_request_msg);
                            
                        }
                    }
                }
                else{
                    if(lane_approach_ < 0.65){
                        FinalLocalPath = LaneChanging_Ref_Path;
                    }
                    else{
                        FinalLocalPath = LaneChanging_Ref_Path;

                        if(lane_approach_ < 0.65){
                            lane_change_state_ = LANE_CHANGING_FINISH;
                            std_msgs::Int32 global_path_planning_request_msg;
                            global_path_planning_request_msg.data = 1;
                            global_path_planning_request_pub_.publish(global_path_planning_request_msg);
                        }
                    }
                }
            }
            
        }

        else if(lane_change_state_ == LANE_CHANGING_FINISH){
            
            if(!global_path_update_ || ExitLane->lane_number != current_lane_id){ // waiting until global path is updated
                FinalLocalPath = LaneChanging_Ref_Path;
            }
            else{
                lane_change_state_ = 0;
                global_path_update_ = false;
                lane_approach_ = 0.0;
                max_lat_offset = 0.0;
                planning_mode = 1;
                lateral_avoiding_result = LateralAvoiding.LateralAvoidingPlanning(LateralAvoiding.reference_path_,
                                                                        init_vehicle_pose,
                                                                        obstacle_map,
                                                                        EgoVehiclePose,
                                                                        init_current_wp_idx,
                                                                        init_vehicle_speed,
                                                                        planning_mode,
                                                                        old_path_id,
                                                                        lateral_avoiding_path,
                                                                        accumulated_map);

                LateralAvoiding.candidate_path_line_marker_pub_.publish(LateralAvoiding.path_line_array_);
                LateralAvoiding.candidate_path_point_marker_pub_.publish(LateralAvoiding.path_points_array_);

                Lateral_Avoiding::Reference_Path LateralAvoiding_Ref_Path;
                LateralAvoiding_Ref_Path.waypoint = *LateralAvoiding.final_path_;
                LateralAvoiding_Ref_Path.curvature = *LateralAvoiding.final_path_curvature_;
                LateralAvoiding_Ref_Path.speed = *LateralAvoiding.final_path_speed_;

                printf("Lateral Avoiding planning with updated global path\n");
                FinalLocalPath = LateralAvoiding_Ref_Path;
            }
        }
    }
    else{
        lane_change_state_ = 0;
        global_path_update_ = false;
        lane_approach_ = 0.0;
        // static bool force_avoiding = false;
        // if(DrivingState_.situation == 1){
        //     if(lateral_avoiding_result == -1 || lateral_avoiding_result == 2){
        //         if(vehicle_speed_*3.6 < current_road_info.reference_speed*0.3){
        //             force_avoiding = true;
        //         }
        //     }
        // }
        // if(force_avoiding){
        //     printf("road is blocked force avoiding is required!!!\n");
        // }
    }
    printf("lateral_avoiding result: %d\n", lateral_avoiding_result);
    

    printf("Lane change state: %d\n",lane_change_state_);

    FinalLocalPath.waypoint.header.frame_id = "odom";
    FinalLocalPath.waypoint.header.stamp = obstacle_map->header.stamp;
    FinalLocalPath.speed.header.frame_id = "odom";
    FinalLocalPath.speed.header.stamp = obstacle_map->header.stamp;
    FinalLocalPath.curvature.header.frame_id = "odom";
    FinalLocalPath.curvature.header.stamp = obstacle_map->header.stamp;

    final_path_pub_.publish(FinalLocalPath.waypoint);
    planner_speed_pub_.publish(FinalLocalPath.speed);
    planner_curvature_pub_.publish(FinalLocalPath.curvature);



//     if(!planner_solution_path->poses.empty()){
//         // ---- Final Path: Old Path front part + Planner solution path
//         nav_msgs::PathPtr final_path(new nav_msgs::Path);
//         nav_msgs::PathPtr final_splined_path(new nav_msgs::Path);

//         // for(unsigned int i=0;i<old_path_front->poses.size();i++){
//         //     final_path->poses.push_back(old_path_front->poses.at(i));
//         // }
//         for(unsigned int i=0;i<planner_solution_path->poses.size();i++){
//             final_path->poses.push_back(planner_solution_path->poses.at(i));
//         }

//         final_path->header.frame_id = "odom";
//         final_path->header.stamp = obstacle_map->header.stamp;
//         // final_path_pub_.publish(final_path);
//         *old_path = *final_path;

//         final_path_pub_.publish(FinalLocalPath.waypoint);
//         planner_speed_pub_.publish(FinalLocalPath.speed);
//         planner_curvature_pub_.publish(FinalLocalPath.curvature);

// //        final_splined_path->header.frame_id = "odom";
// //        final_splined_path->header.stamp = obstacle_map->header.stamp;
// //        B_spline(final_path,final_splined_path);
// //        //*final_splined_path = *final_path;
// //        final_path_pub_.publish(final_splined_path);
// //        *old_path = *final_splined_path;

//         // Planner_Failure_Counting = 0;
//     }
//     else{
//         ROS_ERROR("Fail to find a Path!!... ");
//         // Planner_Failure_Counting++;
//     }



    ros::Time t_end = ros::Time::now();
    double Dt = (t_end-t0).toSec();
    static double max_dt = 0.0;
//        if(Dt>max_dt){
//            max_dt = Dt;
//            printf("Max Processing Time: %.4f\n",Dt);
//        }
    printf("Path Planner processing time: %.4f\n", Dt);


    //r.sleep();
}


int PathPlanning3D::init(){

    road_networks_ = pharos_road_information::RoadNetworksPtr(new pharos_road_information::RoadNetworks);

    node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
    pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    pnode_->param("planning_time", Planning_Total_Time_, 3.0);
    pnode_->param("planning_dt", Planning_Dt_, 0.2);
    pnode_->param("Re_Trial", Re_Trial_, 10);

    pnode_->param("waiting_time_for_force_avoiding", force_avoiding_waiting_time_, 10.0);



    FutureMaps_.resize((int)(Planning_Total_Time_/Planning_Dt_)+1);


    // vehicle_odom_sub_ = node_->subscribe("/odom/ekf",10,&PathPlanning3D::VehicleOdomCallback,this);
    tracked_vehicles_sub_ = node_->subscribe("/wave_posearray",10,&PathPlanning3D::TrackedVehiclesCallback,this); 
    // tracekd_objects_sub_ = node_->subscribe("/tracked_pharos_msgs",10,&PathPlanning3D::TrackedObjectCallback,this);
    vehicle_odom_sub_ = node_->subscribe("/odom/ekf",10,&PathPlanning3D::VehicleOdomCallback,this);
    vehicle_state_sub_ = node_->subscribe("/vehicle/state2016",10,&PathPlanning3D::VehicleStateCallback,this);

    // vehicle_tracking_info_sub_ = node_->subscribe("/tracking_ekf",10,&PathPlanning3D::TrackedVehicleInfoCallback,this);
    //vehicle_tracking_info_sub_ = node_->subscribe("/tracked_vehicle_info",10,&PathPlanning3D::TrackedVehicleInfoCallback,this);

    ref_road_network_sub_ = node_->subscribe("road_network_graph",3, &PathPlanning3D::RoadNetworkGraphCallback, this);
    ref_lanes_sub_ = node_->subscribe("reference_lanes",3, &PathPlanning3D::ReferenceLanesCallback, this);

    ref_roadinfo_sub_ = node_->subscribe("/ref_road_info_with_speed_profile", 3, &PathPlanning3D::ReferenceRoadInfoCallback, this);
    current_roadinfo_sub_ = node_->subscribe("/current_road_info", 5, &PathPlanning3D::CurrentRoadInfoCallback, this);

    obstacle_map_sub_ = node_->subscribe("/obstacle_map",5,&PathPlanning3D::ObstacleMapCallback,this);

    lookahead_dist_sub_ = node_->subscribe("/vehicle/lookahead_dist",10,&PathPlanning3D::LookaheadCallback,this);

    global_path_update_sub_ = node_->subscribe("global_path_update",10,&PathPlanning3D::GlobalPathUpdateCallback,this);

    DrivingState_sub_ = node_->subscribe("/behavior/driving_state",5, &PathPlanning3D::DrivingStateCallback, this);
    LaneChangeState_sub_ = node_->subscribe("/behavior/LaneChangeState",5, &PathPlanning3D::LaneChangeStateCallback, this);

    
    final_path_pub_ = node_->advertise<nav_msgs::Path>(std::string("/final_path/path"), 10);
    planner_speed_pub_ = node_->advertise<nav_msgs::Path>("/final_path/speed", 10);
    planner_curvature_pub_ = node_->advertise<nav_msgs::Path>("/final_path/curvature", 10);

    side_lane_path_pub_ = node_->advertise<nav_msgs::Path>("side_lane_path",10);
    lane_changing_path_pub_ = node_->advertise<nav_msgs::Path>("lane_changing_path",10);

    global_path_planning_request_pub_ = node_->advertise<std_msgs::Int32>("global_path_planning_request",10);

    predicted_map_pub_ = node_->advertise<nav_msgs::OccupancyGrid>("PredictedMap",10);

    vehicle_speed_ = 0.0;
    ref_goal_speed_ = 0.0;
    ref_goal_speed_dist_ = 0.0;
    lookahaed_distance_ = 2.0;
    //lookahaed_distance_ = 3.5;
    current_wp_idx_ = 0;
    wp_idx_get_ = false;
    vehicle_odom_get_ = false;
    vehicle_speed_get_ = false;
    road_networks_get_ = false;

    lane_change_state_ = 0;
    global_path_update_ = false;
    lane_approach_ = 0.0;

    map_get_ = false;

    ForceAvoiding_ = false;
    tracked_map_generation_ = false;

    return 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "3d_path_planner_node");

    PathPlanning3D path_planning;

    if (path_planning.init())
    {
        ROS_FATAL("3d_path_planner_node initialization failed");
        return -1;
    }

    if (LateralAvoiding.init())
    {
        ROS_FATAL("3d_path_planner_node [lateral avoiding] initialization failed");
        return -1;   
    }

    ros::spin();

    return 0;
}





#include <lateral_avoiding.h>
#include <collision_check.h>
//#include <local_path_planner.h>
#include <pharos_msgs/MotionCommandStamped3.h>
extern VehicleCollisionCheck VehicleCollisionCheck;

int Lateral_Avoiding::LateralAvoidingPlanning(const Lateral_Avoiding::Reference_Path *reference_path,
                                             const geometry_msgs::PoseStamped init_vehicle_pose,
                                             const nav_msgs::OccupancyGridConstPtr& obstacle_map,
                                             const tf::Point EgoVehiclePose,
                                             const unsigned int init_current_wp_idx,
                                             const double init_vehicle_speed,
                                             const int planning_mode,
                                             unsigned int &path_id,
                                             nav_msgs::PathPtr &final_path,
                                             nav_msgs::OccupancyGridPtr &collision_check_map,
                                             const int Mission_State) {

    int result = 0;
    unsigned int old_path_id = path_id;

    //----Path Lateral Offset Value Generation

    unsigned int num_candidate_path = Num_Candidate_Paths_;
    double path_lateral_offset = Path_Lateral_offset_;
    unsigned int planning_step = planning_steps_;

    if(Mission_State == 1)
    {
        num_candidate_path = 1;
    }
    else{
        num_candidate_path = Num_Candidate_Paths_;
    }



    std::cout << "planning_mode: "<< planning_mode <<std::endl;
    //// ************************************************
    double collision_margin_scale = 1.0;
    if(planning_mode == 3){
        num_candidate_path = 15;
        path_lateral_offset = 0.5;
        planning_step = planning_steps_ + 20;
        collision_margin_scale = 1.35;   // avoid margin
    }
    else if(planning_mode == 2){
        num_candidate_path = 5;
        planning_step = planning_steps_ ;  // original :: planning steps_-5
        collision_margin_scale = 1.0;
    }
    //// ************************************************


    std::vector<double> PATHS_OFFSETs;
    for(unsigned i=0; i<num_candidate_path;i++){
        double path_offset;

        if((i%2)== 0) path_offset = path_lateral_offset * (i/2);
        else path_offset = (-1.0)*path_lateral_offset * ((i+1)/2);
        PATHS_OFFSETs.push_back(path_offset);
    }

    //----Lateral Avoiding Path Generation

    std::vector<Candidate_Path> PATHS;
    int collision_count;
    geometry_msgs::PoseStampedPtr vehicle_pose(new geometry_msgs::PoseStamped);
    geometry_msgs::PoseStampedPtr curv_pose(new geometry_msgs::PoseStamped);  //// **
    geometry_msgs::PoseStampedPtr speed_pose(new geometry_msgs::PoseStamped); //// **
    geometry_msgs::PoseStampedPtr new_vehicle_pose(new geometry_msgs::PoseStamped);

    double vehicle_speed = init_vehicle_speed;
    // unsigned int planning_steps_ = (int)(Planning_Total_Time_/Planning_Dt_)+1;

    // printf("lateral avoiding: 1\n");
    dcc_G_ = Init_dcc_G;

    do{
        collision_count = 0;
        PATHS.clear();

        double travel_distnace_ = 0.0;
        double tmp = 0.0;

        // printf("lateral avoiding: 1-1\n");
        for(unsigned i=0; i<num_candidate_path;i++){

            nav_msgs::PathPtr candidate_path(new nav_msgs::Path);
            nav_msgs::PathPtr candidate_curvature(new nav_msgs::Path);
            nav_msgs::PathPtr candidate_speed(new nav_msgs::Path);

            Candidate_Path virtual_path;

            unsigned int current_wp_idx = init_current_wp_idx;

            *vehicle_pose = init_vehicle_pose;
            *curv_pose = init_vehicle_pose;
            *speed_pose = init_vehicle_pose;
//            ROS_ERROR("init_vehicle_speed :: %lf ", init_vehicle_speed*3.6);
            curv_pose->pose.position.z = 0.0;
            speed_pose->pose.position.z =init_vehicle_speed*3.6;

            candidate_path->header.frame_id = "odom";
            candidate_path->header.stamp = obstacle_map->header.stamp;
            candidate_path->poses.push_back(*vehicle_pose);

            candidate_curvature->header.frame_id = "odom";
            candidate_curvature->header.stamp = obstacle_map->header.stamp;
            candidate_curvature->poses.push_back(*curv_pose);

            candidate_speed->header.frame_id = "odom";
            candidate_speed->header.stamp = obstacle_map->header.stamp;
            candidate_speed->poses.push_back(*speed_pose);

            vehicle_speed = init_vehicle_speed;

            double max_dcc = vehicle_speed/Planning_Total_Time_; // Maximum decceleration for stopping at 3sec(planning total time) later

            bool collision = false;
            double travel_time = 0.0;
            double travel_distnace = 0.0;

            //Virtual Driving
            for(unsigned int j=0; j<planning_step;j++){

                // Ego Vehicle Pose Update
                double curvature = 0.0;
                double prev_vehicle_speed = vehicle_speed;

                VehiclePoseUpdate(vehicle_pose,new_vehicle_pose,vehicle_speed,curvature,PATHS_OFFSETs.at(i),Planning_Dt_,current_wp_idx,planning_mode);
                *vehicle_pose = *new_vehicle_pose;
                curv_pose->pose = vehicle_pose->pose;
                speed_pose->pose = vehicle_pose->pose;


                // I Changed code !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
                curv_pose->pose.position.z = curvature*1.0;
                // origin gain was curvature*1.0
                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

                if(dcc_G_ > MAX_dcc_G){
                    speed_pose->pose.position.z = 0.0;
                }
                else{
                    speed_pose->pose.position.z = vehicle_speed*3.6;
                    // if(planning_mode == 2){
                    //     speed_pose->pose.position.z *= 0.7;
                    // }
                    // else if(planning_mode == 3){
                    //     speed_pose->pose.position.z *= 0.5;
                    // }
                }

                if(Mission_State == 1)
                {
                    collision_margin_scale = 1.3;
                }
                if(Mission_State == 4)
                {
                    collision_margin_scale = 1;
                }
                collision = VehicleCollisionCheck.CollisionCheck(vehicle_pose,EgoVehiclePose,collision_margin_scale,obstacle_map,collision_check_map);


                if(collision){
                    collision_count++;
                    break;
                }
                else{
                    // Point add to Path
                    candidate_path->poses.push_back(*vehicle_pose);
                    candidate_curvature->poses.push_back(*curv_pose);
                    candidate_speed->poses.push_back(*speed_pose);
                    travel_distnace += prev_vehicle_speed*Planning_Dt_;   // visualizing the travel distance !!!!!!!!!!!!
                    travel_time+=Planning_Dt_;
//                    ROS_ERROR("travel_distance  :: %lf \n\n\n" ,travel_distnace);
                }

                if(current_wp_idx > reference_path_->waypoint.poses.size()-2){
                    // Reference path finish
//                    ROS_ERROR("current_wp_idx :: %d\n\n" , current_wp_idx);
//                    ROS_ERROR("reference_path_->waypoint.pose.size-2 :: %lu\n\n" , reference_path_->waypoint.poses.size()-2);

                    result = 3;
                    break;
                }

                if(vehicle_speed == 0.0){
                    break;
                }

                
            }

//            ROS_ERROR("init vehicle speed :: %lf " , init_vehicle_speed);
//            ROS_ERROR("Max dcc :: %lf " , MAX_DCC);
//            ROS_ERROR("able distance :: %lf \n\n\n\n\n" , able_distance);




            // printf("lateral avoiding: 1-2\n");

            //cost
            double cost;
            double opposite_direc_cost = 1.05;

            // if((PATHS_OFFSETs.at(old_path_id)*PATHS_OFFSETs.at(i))<0){ // opposite direction
            //     cost = fabs(PATHS_OFFSETs.at(i))+opposite_direc_cost*fabs(PATHS_OFFSETs.at(old_path_id)-PATHS_OFFSETs.at(i));
            // }
            // else{
            //     cost = fabs(PATHS_OFFSETs.at(i))+fabs(PATHS_OFFSETs.at(old_path_id)-PATHS_OFFSETs.at(i));
            // }
            cost = fabs(PATHS_OFFSETs.at(i));

            // printf("lateral avoiding: 1-3\n");

            virtual_path.collision = collision;
            virtual_path.path_offset = PATHS_OFFSETs.at(i);
            virtual_path.cost = cost;
            virtual_path.path = *candidate_path;
            virtual_path.path_curvature = *candidate_curvature; // **
            virtual_path.path_speed = *candidate_speed;         // **
            virtual_path.distance = travel_distnace;
            PATHS.push_back(virtual_path);

            if(tmp < travel_distnace)
            {
                tmp  = travel_distnace;
            }
            travel_distnace_ = tmp;
//            ROS_ERROR("travel distance :: %lf \n" , travel_distnace);
//            ROS_ERROR("travel distance tmp :: %lf \n" , travel_distnace_);
            // printf("lateral avoiding: 1-4\n");



        }

//        ROS_ERROR("travel distance :: %lf \n" , travel_distnace_);


        if(Mission_State == 4 /*&& (dcc_G_ > 1.0  || travel_distnace_ < 7)*/)
        {
            printf("we need a star !! \n");
//            ROS_ERROR("Astar condition travel distance :: %lf \n" , travel_distnace_);
            result = 5;
            break;
        }


        double able_distance = 0;
        able_distance = -pow(init_vehicle_speed,2)/(2*MAX_DCC);

//        ROS_ERROR("able distance :: %lf \n" ,able_distance);
//         if( travel_distnace_-2 < able_distance /*travel_distnace_ < 5*/ && Mission_State == 1)
//         {
//             printf("Can't stop by gateway dcc ! \n");

// //            ROS_ERROR("stop count :: %lf \n" ,stop_count);
//             stop_count++;



//             if(stop_count > 3 )
//             {
//                 ROS_ERROR("****** Emergency Stop !! ************\n");
//                 result = -2;
//                 stop_count = 0;
//                 break;
//             }
//         }

        if(travel_distnace_ > 5 && Mission_State == 1)
        {
            result = -3;
            break;
        }


        static bool old_collision_expectation = collision_expectation_;
        pharos_msgs::MotionCommandStamped3 motion_cmd_;

        std::cout << "collision_count: " << collision_count << std::endl;
        std::cout << "num_candidate_path: " << num_candidate_path << std::endl;
        if(collision_count == num_candidate_path){
            printf("No free path! Speed Down!\n");
//            ROS_ERROR("trvel distance :: %lf \n" , travel_distnace_);
            collision_expectation_ = true;

            result = 2;

            double dcc_incremental = 0.05;
            if(old_collision_expectation){
                dcc_G_ += dcc_incremental*G;
                printf("Need more speed down\n");

                if(travel_distnace_-2 < able_distance)
                {
                    printf("Can't stop by gateway dcc ! \n");
                    result = -2;
                    break;
                }
                if(dcc_G_ > MAX_dcc_G){     //// I changed original was :: MAX_dcc_G
                    printf("--- Stop! --- \n");
                    printf(" over than maximum gateway dcc ! \n");
                    result = -1;
                    old_collision_expectation=false;
                    break;
                }



            }
            else{
                unsigned int max_travel_path_id = 0;
                for(unsigned int i=0;i<num_candidate_path;i++){
                    if(PATHS.at(i).distance > PATHS.at(max_travel_path_id).distance){
                        max_travel_path_id = i;
                    }
                }
                double dist_to_collision = PATHS.at(max_travel_path_id).distance;
                dcc_G_ = -2.0*(dist_to_collision - init_vehicle_speed*Planning_Total_Time_)/pow(Planning_Total_Time_,2);
                dcc_G_ *= 2.0;
            }
            printf("decceleration: %.4f\n",dcc_G_);
        }
        else{
            collision_expectation_ = false;
            dcc_G_ = Init_dcc_G;
        }
        old_collision_expectation = collision_expectation_;

    }while(collision_expectation_);

    // printf("lateral avoiding: 2\n");

    //---- Virtual Path Visualization

    Candidate_Path_Visualize(PATHS);


    //---- Path Choice
    if(old_path_id > num_candidate_path-1){
        old_path_id = num_candidate_path/2;
    }
    unsigned int final_path_idx = 0;
    if(collision_count < num_candidate_path){
        final_path_idx = PathChoice(PATHS,old_path_id);
        *final_path_ = PATHS.at(final_path_idx).path;
        *final_path = PATHS.at(final_path_idx).path;
        *final_path_speed_ = PATHS.at(final_path_idx).path_speed;
        *final_path_curvature_ = PATHS.at(final_path_idx).path_curvature;
        path_id = final_path_idx;
        if(result == 0){
            result = 1;
        }
    }
    else{
        double max_travel_dist = 0.0;
        for(unsigned int i=0;i<PATHS.size();i++){
            if(PATHS.at(i).distance > max_travel_dist){
                max_travel_dist = PATHS.at(i).distance;
                final_path_idx = i;
            }
        }
        *final_path_ = PATHS.at(final_path_idx).path;
        *final_path = PATHS.at(final_path_idx).path;
        *final_path_speed_ = PATHS.at(final_path_idx).path_speed;
        *final_path_curvature_ = PATHS.at(final_path_idx).path_curvature;
        path_id = final_path_idx;
        if(result == 0){
            result = 2;
        }
    }


    if(result == -1){
        for(unsigned int i=0;i<final_path_speed_->poses.size();i++){
            final_path_speed_->poses.at(i).pose.position.z = 0.0;
        }
    }
    else if(result == 2){
        double path_dist = 0;
        for(unsigned int i=0;i<final_path_->poses.size()-1;i++){
            tf::Point p1,p2;
            double d = p1.distance(p2);
            path_dist += d;
        }
        if(path_dist < 30.0){
            for(unsigned int j=0;j<final_path_speed_->poses.size();j++){
            final_path_speed_->poses.at(j).pose.position.z = 0.0;
        }

        }
    }



    return result;
}


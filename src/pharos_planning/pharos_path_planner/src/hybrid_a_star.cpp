#include <hybrid_a_star.h>
//#include <lateral_avoiding.h>
#include <collision_check.h>

PathPlanning3D PP3D;
Lateral_Avoiding LA;
using namespace HYBRID_A_STAR_NS;
A_STAR_NODE AStarNode;
HEURISTICS Heuristics;
extern VehicleCollisionCheck VehicleCollisionCheck;


visualization_msgs::MarkerArrayPtr node_edge_array(new visualization_msgs::MarkerArray);
visualization_msgs::MarkerArrayPtr new_vehicle_pose_visualization_array(new visualization_msgs::MarkerArray);
visualization_msgs::MarkerArrayPtr new_node_pose_visualization_array(new visualization_msgs::MarkerArray);

visualization_msgs::MarkerPtr new_vehicle_pose_visualization(new visualization_msgs::Marker);
visualization_msgs::MarkerPtr new_node_pose_visualization(new visualization_msgs::Marker);
visualization_msgs::MarkerPtr node_edge(new visualization_msgs::Marker);
visualization_msgs::MarkerPtr goal_line(new visualization_msgs::Marker);
visualization_msgs::MarkerPtr goal_node_(new visualization_msgs::Marker);
visualization_msgs::MarkerPtr heuristic_node(new visualization_msgs::Marker);

nav_msgs::PathPtr a_star_path(new nav_msgs::Path);
nav_msgs::PathPtr a_star_speed(new nav_msgs::Path);
nav_msgs::PathPtr a_star_curvature(new nav_msgs::Path);

geometry_msgs::PoseStamped wp_pose;
geometry_msgs::PoseStamped speed_pose;
geometry_msgs::PoseStamped curv_pose;


int num_of_expension = 750; // original :: 150

double A_STAR_NODE::Node::cost() {
    return moving_cost+heuristic;
}



void HYBRID_A_STAR::HybridAStarPathPlanning(const Lateral_Avoiding::Reference_Path LA_reference_path,
                                            const pharos_road_information::LanePtr reference_lanes,
                                            const pharos_path_planner::ReferencePathPtr reference_path_,
                                            const geometry_msgs::PoseStampedConstPtr &InitVehiclePose,
                                            const tf::Point ego_vehicle_pose,
                                            const nav_msgs::OccupancyGridConstPtr &obstacleMap,
                                            const double vehicle_speed,
                                            const double goal_speed,
                                            const double dt,
                                            const unsigned int init_wp_idx,
                                            nav_msgs::OccupancyGridPtr &history_map,
                                            nav_msgs::PathPtr &path_solution) {

    node_edge_array->markers.clear();
    new_vehicle_pose_visualization_array->markers.clear();
    new_node_pose_visualization_array->markers.clear();
    new_vehicle_pose_visualization->colors.clear();
    new_vehicle_pose_visualization->points.clear();
    new_node_pose_visualization->colors.clear();
    new_node_pose_visualization->points.clear();
    node_edge->colors.clear();
    node_edge->points.clear();
    goal_line->colors.clear();
    goal_line->points.clear();
    goal_node_->colors.clear();
    goal_node_->points.clear();
    heuristic_node->colors.clear();
    heuristic_node->points.clear();



    ros::Time t0 = ros::Time::now();
    path_solution->header.frame_id = "odom";
    path_solution->header.stamp = obstacleMap->header.stamp;

    //printf("A star planning start\n");
    double new_vehicle_speed = 20.0 / 3.6;

    if (vehicle_speed < 7.0) {
        new_vehicle_speed = 7.0;
    } else if (vehicle_speed > 15.0) {
        new_vehicle_speed = 15.0;
    } else {
        new_vehicle_speed = vehicle_speed;
    }

    // Initialization
    A_STAR_NODE *start_node(new A_STAR_NODE);

    // sorting by std::multiset
    std::multiset<A_STAR_NODE> *OpenNodeSet(new std::multiset<A_STAR_NODE>);
    std::multiset<A_STAR_NODE> *ClosedNodeSet(new std::multiset<A_STAR_NODE>);


    double Planning_distance = HybridAstarPlanningDist;
    unsigned int goal_wp_idx2 = 0;
    double p_d = 0.0;
    unsigned int wp_iter = init_wp_idx;

    a_star_path->poses.clear();
    a_star_curvature->poses.clear();
    a_star_speed->poses.clear();

    while (p_d < Planning_distance) {

        unsigned long path_size = reference_path_->roadinfo.size() - 2;

        double x1 = reference_path_->roadinfo.at(wp_iter).position.x;
        double y1 = reference_path_->roadinfo.at(wp_iter).position.y;
        double x2 = reference_path_->roadinfo.at((wp_iter + 1) % path_size).position.x; // % is for don't floting the path_size
        double y2 = reference_path_->roadinfo.at((wp_iter + 1) % path_size).position.y; // % is for don't floting the path_size

        wp_pose.pose.position.x = reference_path_->roadinfo.at(wp_iter).position.x;
        wp_pose.pose.position.y = reference_path_->roadinfo.at(wp_iter).position.y;
        wp_pose.pose.position.z = reference_path_->roadinfo.at(wp_iter).position.z;

        speed_pose.pose.position.x = reference_path_->roadinfo.at(wp_iter).position.x;
        speed_pose.pose.position.y = reference_path_->roadinfo.at(wp_iter).position.y;
        speed_pose.pose.position.z = reference_path_->roadinfo.at(wp_iter).reference_speed;
//        speed_pose.pose.position.z = 10;     //vehicle_speed*3.6;;



        curv_pose.pose.position.x = reference_path_->roadinfo.at(wp_iter).position.x;
        curv_pose.pose.position.y = reference_path_->roadinfo.at(wp_iter).position.y;
        curv_pose.pose.position.z = reference_path_->roadinfo.at(wp_iter).curvature;

        a_star_path->poses.push_back(speed_pose);  // i put it in speed pose
        a_star_speed->poses.push_back(speed_pose);
        a_star_curvature->poses.push_back(curv_pose);

        p_d += sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
        wp_iter++;
        wp_iter = (wp_iter) % path_size;
    }

    goal_wp_idx2 = (a_star_path->poses.size() - 0.2 * a_star_path->poses.size());


    a_star_path_pub_.publish(a_star_path);


    A_STAR_NODE *goal_node(new A_STAR_NODE);
    goal_node->node.pose.x = a_star_path->poses.at(goal_wp_idx2).pose.position.x;
    goal_node->node.pose.y = a_star_path->poses.at(goal_wp_idx2).pose.position.y;
    goal_node->node.pose.z = a_star_path->poses.at(goal_wp_idx2).pose.position.z;
    goal_node->node.pose.heading = PP3D.PathHeading(goal_wp_idx2 - 1, a_star_path);
    goal_node->node.wp_idx = goal_wp_idx2;                  //a_star_path->poses.size()-2;
    geometry_msgs::PoseStampedPtr goal_pose(new geometry_msgs::PoseStamped);
    AStarNode.NodePoseToPoseStamped(goal_node->node.pose, goal_pose);


    //// this line is if collision checked  , then goal pose get posed far away
//    while(VehicleCollisionCheck.CollisionCheck(goal_pose, ego_vehicle_pose, p_d, obstacleMap, history_map)) {
//        if (goal_node->node.wp_idx > a_star_path->poses.size() - 2) break;
//
//        goal_node->node.wp_idx++;
//        goal_node->node.pose.x = a_star_path->poses.at((goal_node->node.wp_idx % (a_star_path->poses.size()))).pose.position.x;
//        goal_node->node.pose.y = a_star_path->poses.at((goal_node->node.wp_idx % (a_star_path->poses.size()))).pose.position.y;
//
//        goal_node->node.pose.heading = PP3D.PathHeading(goal_node->node.wp_idx, a_star_path);
//        AStarNode.NodePoseToPoseStamped(goal_node->node.pose, goal_pose);
//    }

//    ROS_ERROR("collision check \n");
    goal_line->header.stamp = ros::Time(0);
    goal_node_->header.stamp = ros::Time::now();

    //goal_line->action = visualization_msgs::Marker::ADD;
    goal_line->points.clear();
    goal_node_->points.clear();

    geometry_msgs::Point gp1, gp2;
    geometry_msgs::Point gp3;
    double goal_line_distance = 5.0;
    double goal_heading = PP3D.PathHeading(goal_node->node.wp_idx, a_star_path);
    double goal_x = goal_node->node.pose.x;
    double goal_y = goal_node->node.pose.y;

    gp1.x = goal_x + (goal_line_distance / 2.0) * cos(goal_heading + M_PI / 2.0);
    gp1.y = goal_y + (goal_line_distance / 2.0) * sin(goal_heading + M_PI / 2.0);
    gp1.z = 0.0;
    gp2.x = goal_x + (goal_line_distance / 2.0) * cos(goal_heading - M_PI / 2.0);
    gp2.y = goal_y + (goal_line_distance / 2.0) * sin(goal_heading - M_PI / 2.0);
    gp2.z = 0.0;

    gp3.x = goal_node->node.pose.x;
    gp3.y = goal_node->node.pose.y;
    gp3.z = 0.0;

    goal_line->points.push_back(gp1);
    goal_line->points.push_back(gp2);
    goal_node_->points.push_back(gp3);


    goal_node_marker.publish(goal_node_);
    a_star_goal_marker_pub_.publish(goal_line);

    A_STAR_NODE::Pose start_node_pose;
    AStarNode.PoseStampedToNodePose(InitVehiclePose, start_node_pose);

    int node_id = 0;
    start_node->node.pose = start_node_pose;
    start_node->node.wp_idx = 0;    //originaly it was init_wp_idx
    start_node->node.moving_cost = 0.0;
    start_node->node.heuristic = Heuristics.EuclidDist(*start_node, *goal_node);
    start_node->node.id = node_id;
    start_node->node.parent = NULL;


    //OpenNodes.push_back(*start_node);
    OpenNodeSet->insert(*start_node);
    //printf("A star planning initialized\n");

    geometry_msgs::PoseStampedPtr vehicle_pose(new geometry_msgs::PoseStamped);
    geometry_msgs::PoseStampedPtr new_vehicle_pose(new geometry_msgs::PoseStamped);

    double max_lat_acc_limit = 0.65 * 9.81;
    double max_steering_angle = AStarNode.MaxSteeringAngle(new_vehicle_speed, max_lat_acc_limit);

    double dt_heading = new_vehicle_speed * dt / VehicleWheelBase * tan(max_steering_angle);
    double node_dx = new_vehicle_speed * dt * cos(dt_heading);
    double node_dy = new_vehicle_speed * dt * sin(dt_heading);
    double dt_distance = sqrt(pow(node_dx, 2) + pow(node_dy, 2));

    // Planning
    bool success = false;
    int iterator = 0;


//    printf("A star Search Start\n");

    node_edge->header.stamp = ros::Time(0);  // visualization
    //node_edge->action = visualization_msgs::Marker::ADD;
    node_edge->points.clear();
    node_edge->colors.clear();
    new_vehicle_pose_visualization->points.clear();
    new_node_pose_visualization->points.clear();

    A_STAR_NODE *end_node(new A_STAR_NODE);

    while (!success) {

        double current_vehicle_speed = vehicle_speed;
        double a_star_path_speed = vehicle_speed;
        double desired_vehicle_speed = goal_speed;

//        ROS_ERROR("opennodeset size :: %d \n", OpenNodeSet->size());
//        ROS_ERROR("inside of success while loop \n");
        if (OpenNodeSet->empty()) {
//            printf("[Hybrid A star] OpenNodeSet is empty!\n");
            break;
        }

        //node select in open node list
        A_STAR_NODE *current(new A_STAR_NODE);

        std::set<A_STAR_NODE>::iterator open_iter = OpenNodeSet->begin();

////        ROS_ERROR("*******open_node list start******************\n");
////        for(open_iter = OpenNodeSet->begin() ; open_iter != OpenNodeSet->end(); open_iter++)
////        {
////            printf("\n");
////            printf("(*open_iter).node.wp_idx :: %d \n " , (*open_iter).node.wp_idx );
////            printf("(*open_iter).node.id :: %d \n" , (*open_iter).node.id);
////            printf("open_iter.node.pose ::  x ::  %lf ,  y :: %lf   \n" , (*open_iter).node.pose.x , (*open_iter).node.pose.y);
////            printf("open_iter.node.speed :: %lf \n" , (*open_iter).node.pose.z);
////           printf("\n");
////        }
////        ROS_ERROR("*******open_node list end!!******************\n");
////
////        ROS_ERROR("OpenNodeset begin node idx , id , x , y  :: %d , %d, %lf , %lf \n ", OpenNodeSet->begin()->node.wp_idx, OpenNodeSet->begin()->node.id ,OpenNodeSet->begin()->node.pose.x, OpenNodeSet->begin()->node.pose.y);


        *current = *(OpenNodeSet->begin());    //// algorithm list 1 2
        OpenNodeSet->erase(OpenNodeSet->begin());
        ClosedNodeSet->insert(*current);
        //printf("current: x: %.3f, y: %.3f\n",current->node.pose.x,current->node.pose.y);



        //node expansion with bicycle model
        for (int i = 0; i < 3; i++) {

            //----1.  New node position update

            A_STAR_NODE *new_node(new A_STAR_NODE);

            new_node->node.parent = current;
            AStarNode.NodePoseToPoseStamped(new_node->node.parent->node.pose, vehicle_pose);

            node_id++;

            //// controling max steering for a star nodes  (avoiding  right side && decline right side angle gain)
            double steering_angle = -((i-1)*35*M_PI/180);         //double(i-1)*max_steering_angle;
//            if(i>0)
//            {
//                steering_angle = 0.75*max_steering_angle;
//            }


            AStarNode.BicycleModelMotionUpdate2(vehicle_pose, new_vehicle_pose, steering_angle, new_vehicle_speed, dt);


            A_STAR_NODE::Pose new_node_pose;
            AStarNode.PoseStampedToNodePose(new_vehicle_pose, new_node_pose);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            //----2.  Closed Check

            std::set<A_STAR_NODE>::iterator closed_iter = ClosedNodeSet->begin();
            bool closed_check = false;
            double node_distance = 0.0;
            double heading_err = 0.0;
            int closed_iter2 = 0;
            unsigned int closed_node_id = 0;

//            ROS_ERROR("***** closed node list start *****");
            while (closed_iter != ClosedNodeSet->end()) {

                if(new_node->node.parent == closed_iter->node.parent){
                    continue;
                }

                double new_x = new_node_pose.x;
                double new_y = new_node_pose.y;
                double new_heading = new_node_pose.heading;

                double closed_x = closed_iter->node.pose.x;
                double closed_y = closed_iter->node.pose.y;
                double closed_heading = closed_iter->node.pose.heading;

                double dist_gain = new_node->node.parent->node.moving_cost / 100.0;

                double closed_dist_margin = 3.0 * node_dy; // need explain      //
                double closed_angle_margin = 5.0 * dt_heading; // need explain  //

//                printf("closed dist  margin : %lf \n" , closed_dist_margin);
//                printf("closed angle margin : %lf \n" , closed_angle_margin);

//                double closed_dist_margin = 0.15; // need explain      //
//                double closed_angle_margin = 20.0*M_PI/180.0; // need explain  //

                node_distance = sqrt(pow(new_x - closed_x, 2) + pow(new_y - closed_y,2)); // I think    new : current       closed : just before current
                heading_err = fabs(new_heading - closed_heading);
                if (heading_err > M_PI) {
                    heading_err -= 2 * M_PI;
                }
                heading_err = fabs(heading_err);        //// heading error absolute value
                //printf("closed_iter: %d, dist_error: %.4f, heading_error: %.4f\n", closed_iter2, node_distance, heading_err);
                if (node_distance < closed_dist_margin && heading_err <closed_angle_margin) {    //// node distance :: ucleadian distance between current node and before node
                    closed_check = true;
                    closed_node_id = (*closed_iter).node.id;
                    //printf("This node is closed\n");
                    break;
                }

////                printf("\n");
////                printf("(*closed_iter).node.wp_idx :: %d \n " , (*closed_iter).node.wp_idx );
////                printf("(*closed_iter).node.id :: %d \n" , (*closed_iter).node.id);
////               printf("closed_iter.node.pose ::  x ::  %lf ,  y :: %lf   \n" , (*closed_iter).node.pose.x , (*closed_iter).node.pose.y);
////                printf("\n");

                closed_iter++;
                closed_iter2++;
            }
//            ROS_ERROR("closed node heading error ! %.4f \n" , heading_err);

////            ROS_ERROR("***** closed node list end *****");

            if (closed_check) {
//                printf("iter:%d(%d), This node is closed with %d (dist: %.3f, heading: %.3f,parent_node: %d)\n",iterator,new_node->node.id,closed_node_id,node_distance,heading_err*180.0/M_PI,new_node->node.parent->node.id);
                continue;
            }


            // Similar open node check
            if(!OpenNodeSet->empty()){
                std::set<A_STAR_NODE>::iterator opened_iter = OpenNodeSet->begin();
                bool opened_check = false;
                double node_distance_o = 0.0;
                double heading_err_o = 0.0;


                while (opened_iter != OpenNodeSet->end()) {

                    double new_x = new_node_pose.x;
                    double new_y = new_node_pose.y;
                    double new_heading = new_node_pose.heading;

                    double closed_x = opened_iter->node.pose.x;
                    double closed_y = opened_iter->node.pose.y;
                    double closed_heading = opened_iter->node.pose.heading;

                    double dist_gain = new_node->node.parent->node.moving_cost / 100.0;

                    double closed_dist_margin = 3.0 * node_dy; // need explain      //
                    double closed_angle_margin = 5.0 * dt_heading; // need explain  //

//                printf("closed dist  margin : %lf \n" , closed_dist_margin);
//                printf("closed angle margin : %lf \n" , closed_angle_margin);

//                double closed_dist_margin = 0.15; // need explain      //
//                double closed_angle_margin = 20.0*M_PI/180.0; // need explain  //

                    node_distance_o = sqrt(pow(new_x - closed_x, 2) + pow(new_y - closed_y,2)); // I think    new : current       closed : just before current
                    heading_err_o = fabs(new_heading - closed_heading);
                    if (heading_err_o > M_PI) {
                        heading_err_o -= 2 * M_PI;
                    }
                    heading_err_o = fabs(heading_err_o);        //// heading error absolute value
                    //printf("closed_iter: %d, dist_error: %.4f, heading_error: %.4f\n", closed_iter2, node_distance, heading_err);
                    if (node_distance_o < closed_dist_margin && heading_err_o <closed_angle_margin) {    //// node distance :: ucleadian distance between current node and before node
                        opened_check = true;
                        closed_node_id = (*opened_iter).node.id;
                        //printf("This node is closed\n");
                        break;
                    }

////                printf("\n");
////                printf("(*closed_iter).node.wp_idx :: %d \n " , (*closed_iter).node.wp_idx );
////                printf("(*closed_iter).node.id :: %d \n" , (*closed_iter).node.id);
////               printf("closed_iter.node.pose ::  x ::  %lf ,  y :: %lf   \n" , (*closed_iter).node.pose.x , (*closed_iter).node.pose.y);
////                printf("\n");

                    opened_iter++;
                }
                if (opened_check) {
//                printf("iter:%d(%d), This node is closed with %d (dist: %.3f, heading: %.3f,parent_node: %d)\n",iterator,new_node->node.id,closed_node_id,node_distance,heading_err*180.0/M_PI,new_node->node.parent->node.id);
                    continue;
                }


            }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


            //----3.  Collision Check
            bool collision = false;
            double travel_dist = new_node->node.parent->node.moving_cost + vehicle_speed * dt;
            double collision_scale = 1.37;
            collision = VehicleCollisionCheck.CollisionCheck(new_vehicle_pose, ego_vehicle_pose, collision_scale ,obstacleMap, history_map);  //// we need collision back


            if (collision) {
////                printf("*****start of collision checked !!***** \n");
////                printf("start node :: x :: %lf , y :: %lf  \n", start_node->node.pose.x, start_node->node.pose.y);
////                printf("\n");
////
////                printf("new node idx :: %d \n", new_node->node.wp_idx);
////                printf("new_node pose    x :: %lf ,   y :: %lf   \n", new_node->node.pose.x, new_node->node.pose.y);
////                printf("\n");
////
////                printf("iter:%d, This node has collision, parent: %d\n", iterator, new_node->node.parent->node.id);
////                printf("Collision node idx :: %d   id :: %d \n", new_node->node.wp_idx, new_node->node.id);
////                printf("Collision node position  x :: %lf , y :: %lf \n", new_node->node.pose.x, new_node->node.pose.y);
////                printf("*****end of collision checked !!***** \n");
////                printf("\n");

                continue;
            }else{
//                ROS_ERROR("there is no collision in this loop !! \n");
            }






//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //----4.  Heuristic and Moving Cost Update

            //printf("path_heading: %.4f, init_heading: %.4f, node_heading: %.4f\n",PP3D.PathHeading(new_node->node.wp_idx,waypoint_),start_node_pose.heading,new_node_pose.heading);

            new_node->node.wp_idx = AStarNode.NearestWaypointIdx2(new_vehicle_pose, new_node->node.parent->node.wp_idx,
                                                                  50, a_star_path);    // 10 is waypoint search number
            new_node->node.pose = new_node_pose;
            new_node->node.id = node_id;
            new_node->node.moving_cost = AStarNode.MovingCost(*new_node);
            //new_node->node.moving_cost = AStarNode.MovingCost_Lateral(*new_node);
            new_node->node.heuristic = Heuristics.EuclidDist_Angle(*new_node,*goal_node);

//            new_node->node.heuristic = Heuristics.VirtualPathFollowing(*new_node, *goal_node, new_vehicle_speed, 0.1);
//            new_node->node.heuristic = Heuristics.EuclidDist_Lateraloffset(*new_node,*goal_node);
//            std::cout <<" new node " << new_node->node.wp_idx<< std::endl;
//            std::cout <<" goal node " << goal_node->node.wp_idx << std::endl;
//            std::cout <<" new_vehicle_speed " <<new_vehicle_speed << std::endl;
//            printf("new node's parent idx :: %d \n", new_node->node.parent->node.wp_idx);
//
//            printf("new node parent id :: %d \n",new_node->node.parent->node.id);
//            printf("new node id :: %d \n",new_node->node.id);
//            printf("start_node: x: %.3f, y: %.3f\n",start_node_pose.x,start_node_pose.y);
//
//            printf("new node pose x :: %lf \n" , new_node->node.pose.x);
//            printf("new node pose y :: %lf \n" , new_node->node.pose.y);

            //////////////////////////////////////////////////////////////////////////////////////////////////


            if (new_node->node.wp_idx > goal_wp_idx2 - 10) {

                *end_node = *new_node;
                success = true;
//                printf("[Hybrid A star] Success! \n");
                break;
            }

            geometry_msgs::Point p1, p2;

            p1.x = new_node->node.parent->node.pose.x;
            p1.y = new_node->node.parent->node.pose.y;
            p1.z = 0.0;
            p2.x = new_node->node.pose.x;
            p2.y = new_node->node.pose.y;
            p2.z = 0.0;

            node_edge->points.push_back(p1);
            node_edge->points.push_back(p2);

            geometry_msgs::Point p3;
            p3.x = new_vehicle_pose->pose.position.x;
            p3.y = new_vehicle_pose->pose.position.y;
            new_vehicle_pose_visualization->points.push_back(p3);

            geometry_msgs::Point p4;
            p4.x = new_node_pose.x;
            p4.y = new_node_pose.y;
            new_node_pose_visualization->points.push_back(p4);

            OpenNodeSet->insert(*new_node);


            collision_scale = 1.0;
        }   // end of for(int i = 0; i< 3 ; i++)


        //printf("Astar iter: %d\n",iterator);
        if (iterator > num_of_expension) {   // expension means bigger and bigger
            printf("[Hybrid A star] Fail to find a path within %d iterations\n", num_of_expension);
            break;
        }
        iterator++;
        //success = true;
    }


    //// node_edge  ::  visualization
    node_edge->colors.clear();
    new_node_pose_visualization->colors.clear();

    std_msgs::ColorRGBA c;
    std_msgs::ColorRGBA c2;


    for (int i = 0; i < node_edge->points.size(); i++) {
        c.r = 1.0;
        c.g = 1.0 - float(i) / node_edge->points.size();
        c.b = 1.0 - float(i) / node_edge->points.size();
        node_edge->colors.push_back(c);
    }
    if (node_edge->colors.size() > 2) {
        node_edge->colors.at(node_edge->colors.size() - 2).r = 0.0;
        node_edge->colors.at(node_edge->colors.size() - 2).g = 0.0;
        node_edge->colors.at(node_edge->colors.size() - 2).b = 1.0;

        node_edge->colors.at(node_edge->colors.size() - 1).r = 0.0;
        node_edge->colors.at(node_edge->colors.size() - 1).g = 0.0;
        node_edge->colors.at(node_edge->colors.size() - 1).b = 1.0;
    }

    for (int i = 0; i < new_node_pose_visualization->points.size(); i++) {

        c2.r = 1.0 - float(i) / new_node_pose_visualization->points.size();
        c2.g = 1.0;
        c2.b = 1.0- float(i) / new_node_pose_visualization->points.size();
        c2.a = 1.0;
        new_node_pose_visualization->colors.push_back(c2);
    }
    if(new_node_pose_visualization->colors.size() > 2)
    {
        new_node_pose_visualization->colors.at(new_node_pose_visualization->colors.size() - 2).r = 0.0;
        new_node_pose_visualization->colors.at(new_node_pose_visualization->colors.size() - 2).g = 0.0;
        new_node_pose_visualization->colors.at(new_node_pose_visualization->colors.size() - 2).b = 1.0;

        new_node_pose_visualization->colors.at(new_node_pose_visualization->colors.size() - 1).r = 0.0;
        new_node_pose_visualization->colors.at(new_node_pose_visualization->colors.size() - 1).g = 0.0;
        new_node_pose_visualization->colors.at(new_node_pose_visualization->colors.size() - 1).b = 1.0;
    }

    node_edge_array->markers.push_back(*node_edge);
    new_vehicle_pose_visualization_array->markers.push_back(*new_vehicle_pose_visualization);
    new_node_pose_visualization_array->markers.push_back(*new_node_pose_visualization);

    new_vehicle_pose_pub_.publish(new_vehicle_pose_visualization_array);
    a_star_node_pub_.publish(node_edge_array);
    new_node_pose_pub_.publish(new_node_pose_visualization_array);

    std::multiset<A_STAR_NODE>::iterator iter2 = ClosedNodeSet->begin();
//    while(iter2 != ClosedNodeSet->end()){
//        printf("x:%.3f, y:%.3f, yaw:%.3f g:%.3f, h: %.3f\n",(*iter2).node.pose.x,(*iter2).node.pose.y,(*iter2).node.pose.heading,(*iter2).node.moving_cost,(*iter2).node.heuristic);
//        iter2++;
//    }

    ros::Time t_end = ros::Time::now();
    double Dt = (t_end-t0).toSec();
    static double max_dt = 0.0;
//        if(Dt>max_dt){
//            max_dt = Dt;
//            ROS_INFO("Max Processing Time: %.4f",Dt);
//        }
    printf("A_Star Iteration: %d, Processing Time: %.3f, Open: %d, Closed: %d\n",iterator,Dt,OpenNodeSet->size(),ClosedNodeSet->size());

    int path_refine_iter = 0;

    nav_msgs::PathPtr init_solution(new nav_msgs::Path);

    if(success){
//        ROS_ERROR("Hybird a star is successed !! \n");
        geometry_msgs::PoseStampedPtr path_pose(new geometry_msgs::PoseStamped);
        A_STAR_NODE *path_node(new A_STAR_NODE);
        *path_node = *end_node;


//        printf(" end_node->node.pose x :: %lf \n" , end_node->node.pose.x);
//        printf(" end_node->node.pose y :: %lf \n" , end_node->node.pose.y);

        while(path_node->node.parent != NULL){

            AStarNode.NodePoseToPoseStamped(path_node->node.parent->node.pose,path_pose);
            init_solution->poses.insert(init_solution->poses.begin(),*path_pose);
            *path_node = *(path_node->node.parent);

            //printf("Path Refinement iter: %d\n",path_refine_iter);
            path_refine_iter++;
        }
//        PP3D.B_spline(init_solution,path_solution);

//        std::cout << "init_solution \n " << *init_solution << std::endl;


        path_solution = init_solution;
//        ROS_ERROR("path solution speed : %lf \n\n" , path_solution->poses.at(0).pose.position.z);

        //        for(unsigned int i=0;i<path_solution->poses.size();i++){
//            printf("x: %.3f, y:%.3f\n",path_solution->poses.at(i).pose.position.x,path_solution->poses.at(i).pose.position.y);
//        }
//        printf("B spline finish!\n");
        //*path_solution = *init_solution;
    }
    else{
        path_solution->poses.clear();
    }


//    ROS_ERROR("Hybrid A star end !!  \n");


}



inline double HEURISTICS::EuclidDist(const A_STAR_NODE &current_node, const A_STAR_NODE &goal_node){
    double distance = sqrt(pow(goal_node.node.pose.x-current_node.node.pose.x,2)+pow(goal_node.node.pose.y-current_node.node.pose.y,2));
    return distance;
}

double HEURISTICS::EuclidDist_Angle(const A_STAR_NODE &current_node, const A_STAR_NODE &goal_node){
    double distance = sqrt(pow(goal_node.node.pose.x-current_node.node.pose.x,2)+pow(goal_node.node.pose.y-current_node.node.pose.y,2));
    double angle_error = fabs(goal_node.node.pose.heading - current_node.node.pose.heading);
    if (angle_error > M_PI) angle_error = fabs(angle_error - 2 * M_PI);
    return distance + angle_error;
}

double HEURISTICS::EuclidDist_Lateraloffset(const A_STAR_NODE &current_node, const A_STAR_NODE &goal_node){
    double distance = sqrt(pow(goal_node.node.pose.x-current_node.node.pose.x,2)+pow(goal_node.node.pose.y-current_node.node.pose.y,2));


    geometry_msgs::PoseStampedPtr current_node_pose(new geometry_msgs::PoseStamped);
    AStarNode.NodePoseToPoseStamped(current_node.node.pose,current_node_pose);
    double lateral_offset = fabs(LA.LateralOffset(current_node.node.wp_idx,current_node_pose));  //// lateraloffset function wants wp_idx & vehicle pose but why current_node_pose is in the vehicle pose's place?

    return distance+ 0.4*lateral_offset;
}

double HEURISTICS::VirtualPathFollowing(const A_STAR_NODE &current_node, const A_STAR_NODE &goal_node, const double speed, double dt) {
    double distance = 0.0;
    double steering_moving = 0.0;
    double lateral_moving = 0.0;
    A_STAR_NODE *updated_node(new A_STAR_NODE);  //// current node
    *updated_node = current_node;
    double Error = 0.0;
    double lateral_offset = DBL_MAX;
    double angle_difference = DBL_MAX;

    double lateral_origin = DBL_MAX;
    double angle_origin = DBL_MAX;
    double init_dt = dt;

//    ROS_ERROR("inside of VirtualPathFollowing !!! \n");

    geometry_msgs::PoseStampedPtr init_node_pose(new geometry_msgs::PoseStamped);
    AStarNode.NodePoseToPoseStamped(updated_node->node.pose,init_node_pose);

    double init_lateral_offset = fabs(AStarNode.LateralOffset2(current_node.node.wp_idx,init_node_pose, a_star_path));

    heuristic_node->points.clear();  //// visualization
    geometry_msgs::Point p1;
    p1.x = updated_node->node.pose.x;
    p1.y = updated_node->node.pose.y;
    p1.z = 0.0;
    heuristic_node->points.push_back(p1);

    //while(fabs(lateral_origin) > 0.3 || fabs(angle_origin*180.0/M_PI)> 5.0 ){
    while(updated_node->node.wp_idx < goal_node.node.wp_idx || fabs(lateral_origin) > 0.5 || fabs(angle_origin*180.0/M_PI) > 15.0 ){
        //while(updated_node->node.wp_idx<goal_node.node.wp_idx){
        //lateral offset
        geometry_msgs::PoseStampedPtr node_pose(new geometry_msgs::PoseStamped);
        geometry_msgs::PoseStampedPtr lookahead_node_pose(new geometry_msgs::PoseStamped);

//        ROS_ERROR("in virtual driving A star Path size :: %d \n" , a_star_path->poses.size());

        AStarNode.NodePoseToPoseStamped(updated_node->node.pose,node_pose);
        *lookahead_node_pose = *node_pose;
        lookahead_node_pose->pose.position.x += 3.3*cos(updated_node->node.pose.heading);  //// lookahead node  original : 1.5*cos & sin
        lookahead_node_pose->pose.position.y += 3.3*sin(updated_node->node.pose.heading);
        int lookahead_idx = AStarNode.NearestWaypointIdx2(lookahead_node_pose,updated_node->node.wp_idx,5,a_star_path);

        lateral_offset = AStarNode.LateralOffset2(lookahead_idx,lookahead_node_pose,a_star_path);
        lateral_origin = AStarNode.LateralOffset2(updated_node->node.wp_idx,node_pose,a_star_path);
        //angle difference
        angle_difference = AStarNode.AngleDifference2(lookahead_idx,lookahead_node_pose,a_star_path);
        angle_origin = AStarNode.AngleDifference2(updated_node->node.wp_idx,node_pose,a_star_path);
//        printf("[   origin  ] lateral: %.3f, angle_diff: %.3f\n",lateral_origin,angle_origin*180.0/M_PI);
//        printf("[ lookahead ] lateral: %.3f, angle_diff: %.3f\n",lateral_offset,angle_difference*180.0/M_PI);

        //curvuatre
        double curvature = a_star_curvature->poses.at(lookahead_idx).pose.position.z;

        //steering control
        double wheel_angle = AStarNode.SteeringControl(angle_difference,lateral_offset,curvature,speed);

        //pose update
        double beta = speed*dt/VehicleWheelBase*tan(wheel_angle);
        updated_node->node.pose.heading += beta;
        double dx = speed*dt * cos(updated_node->node.pose.heading);
        double dy = speed*dt * sin(updated_node->node.pose.heading);


        updated_node->node.pose.x += dx;
        updated_node->node.pose.y += dy;

        geometry_msgs::PoseStampedPtr updated_node_pose(new geometry_msgs::PoseStamped);

        updated_node_pose->pose.position.x = updated_node->node.pose.x;
        updated_node_pose->pose.position.y = updated_node->node.pose.y;
        updated_node_pose->pose.position.z = updated_node->node.pose.z;
        updated_node_pose->pose.orientation.w = 1.0;
        updated_node->node.wp_idx = AStarNode.NearestWaypointIdx2(updated_node_pose,updated_node->node.wp_idx,20,a_star_path);


//        p1.x = updated_node->node.pose.x;
//        p1.y = updated_node->node.pose.y;
//        p1.z = 0.0;
//        heuristic_node->points.push_back(p1);

        distance += dt*speed;
        steering_moving += beta;
        lateral_moving += fabs(lateral_origin);

        //multi resolution
        tf::Point goal_wp_pt, node_pose_pt;
        goal_wp_pt.setValue(a_star_path->poses.at(goal_node.node.wp_idx).pose.position.x,a_star_path->poses.at(goal_node.node.wp_idx).pose.position.y,0.0);
        node_pose_pt.setValue(updated_node->node.pose.x,updated_node->node.pose.y,0.0);
        double remain_dist = goal_wp_pt.distance(node_pose_pt);

        if(dt==init_dt && remain_dist < 3.0){
            dt = 0.02;
        }
//        if(dt==init_dt && fabs(lateral_offset)<0.35){
//            dt = 0.02;
//        }
    }

//    heuristic_node->header.stamp = ros::Time(0);
//    a_star_heuristic_pub_.publish(heuristic_node);

    tf::Point near_wp_pt, node_pose_pt2;
    near_wp_pt.setValue(a_star_path->poses.at(updated_node->node.wp_idx).pose.position.x,a_star_path->poses.at(updated_node->node.wp_idx).pose.position.y,0.0);
    node_pose_pt2.setValue(updated_node->node.pose.x,updated_node->node.pose.y,0.0);

    double extra_dist = near_wp_pt.distance(node_pose_pt2);

    double x0 = near_wp_pt.x();
    double y0 = near_wp_pt.y();

    double x1, y1;
    double theta = updated_node->node.pose.heading;

    x0 -= node_pose_pt2.x();
    y0 -= node_pose_pt2.y();

    x1 = x0*cos(-theta) - y0*sin(-theta);
    y1 = x0*sin(-theta) - y0*cos(-theta);

    distance += x1;
    steering_moving = fabs(steering_moving);
//    ROS_ERROR("end of virtual path planning !!! \n");
    return 0.2*init_lateral_offset+distance;
}

inline double A_STAR_NODE::MaxSteeringAngle(const double vehicle_speed, const double LatAccLimit) {
    double turning_radius = pow(vehicle_speed,2)/LatAccLimit;
    double steering_angle;
    steering_angle = VehicleWheelBase/turning_radius;
    return steering_angle;
}

inline void A_STAR_NODE::NodeVisualizationMarkerInit(){
    node_edge->header.frame_id = "odom";
    node_edge->header.stamp = ros::Time::now();
    node_edge->ns = "A_star_nodes";
    node_edge->id = 0;
    node_edge->type = visualization_msgs::Marker::LINE_LIST;
    node_edge->action = visualization_msgs::Marker::ADD;
    node_edge->pose.orientation.x = 0.0;
    node_edge->pose.orientation.y = 0.0;
    node_edge->pose.orientation.z = 0.0;
    node_edge->pose.orientation.w = 1.0;
    node_edge->scale.x = 0.05; //Line width
    node_edge->scale.y = 0.0;
    node_edge->scale.z = 0.0;
    node_edge->color.a = 1.0;


//    node_edge->color.r = 1.0;
//    node_edge->color.g = 0.5;
//    node_edge->color.b = 0.0;

    goal_line->header.frame_id = "odom";
    goal_line->ns = "A_star_Goal_line";
    goal_line->id = 0;
    goal_line->type = visualization_msgs::Marker::LINE_LIST;
    goal_line->action = visualization_msgs::Marker::ADD;
    goal_line->pose.orientation.x = 0.0;
    goal_line->pose.orientation.y = 0.0;
    goal_line->pose.orientation.z = 0.0;
    goal_line->pose.orientation.w = 1.0;
    goal_line->scale.x = 0.1; //Line width
    goal_line->scale.y = 0.0;
    goal_line->scale.z = 0.0;
    goal_line->color.a = 1.0;
    goal_line->color.r = 1.0;
    goal_line->color.g = 0.0;
    goal_line->color.b = 0.0;


    goal_node_->header.frame_id = "odom";
    goal_node_->ns = "A_star_goal_node";
    goal_node_->id = 0;
    goal_node_->type = visualization_msgs::Marker::SPHERE_LIST;
    goal_node_->action = visualization_msgs::Marker::ADD;
    goal_node_->pose.orientation.x = 0.0;
    goal_node_->pose.orientation.y = 0.0;
    goal_node_->pose.orientation.z = 0.0;
    goal_node_->pose.orientation.w = 1.0;
    goal_node_->scale.x = 1.1; //Line width
    goal_node_->scale.y = 0.0;
    goal_node_->scale.z = 0.0;
    goal_node_->color.a = 1.0;
    goal_node_->color.r = 1.0;
    goal_node_->color.g = 0.5;
    goal_node_->color.b = 0.5;


    heuristic_node->header.frame_id = "odom";
    heuristic_node->ns = "A_star_Heuristic";
    heuristic_node->id = 0;
    heuristic_node->type = visualization_msgs::Marker::LINE_STRIP;
    heuristic_node->action = visualization_msgs::Marker::ADD;
    heuristic_node->pose.orientation.x = 0.0;
    heuristic_node->pose.orientation.y = 0.0;
    heuristic_node->pose.orientation.z = 0.0;
    heuristic_node->pose.orientation.w = 1.0;
    heuristic_node->scale.x = 0.02; //Line width
    heuristic_node->scale.y = 0.0;
    heuristic_node->scale.z = 0.0;
    heuristic_node->color.a = 1.0;
    heuristic_node->color.r = 1.0;
    heuristic_node->color.g = 1.0;
    heuristic_node->color.b = 0.0;

    new_vehicle_pose_visualization->header.frame_id = "odom";
    new_vehicle_pose_visualization->ns = "new_vehicle_pose";
    new_vehicle_pose_visualization->id = 0;
    new_vehicle_pose_visualization->type = visualization_msgs::Marker::SPHERE_LIST;
    new_vehicle_pose_visualization->action = visualization_msgs::Marker::ADD;
    new_vehicle_pose_visualization->pose.orientation.x = 0.0;
    new_vehicle_pose_visualization->pose.orientation.y = 0.0;
    new_vehicle_pose_visualization->pose.orientation.z = 0.0;
    new_vehicle_pose_visualization->pose.orientation.w = 1.0;
    new_vehicle_pose_visualization->scale.x = 0.1; //Line width
    new_vehicle_pose_visualization->scale.y = 0.0;
    new_vehicle_pose_visualization->scale.z = 0.0;
    new_vehicle_pose_visualization->color.a = 1.0;
    new_vehicle_pose_visualization->color.r = 1.0;
    new_vehicle_pose_visualization->color.g = 1.0;
    new_vehicle_pose_visualization->color.b = 0.0;


    new_node_pose_visualization->header.frame_id = "odom";
    new_node_pose_visualization->ns = "new_node_pose";
    new_node_pose_visualization->id = 0;
    new_node_pose_visualization->type = visualization_msgs::Marker::SPHERE_LIST;
    new_node_pose_visualization->action = visualization_msgs::Marker::ADD;
    new_node_pose_visualization->pose.orientation.x = 0.0;
    new_node_pose_visualization->pose.orientation.y = 0.0;
    new_node_pose_visualization->pose.orientation.z = 0.0;
    new_node_pose_visualization->pose.orientation.w = 1.0;
    new_node_pose_visualization->scale.x = 0.1; //Line width
    new_node_pose_visualization->scale.y = 0.0;
    new_node_pose_visualization->scale.z = 0.0;
    new_node_pose_visualization->color.a = 1.0;

//    new_node_pose_visualization->color.r = 1.0;
//    new_node_pose_visualization->color.g = 0.0;
//    new_node_pose_visualization->color.b = 1.0;

    a_star_path->header.frame_id = "odom";
    a_star_path->header.stamp = ros::Time::now();
}


inline double A_STAR_NODE::MovingCost(const A_STAR_NODE& node){
    double moving, dist_moving, angle_moving;
    A_STAR_NODE parent;
    parent = *(node.node.parent);
    dist_moving = sqrt(pow(parent.node.pose.x-node.node.pose.x,2)+pow(parent.node.pose.y-node.node.pose.y,2));
    angle_moving = fabs(parent.node.pose.heading - node.node.pose.heading);

//    ROS_ERROR("inside of Moving Cost    dist_moving :: %lf \n" , dist_moving);

    if (angle_moving > M_PI) angle_moving = fabs(angle_moving - 2 * M_PI);
    moving = dist_moving + 0.0*angle_moving;
    return parent.node.moving_cost + moving;
}


inline double A_STAR_NODE::MovingCost_Lateral(const A_STAR_NODE& node){
    double moving, dist_moving, angle_moving, lateral_cost;
    A_STAR_NODE parent;
    parent = *(node.node.parent);
    dist_moving = sqrt(pow(parent.node.pose.x-node.node.pose.x,2)+pow(parent.node.pose.y-node.node.pose.y,2));
    angle_moving = fabs(parent.node.pose.heading - node.node.pose.heading);
    if (angle_moving > M_PI) angle_moving = fabs(angle_moving - 2 * M_PI);
    moving = dist_moving + 0.0*angle_moving;

    //lateral cost
    geometry_msgs::PoseStampedPtr current_node_pose(new geometry_msgs::PoseStamped);
    AStarNode.NodePoseToPoseStamped(node.node.pose,current_node_pose);
    lateral_cost = fabs(AStarNode.LateralOffset2(node.node.wp_idx,current_node_pose,a_star_path));

    return parent.node.moving_cost + moving + 0.4*lateral_cost;
}


int HYBRID_A_STAR::init(){

    node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
    pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    a_star_node_pub_ = node_->advertise<visualization_msgs::MarkerArray>("a_star_node", 10);
    a_star_goal_marker_pub_ = node_->advertise<visualization_msgs::Marker>("a_star_goal_line", 10);
    a_star_heuristic_pub_ = node_->advertise<visualization_msgs::Marker>("a_star_heuristic", 10);
    new_vehicle_pose_pub_ = node_->advertise<visualization_msgs::MarkerArray>("new_vehicle_pose",10);
    a_star_path_pub_ = node_->advertise<nav_msgs::Path>(std::string("/a_star_path"), 10);
    new_node_pose_pub_ = node_->advertise<visualization_msgs::MarkerArray>("new_node_pose",10);
    goal_node_marker = node_->advertise<visualization_msgs::Marker>("goal_node_marker",10);
    vehicle_collision = node_->advertise<visualization_msgs::MarkerArray>("/a_star_collision", 10);

    AStarNode.NodeVisualizationMarkerInit();

    printf("hybrid A star initialized!! \n");

    return 0;
}

double A_STAR_NODE::SteeringControl(const double angle_diff, const double lat_offset, const double curvature, const double velocity){
    double wheel_angle;

    double kp_angle;
    double kp_lat;
    double kp_curv;
    double kd_angle;
    double kd_lat;

    double angle_control;
    double lat_control;
    double curv_control;

    kp_angle = 1.0;
    kp_lat = 3.8;
    kp_curv = 1.0;

    angle_control = kp_angle * angle_diff;
    lat_control = atan2(kp_lat * lat_offset, 1.0*velocity);
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
}

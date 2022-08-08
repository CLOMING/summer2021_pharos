#ifndef _HYBRID_A_STAR_H_
#define _HYBRID_A_STAR_H_

#include <local_path_planner.h>
//#include <lateral_offset_planning.h>

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <tf/transform_datatypes.h>


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

#include <pharos_road_information/RoadNetworks.h>
#include <pharos_road_information/Road.h>
#include <pharos_road_information/Lane.h>
#include <pharos_road_information/Lanes.h>
#include <pharos_road_information/Waypoint.h>
#include "lateral_avoiding.h"


#define MovingCostScale 0.9f
#define HybridAstarPlanningDist 34.0f    // original :: 27.0f

namespace HYBRID_A_STAR_NS {


    class A_STAR_NODE {
    public:
        struct Pose {
            double x;
            double y;
            double z;
            double heading;
        };

        struct Node {
            double moving_cost;
            double heuristic;
            unsigned int wp_idx;
            double cost();
            int id;
            Pose pose;
            A_STAR_NODE *parent;
        };

//        struct Reference_Path_{
//            nav_msgs::Path waypoint;
//            nav_msgs::Path curvature;
//            nav_msgs::Path speed;
//        };

        Node node;

        pharos_path_planner::ReferencePathPtr reference_road_;

    public:
        bool operator<(const A_STAR_NODE& other) const
        {
            if ((node.heuristic+MovingCostScale*node.moving_cost) < (other.node.heuristic+MovingCostScale*other.node.moving_cost))
                return true;
            else
                return false;
        };
//        bool operator<(const A_STAR_NODE& other) const {
//            if ((node.heuristic) < (other.node.heuristic))
//                return true;
//            else
//                return false;
//        };
        inline double MaxSteeringAngle(const double vehicle_speed,const double LatAccLimit);

        inline double MovingCost(const A_STAR_NODE& node);
        inline double MovingCost_Lateral(const A_STAR_NODE& node);


        //// NearestWaypointIdx2 for find Astar idx
        unsigned int NearestWaypointIdx2(const geometry_msgs::PoseStampedConstPtr &vehicle_pose, const unsigned int old_wp_idx,const int search_num, const nav_msgs::PathPtr &a_star_path ){
            unsigned int idx = 0;

            tf::Point p1,p2;
            p1.setValue(vehicle_pose->pose.position.x,vehicle_pose->pose.position.y,0.0);
            double min_dist = DBL_MAX;

            unsigned long path_length = a_star_path->poses.size();

            unsigned int search_start_idx = old_wp_idx;
            if(search_start_idx < 0){
                search_start_idx += path_length;
            }

            unsigned int search_finish_idx = search_start_idx + search_num;

            for(unsigned int i=search_start_idx; i<search_finish_idx;i++){
                p2.setValue(a_star_path->poses.at(i%path_length).pose.position.x,a_star_path->poses.at(i%path_length).pose.position.y,0.0);
                double dist = p1.distance(p2);
                if(dist < min_dist){
                    min_dist = dist;
                    idx = (unsigned int)(i%path_length);
                }
            }
            return idx;
        };

        double AngleDifference2(const unsigned int wp_idx, const geometry_msgs::PoseStampedConstPtr &vehicle_pose, const nav_msgs::PathPtr &a_star_path){

            double angle_diff = 0.0;

            double r_path_heading;
            double vehicle_heading;

            unsigned long path_length = a_star_path->poses.size();

            double x0 = a_star_path->poses.at(wp_idx).pose.position.x;
            double y0 = a_star_path->poses.at(wp_idx).pose.position.y;
            double x1 = a_star_path->poses.at((wp_idx+1)%path_length).pose.position.x;
            double y1 = a_star_path->poses.at((wp_idx+1)%path_length).pose.position.y;

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
        };

        void BicycleModelMotionUpdate2(const geometry_msgs::PoseStampedConstPtr &in_pose,
                                       geometry_msgs::PoseStampedPtr &out_pose,
                                       const double wheel_angle,
                                       const double vehicle_speed,
                                       const double dt){
            tf::Pose pose;
            tf::poseMsgToTF(in_pose->pose, pose);
            double Roll,Pitch, Yaw;
            pose.getBasis().getRPY(Roll, Pitch, Yaw);
            double vehicle_heading = Yaw;

            double beta = vehicle_speed*dt/VehicleWheelBase*tan(wheel_angle);
            vehicle_heading += beta;
            double dx = vehicle_speed*dt * cos(vehicle_heading);
            double dy = vehicle_speed*dt * sin(vehicle_heading);


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


        inline double LateralOffset2(const unsigned int wp_idx, const geometry_msgs::PoseStampedConstPtr &vehicle_pose, const nav_msgs::PathPtr &a_star_path){
            double lateral_offset;

            double vehicle_x = vehicle_pose->pose.position.x;
            double vehicle_y = vehicle_pose->pose.position.y;

            unsigned long path_length = a_star_path->poses.size();

            double x1,y1,x2,y2;

            x1 = a_star_path->poses.at(wp_idx).pose.position.x;
            y1 = a_star_path->poses.at(wp_idx).pose.position.y;
            x2 = a_star_path->poses.at((wp_idx+1)%path_length).pose.position.x;
            y2 = a_star_path->poses.at((wp_idx+1)%path_length).pose.position.y;

            lateral_offset = ((vehicle_x-x1)*(y2-y1)-(vehicle_y-y1)*(x2-x1))/(sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)));

            return lateral_offset;
        };



        inline void PoseStampedToNodePose(const geometry_msgs::PoseStampedConstPtr& pose_msg,A_STAR_NODE::Pose &node_pose){
            tf::Pose pose;
            tf::poseMsgToTF(pose_msg->pose, pose);
            double Roll,Pitch, Yaw;
            pose.getBasis().getRPY(Roll, Pitch, Yaw);
            double heading = Yaw;


            node_pose.x = pose_msg->pose.position.x;
            node_pose.y = pose_msg->pose.position.y;
            node_pose.heading = heading;

        }
        inline void NodePoseToPoseStamped(const A_STAR_NODE::Pose &node_pose,geometry_msgs::PoseStampedPtr& pose_msg){
            geometry_msgs::Quaternion quat_heading;
            tf::Quaternion q;
            q.setRPY(0,0,node_pose.heading);
            tf::quaternionTFToMsg(q,quat_heading);

            pose_msg->pose.position.x = node_pose.x;
            pose_msg->pose.position.y = node_pose.y;
            pose_msg->pose.position.z = node_pose.z;
            pose_msg->pose.orientation = quat_heading;
            pose_msg->header.frame_id = "odom";
        }

        inline void NodeVisualizationMarkerInit(void);
        double SteeringControl(const double angle_diff, double lat_offset, const double curvature, const double velocity);


    };




    class HEURISTICS{
    public:
        inline double EuclidDist(const A_STAR_NODE &current_node, const A_STAR_NODE &goal_node);
        double EuclidDist_Angle(const A_STAR_NODE &current_node, const A_STAR_NODE &goal_node);
        double EuclidDist_Lateraloffset(const A_STAR_NODE &current_node, const A_STAR_NODE &goal_node);
        double EuclidDist_Angle_Lateraloffset(const A_STAR_NODE &current_node, const A_STAR_NODE &goal_node);
        double VirtualPathFollowing(/*const pharos_path_planner::ReferencePath &ReferencePath,*/const A_STAR_NODE &current_node, const A_STAR_NODE &goal_node,const double vehicle_speed, double dt);
    };





    class HYBRID_A_STAR {

    public:

        ros::NodeHandlePtr node_;
        ros::NodeHandlePtr pnode_;

        ros::Publisher a_star_node_pub_;
        ros::Publisher a_star_goal_marker_pub_;
        ros::Publisher a_star_heuristic_pub_;
        ros::Publisher a_star_path_pub_;
        ros::Publisher new_vehicle_pose_pub_;
        ros::Publisher new_node_pose_pub_;
        ros::Publisher goal_node_marker;
        ros::Publisher vehicle_collision;

        void HybridAStarPathPlanning(const Lateral_Avoiding::Reference_Path LA_reference_path,
                                     const pharos_road_information::LanePtr reference_lanes,
                                     const pharos_path_planner::ReferencePathPtr reference_path_,
                                     const geometry_msgs::PoseStampedConstPtr &VehiclePose,
                                     const tf::Point EgoVehiclePose,
                                     const nav_msgs::OccupancyGridConstPtr &obstacleMap,
                                     const double vehicle_speed,
                                     const double goal_speed,
                                     const double dt,
                                     const unsigned int init_wp_idx,
                                     nav_msgs::OccupancyGridPtr &history_map,
                                     nav_msgs::PathPtr &path_solution
        );



    public:

        int init();


    };
}

#endif

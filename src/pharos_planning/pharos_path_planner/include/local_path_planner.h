#ifndef _3D_PATH_PLANNER_H_
#define _3D_PATH_PLANNER_H_

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

//#include <lateral_avoiding.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <tf/transform_datatypes.h>

#include <pharos_msgs/StateStamped2016.h>
#include <pharos_msgs/SpeedCommand.h>
#include <pharos_msgs/ObjectInfoArray.h>

#include <pharos_path_planner/RoadInfo.h>
#include <pharos_path_planner/ReferencePath.h>

#include <collision_check.h>



#include <pharos_behavior_planner/DrivingState.h>
#include <pharos_behavior_planner/LaneChangeState.h>

#include <pharos_road_information/RoadNetworks.h>
#include <pharos_road_information/Road.h>
#include <pharos_road_information/Lane.h>
#include <pharos_road_information/Lanes.h>
#include <pharos_road_information/Waypoint.h>


#define OCC_FULL 200
#define OCC_BASE 120
// #define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

#define VehicleWidth 2.0f
#define VehicleLength 5.5f
#define VehicleWheelBase 2.7f
#define MaxWheelAngle 30.0f

#define CollisionMargin 2.0f

#define NumWpSearch 30

#define G 9.81 //gravity acceleration

#define B_Spline_Order 6    //// Bigger B-Spline Order makes more smooth lane but more calculation.
#define B_Spline_Dt 0.5

#define LANE_CHANGING_READY 1
#define LANE_CHANGING_START 2
#define LANE_CHANGING_FINISH 3

const float VehBox_Min_x = -(VehicleLength-VehicleWheelBase)/2;
const float VehBox_Max_x = VehicleWheelBase + (VehicleLength-VehicleWheelBase)/2;
const float VehBox_Min_y = -VehicleWidth/2;
const float VehBox_Max_y = VehicleWidth/2;

const float SqureLength = 1.7;

extern nav_msgs::PathPtr waypoint_;
extern nav_msgs::PathPtr curvature_;

//VehicleCollisionCheck VehicleCollisionCheck_;

const double MAX_ACC = 0.5*G;
const double MAX_DCC = -0.377*G;

class PathPlanning3D {
public:

    struct SideBackDetection{
        bool left;
        bool right;
    };

    SideBackDetection SideBackDetection_;

    ros::NodeHandlePtr node_;
    ros::NodeHandlePtr pnode_;

    ros::Subscriber vehicle_odom_sub_;
    ros::Subscriber vehicle_state_sub_;
    
    ros::Subscriber obstacle_map_sub_;
    ros::Subscriber lookahead_dist_sub_;

    ros::Subscriber ref_road_network_sub_;

    ros::Subscriber current_roadinfo_sub_;
    ros::Subscriber ref_roadinfo_sub_;
    ros::Subscriber ref_lanes_sub_;
    ros::Subscriber global_path_update_sub_;
    ros::Subscriber DrivingState_sub_;

    ros::Subscriber tracekd_objects_sub_;
    ros::Subscriber LaneChangeState_sub_;
    ros::Subscriber mission_state_sub_;

    ros::Subscriber dist_sub_;

    ros::Publisher predicted_map_pub_;

    ros::Publisher Driving_state_pub_;
    ros::Publisher final_path_pub_;
    ros::Publisher planner_speed_pub_;
    ros::Publisher planner_curvature_pub_;
    ros::Publisher Emergency_state_pub;

    ros::Publisher side_lane_path_pub_;
    ros::Publisher lane_changing_path_pub_;
    ros::Publisher global_path_planning_request_pub_;
    ros::Publisher a_star_algorithm_pub_;
    
    std::vector<bool> obstacle_region_;
    
    double Planning_Total_Time_;
    double Planning_Dt_;

    bool ForceAvoiding_;

    double LateralOffset_;

    int Re_Trial_;

    double vehicle_speed_;
    double ref_goal_speed_;
    double ref_goal_speed_dist_;
    unsigned int current_wp_idx_;
    double lookahaed_distance_;
    geometry_msgs::PoseStamped vehicle_pose_;
    geometry_msgs::PolygonStamped vehicle_polygon_;

    bool wp_idx_get_;
    bool vehicle_odom_get_;
    bool vehicle_speed_get_;
    bool road_networks_get_;

    bool global_path_update_;

    double force_avoiding_waiting_time_;
    
    int lane_change_state_;
    double lane_approach_;
    int Mission_states_;

    unsigned int Driving_State_;

    pharos_road_information::RoadNetworksPtr road_networks_;

    pharos_behavior_planner::DrivingState DrivingState_;
    pharos_behavior_planner::DrivingState LocalDrivingState_;

    std_msgs::Bool Emergency_state_;

    std::vector<nav_msgs::OccupancyGrid> FutureMaps_;
    bool tracked_map_generation_;
    std::vector<geometry_msgs::PoseStamped> TrackedVehicle_List_;

    pharos_msgs::ObjectInfoArray TrackedObject_List_;

    nav_msgs::OccupancyGrid init_obstacle_map_;
    bool map_get_;

    int init();

    void GlobalPathUpdateCallback(const std_msgs::BoolConstPtr& msg);
    void RoadNetworkGraphCallback(const pharos_road_information::RoadNetworksConstPtr& msg);
    void ReferenceRoadInfoCallback(const pharos_path_planner::ReferencePathConstPtr &msg);
    void ReferenceLanesCallback(const pharos_road_information::LanesConstPtr &msg);
    void CurrentRoadInfoCallback(const pharos_path_planner::RoadInfoConstPtr &msg);
    void MissionStateCallback(const std_msgs::Int32Ptr msg);

    void VehicleOdomCallback(const nav_msgs::OdometryConstPtr &msg);
    void VehicleStateCallback(const pharos_msgs::StateStamped2016ConstPtr &msg);
    void LookaheadCallback(const std_msgs::Float32ConstPtr& msg);
    void ObjectboxDistanceCallback(const std_msgs::Float32ConstPtr& msg);
    void ObstacleMapCallback(const nav_msgs::OccupancyGridConstPtr &msg);
    
    void DrivingStateCallback(const pharos_behavior_planner::DrivingStateConstPtr &msg);
    void LaneChangeStateCallback(const pharos_behavior_planner::LaneChangeStateConstPtr &msg);

    void TrackedObjectCallback(const pharos_msgs::ObjectInfoArrayConstPtr &msg);

    void B_spline(const nav_msgs::PathConstPtr& input_path, nav_msgs::PathPtr& splined_path){

        int n = input_path->poses.size(); //number of control points
        uint32_t K = B_Spline_Order; //order
        if(n<B_Spline_Order){
            K = n;
        }
        //printf("n: %d, K: %d\n",n,K);

        std::vector<double> T; //knots
        int m = n+K; //number of knots
        T.resize(m);
        std::vector<std::vector<double> > B(m,std::vector<double>(K,0.0)); //Basis function
        if(n<K){
            ROS_ERROR("Number of Control Points(input path) have to be bigger than order k(k = %d)",K);
            return;
        }

        if(K<2){
            ROS_ERROR("Order of B-spline, 'k' have to be bigger than 1!");
            return;
        }

        for(int i=0;i<K;i++){
            T[i] = 0.0;
        }
        for(int i=K;i<m-K+1;i++){
            T[i] = T[i-1] + 1.0;
        }
        for(int i=m-K+1;i<m;i++){
            T[i] = T[i-1];
        }

//		printf("T: ");
//		for(int i=0;i<T.size();i++){
//			printf("%.1f ",T[i]);
//		}
//		printf("\n");

        double dt = B_Spline_Dt;
        double t = T[0];

        while(t <= (T[m-1]+0.0001)){
            for(int j=0;j<K;j++) {
                for (int i=0; i < n+K-1; i++) {
                    if(j==0) {
                        if ((fabs(t-T[i])<0.0001) ||(t > T[i])){
                            if(t < (T[i + 1]-0.0001)){
                                B[i][j] = 1.0;
                            }
                            else B[i][j] = 0.0;
                        }
                        else B[i][j] = 0.0;
                        //printf("t: %.2f, T[%d]: %.2f,B[%d][%d] = %.3f\n",t,i,T[i],i,j,B[i][j]);
                    }
                    else{
                        double r1 = 0.0;
                        double r2 = 0.0;
                        //if(fabs(T[i+j-1]-T[i])<0.0001){
                        if(fabs(T[i+j]-T[i])<0.0001){
                            r1 = 0.0;
                        }
                        else{
                            //r1 = (t-T[i])/(T[i+j-1]-T[i]);
                            r1 = (t-T[i])/(T[i+j]-T[i]);
                        }
                        //if(fabs(T[i+j]-T[i+1])<0.0001){
                        if(fabs(T[i+j+1]-T[i+1])<0.0001){
                            r2 = 0.0;
                        }
                        else{
                            //r2 = (T[i+j]-t)/(T[i+j]-T[i+1]);
                            r2 = (T[i+j+1]-t)/(T[i+j+1]-T[i+1]);
                        }
                        B[i][j] = r1*B[i][j-1] + r2*B[i+1][j-1];
                        if(B[i][j] == 0){
                            //printf("B[%d][%d]: %.1f, r1:%.2f ,r2:%.2f ,B[%d][%d]: %.2f,B[%d][%d]: %.2f\n",i,j,B[i][j],r1,r2,i,j-1,B[i][j-1],i+1,j-1,B[i+1][j-1]);
                        }
                        //printf("B[%d][%d] = %.3f\n",i,j,B[i][j]);
                    }
                }
                n-=1;
            }
            n = input_path->poses.size();

            if(fabs(t-T[0])<0.0001){
                B[0][K-1] = 1.0;
            }
            if(fabs(t - T[m-1])<0.0001){
                //printf("t: %.3f, T[m-1]: %.3f\n",t,T[m-1]);
                B[n-1][K-1] = 1.0;
            }

            geometry_msgs::PoseStamped s;
            for(int k=0;k<m-K;k++){
                s.pose.position.x += input_path->poses.at(k).pose.position.x * B[k][K-1];
                s.pose.position.y += input_path->poses.at(k).pose.position.y * B[k][K-1];
                s.pose.position.z = input_path->poses.at(k).pose.position.z;
                //printf("B[%d][%d] (%.1f) = %.3f\n",k,K-1,t,B[k][K-1]);
            }
            splined_path->poses.push_back(s);
            t+=dt;
        }


        for(unsigned int i=0; i<splined_path->poses.size()-1;i++){

            double x1,x2,y1,y2;
            x1 = splined_path->poses.at(i).pose.position.x;
            y1 = splined_path->poses.at(i).pose.position.y;
            x2 = splined_path->poses.at(i+1).pose.position.x;
            y2 = splined_path->poses.at(i+1).pose.position.y;

            double heading = atan2(y2-y1,x2-x1);

            geometry_msgs::Quaternion quat_heading;
            tf::Quaternion q;
            q.setRPY(0,0,heading);
            tf::quaternionTFToMsg(q,quat_heading);
            splined_path->poses.at(i).pose.orientation = quat_heading;
        }
        //(*splined_path->poses.begin()).pose.orientation = splined_path->poses.at(1).pose.orientation;
        (*splined_path->poses.end()).pose.orientation = splined_path->poses.at(splined_path->poses.size()-2).pose.orientation;

        for(unsigned int i=0; i<splined_path->poses.size();i++){
            tf::Pose pose;
            tf::poseMsgToTF(splined_path->poses.at(i).pose, pose);
            double Roll,Pitch, Yaw;
            pose.getBasis().getRPY(Roll, Pitch, Yaw);
            double vehicle_heading = Yaw;
            //printf("splined_path_heading [%d]: %.3f\n",i,vehicle_heading);
        }
        //printf("Splined path size: %d\n",splined_path->poses.size());
    };

    double LateralOffsetwithWaypoints(const pharos_road_information::LaneConstPtr &lane, geometry_msgs::PoseStamped vehicle_pose){
        double lateral_offset;

        double vehicle_x = vehicle_pose.pose.position.x;
        double vehicle_y = vehicle_pose.pose.position.y;

        unsigned long wp_length = lane->waypoints.size();

        unsigned int idx = 0;

        tf::Point p1,p2;
        p1.setValue(vehicle_x,vehicle_y,0.0);
        double min_dist = DBL_MAX;

        for(unsigned int i=0; i<wp_length-1;i++){
            p2.setValue(lane->waypoints.at(i).x,lane->waypoints.at(i).y,0.0);
            double dist = p1.distance(p2);
            if(dist < min_dist){
                min_dist = dist;
                idx = i;
            }
        }

        double x1,y1,x2,y2;

        x1 = lane->waypoints.at(idx).x;
        y1 = lane->waypoints.at(idx).y;
        x2 = lane->waypoints.at(idx+1).x;
        y2 = lane->waypoints.at(idx+1).y;

        lateral_offset = ((vehicle_x-x1)*(y2-y1)-(vehicle_y-y1)*(x2-x1))/(sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)));

        // printf("vehicle pose[x: %.3f, y: %.3f], wp [x: %.3f, y: %.3f]\n",vehicle_x, vehicle_y, x1, y1);

        return lateral_offset;
    };

    inline double QuatToEulerHeading(const geometry_msgs::PoseStamped &pose_stamped){
        double heading;

        tf::Pose pose;
        tf::poseMsgToTF(pose_stamped.pose, pose);
        double Roll,Pitch, Yaw;
        static double vehicle_heading;
        pose.getBasis().getRPY(Roll, Pitch, Yaw);
        heading = Yaw;

        return heading;
    };


    inline double PathHeading(const unsigned int wp_idx, const nav_msgs::PathConstPtr &path){

        double path_heading;
        unsigned long path_length = path->poses.size();

        double x0 = path->poses.at(wp_idx).pose.position.x;
        double y0 = path->poses.at(wp_idx).pose.position.y;
        double x1 = path->poses.at((wp_idx+1)%path_length).pose.position.x;
        double y1 = path->poses.at((wp_idx+1)%path_length).pose.position.y;
        path_heading = atan2(y1-y0,x1-x0);

        return path_heading;
    }


    void PolygonToMapIdx(const geometry_msgs::Polygon, int &idx){


    };

    void PredictiveMap(const nav_msgs::OccupancyGridConstPtr& map, const geometry_msgs::PoseStamped ego_vehicle_pose){
        
        std::vector<nav_msgs::OccupancyGrid>::iterator iter_grid;
        for(iter_grid=FutureMaps_.begin();iter_grid!=FutureMaps_.end();++iter_grid){
            *iter_grid = *map;
        }

        if(TrackedObject_List_.objects.empty()) return;
        

        //To Grid Index
        int grid_size = map->info.height;
        int gridXOrigin = grid_size/2;
        int gridYOrigin = grid_size/2;
        double resolution = map->info.resolution;
        double resolutionInverse = 1/resolution;
        unsigned int cropMapArraySize = grid_size*grid_size;

        geometry_msgs::PolygonPtr vehicle_polygon (new geometry_msgs::Polygon);
        geometry_msgs::PoseStampedPtr vehicle_pose (new geometry_msgs::PoseStamped);
        
        std::vector<pharos_msgs::ObjectInfo>::iterator iter;

        for(iter=TrackedObject_List_.objects.begin();iter!=TrackedObject_List_.objects.end();++iter){


            int min_x_idx = (int)(-SqureLength/2 * resolutionInverse);
            int max_x_idx = (int)(SqureLength/2 * resolutionInverse);
            int min_y_idx = (int)(-SqureLength/2 * resolutionInverse);
            int max_y_idx = (int)(SqureLength/2 * resolutionInverse);

            double trackVeh_x = iter->pose.x - ego_vehicle_pose.pose.position.x;
            double trackVeh_y = iter->pose.y - ego_vehicle_pose.pose.position.y;
            double trackVeh_yaw = iter->pose.theta;

            for(int x=min_x_idx;x<=max_x_idx;x++){
                for(int y=min_y_idx;y<=max_y_idx;y++){

                    //Rotation
                    double x1 = x*cos(trackVeh_yaw) - y*sin(trackVeh_yaw);
                    double y1 = x*sin(trackVeh_yaw) + y*cos(trackVeh_yaw);

                    //Translation
                    x1 += trackVeh_x*resolutionInverse;
                    y1 += trackVeh_y*resolutionInverse;

                    int IntX = (int)x1 + gridXOrigin;
                    int IntY = (int)y1 + gridYOrigin;
                    if(IntX >= 0 && IntX < grid_size && IntY >= 0 && IntY < grid_size){
                        int mapIndex = MAP_IDX(grid_size,IntX,IntY);
                        if(mapIndex >= 0 && mapIndex < cropMapArraySize){
                            FutureMaps_.at(0).data.at(mapIndex) = OCC_FULL;
                            // history_map->data[mapIndex] = OCC_BASE + (int)(planning_dt/Planning_Dt_)*occ_increment;
                        }
                    }
                }
            }
        
            vehicle_pose->pose.position.x = iter->pose.x;
            vehicle_pose->pose.position.y = iter->pose.y;
            vehicle_pose->pose.position.z = iter->pose.z;

            // VehicleCollisionCheck_.VehiclePoseToPolygon(vehicle_pose, 1.0, vehicle_polygon);
            // VehicleCollisionCheck_.VehiclePoseToSqure(vehicle_pose, 1.0, vehicle_polygon);

            // //Polygon to Map
            // for(unsigned int i=0; i<vehicle_polygon->points.size();i++){
            //     double x = vehicle_polygon->points.at(i).x -= ego_vehicle_pose.pose.position.x;
            //     double y = vehicle_polygon->points.at(i).y -= ego_vehicle_pose.pose.position.y;

            //     int int_x = (unsigned int) (x * resolutionInverse) + gridXOrigin;
            //     int int_y = (unsigned int) (y * resolutionInverse) + gridYOrigin;

            //     unsigned int IDX = MAP_IDX(grid_size,int_x,int_y);

            //     // FutureMaps_.at(0).data.at(IDX) = OCC_FULL;
            // }
        }


    };
};
#endif


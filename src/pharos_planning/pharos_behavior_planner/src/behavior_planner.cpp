#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>

#include <pharos_path_planner/ReferencePath.h>
#include <pharos_path_planner/RoadInfo.h>
#include <pharos_msgs/StateStamped2016.h>

#include <pharos_behavior_planner/DrivingState.h>
#include <pharos_behavior_planner/StopLane.h>
#include <pharos_msgs/ObjectInfoArray.h>
#include <pharos_behavior_planner/LaneChangeState.h>

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


#include <gps_common/conversions.h>
#include <angles/angles.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Geometry>

//Driving Situation
#define SITUATION_LANE_DRIVING 1
#define SITUATION_INTERSECTION 2
#define SITUATION_PASSENGER_PICKUP 3
#define SITUATION_U_TURN 4

//Driving Action - Lane driving
#define NORMAL_DRIVING 1
#define LANECHANGING 2
#define SLOW_DOWN 3
#define STOP 4

//Driving Action - Intersection
#define APPROACH 1
#define STOPPING 2
#define STOPPED 3
#define PASS 4

//Mission Situation
#define SelectingCall 0
#define SelectCallReq 1
#define GettingOn 2
#define GetOnReq 3
#define GettingOff 4
#define GetOffReq 5
#define Complete 6
//Driving Action - Passenger pick up
#define STATE_DRIVING 1
double Inter1_Case1_Lane1_x1 = 122+471230.422970;
double Inter1_Case1_Lane1_y1 = 22.7+3965684.971734;
double Inter1_Case1_Lane1_x2 = 125+471230.422970;
double Inter1_Case1_Lane1_y2 = 22.2+3965684.971734;
double Inter1_Case1_Lane1_x3 = 136+471230.422970;
double Inter1_Case1_Lane1_y3 = 99.3+3965684.971734;
double Inter1_Case1_Lane1_x4 = 139+471230.422970;
double Inter1_Case1_Lane1_y4 = 99.8+3965684.971734;

double Inter1_Case1_Lane2_x1 = 139+471230.422970;
double Inter1_Case1_Lane2_y1 = 135+3965684.971734;
double Inter1_Case1_Lane2_x2 = 142+471230.422970;
double Inter1_Case1_Lane2_y2 = 135+3965684.971734;
double Inter1_Case1_Lane2_x3 = 128+471230.422970;
double Inter1_Case1_Lane2_y3 = 68+3965684.971734;
double Inter1_Case1_Lane2_x4 = 131+471230.422970;
double Inter1_Case1_Lane2_y4 = 68+3965684.971734;

double Inter1_Case2_Lane1_x1 = 139+471230.422970;
double Inter1_Case2_Lane1_y1 = 135+3965684.971734;
double Inter1_Case2_Lane1_x2 = 142+471230.422970;
double Inter1_Case2_Lane1_y2 = 135+3965684.971734;
double Inter1_Case2_Lane1_x3 = 128+471230.422970;
double Inter1_Case2_Lane1_y3 = 68+3965684.971734;
double Inter1_Case2_Lane1_x4 = 131+471230.422970;
double Inter1_Case2_Lane1_y4 = 68+3965684.971734;

double Inter1_Case2_Lane2_x1 = 122+471230.422970;
double Inter1_Case2_Lane2_y1 = 83.5+3965684.971734;
double Inter1_Case2_Lane2_x2 = 121+471230.422970;
double Inter1_Case2_Lane2_y2 = 77.2+3965684.971734;
double Inter1_Case2_Lane2_x3 = 136+471230.422970;
double Inter1_Case2_Lane2_y3 = 80+3965684.971734;
double Inter1_Case2_Lane2_x4 = 135+471230.422970;
double Inter1_Case2_Lane2_y4 = 73.9+3965684.971734;

double Inter2_Case1_Lane1_x1 = -23.2+471230.422970;
double Inter2_Case1_Lane1_y1 = 43.6+3965684.971734;
double Inter2_Case1_Lane1_x2 = -20+471230.422970;
double Inter2_Case1_Lane1_y2 = 43+3965684.971734;
double Inter2_Case1_Lane1_x3 = -10.2+471230.422970;
double Inter2_Case1_Lane1_y3 = 115+3965684.971734;
double Inter2_Case1_Lane1_x4 = -6.84+471230.422970;
double Inter2_Case1_Lane1_y4 = 114+3965684.971734;

double Inter2_Case1_Lane2_x1 = -19.7+471230.422970;
double Inter2_Case1_Lane2_y1 = 47.5+3965684.971734;
double Inter2_Case1_Lane2_x2 = -16+471230.422970;
double Inter2_Case1_Lane2_y2 = 47+3965684.971734;
double Inter2_Case1_Lane2_x3 = -7.23+471230.422970;
double Inter2_Case1_Lane2_y3 = 114+3965684.971734;
double Inter2_Case1_Lane2_x4 = -4+471230.422970;
double Inter2_Case1_Lane2_y4 = 113+3965684.971734;

double Inter2_Case1_Lane3_x1 = -7.5+471230.422970;
double Inter2_Case1_Lane3_y1 = 94.5+3965684.971734;
double Inter2_Case1_Lane3_x2 = -4.4+471230.422970;
double Inter2_Case1_Lane3_y2 = 112+3965684.971734;
double Inter2_Case1_Lane3_x3 = -16.8+471230.422970;
double Inter2_Case1_Lane3_y3 = 96.3+3965684.971734;
double Inter2_Case1_Lane3_x4 = -102.4+471318.992601;
double Inter2_Case1_Lane3_y4 = 123+3965677.304384;

double Inter2_Case2_Lane1_x1 = -23.2+471230.422970;
double Inter2_Case2_Lane1_y1 = 43.6+3965684.971734;
double Inter2_Case2_Lane1_x2 = -20+471230.422970;
double Inter2_Case2_Lane1_y2 = 43+3965684.971734;
double Inter2_Case2_Lane1_x3 = -10.2+471230.422970;
double Inter2_Case2_Lane1_y3 = 115+3965684.971734;
double Inter2_Case2_Lane1_x4 = -6.84+471230.422970;
double Inter2_Case2_Lane1_y4 = 114+3965684.971734;

double Inter2_Case2_Lane2_x1 = -19.7+471230.422970;
double Inter2_Case2_Lane2_y1 = 47.5+3965684.971734;
double Inter2_Case2_Lane2_x2 = -16+471230.422970;
double Inter2_Case2_Lane2_y2 = 47+3965684.971734;
double Inter2_Case2_Lane2_x3 = -7.23+471230.422970;
double Inter2_Case2_Lane2_y3 = 114+3965684.971734;
double Inter2_Case2_Lane2_x4 = -4+471230.422970;
double Inter2_Case2_Lane2_y4 = 113+3965684.971734;

double Inter2_Case2_Lane3_x1 = -6+471230.422970;
double Inter2_Case2_Lane3_y1 = 156+3965684.971734;
double Inter2_Case2_Lane3_x2 = -91.4+471318.992601;
double Inter2_Case2_Lane3_y2 = 163+3965677.304384;
double Inter2_Case2_Lane3_x3 = -16.8+471230.422970;
double Inter2_Case2_Lane3_y3 = 96.4+3965684.971734;
double Inter2_Case2_Lane3_x4 = -13.7+471230.422970;
double Inter2_Case2_Lane3_y4 = 95.8+3965684.971734;

double Area1_x1 = 45.3+302455.135629;
double Area1_y1 = 103+4123701.212429;
double Area1_x2 = 36.8+302455.135629;
double Area1_y2 = 87.9+4123701.212429;
double Area1_x3 = 52.4+302455.135629;
double Area1_y3 = 79.4+4123701.212429;
double Area1_x4 = 60.9+302455.135629;
double Area1_y4 = 94.5+4123701.212429;

double Area2_x1 = 29.3+302455.135629;
double Area2_y1 = 173+4123701.212429;
double Area2_x2 = 16.6+302455.135629;
double Area2_y2 = 151+4123701.212429;
double Area2_x3 = 31.9+302455.135629;
double Area2_y3 = 143+4123701.212429;
double Area2_x4 = 42.2+302455.135629;
double Area2_y4 = 165+4123701.212429;

double Area3_x1 = 68.1+302455.135629;
double Area3_y1 = 149+4123701.212429;
double Area3_x2 = 57.4+302455.135629;
double Area3_y2 = 129+4123701.212429;
double Area3_x3 = 76.4+302455.135629;
double Area3_y3 = 118+4123701.212429;
double Area3_x4 = 89.5+302455.135629;
double Area3_y4 = 140+4123701.212429;

double Area4_x1 = 115+302455.135629;
double Area4_y1 = 124+4123701.212429;
double Area4_x2 = 103+302455.135629;
double Area4_y2 = 101+4123701.212429;
double Area4_x3 = 113+302455.135629;
double Area4_y3 = 94.8+4123701.212429;
double Area4_x4 = 126+302455.135629;
double Area4_y4 = 118+4123701.212429;

double Area5_x1 = 89.6+302455.135629;
double Area5_y1 = 195+4123701.212429;
double Area5_x2 = 82.9+302455.135629;
double Area5_y2 = 176+4123701.212429;
double Area5_x3 = 101+302455.135629;
double Area5_y3 = 166+4123701.212429;
double Area5_x4 = 110+302455.135629;
double Area5_y4 = 183+4123701.212429;

double spat_400_x = 81.7+302516.960443;
double spat_400_y = 211+4123781.980442;
double spat_500_x = 85.5+302516.960443;
double spat_500_y = 331+4123781.980442;
double spat_700_x = 83+302516.960443;
double spat_700_y = 454+4123781.980442;


pharos_path_planner::ReferencePathPtr reference_path_ (new pharos_path_planner::ReferencePath);
pharos_msgs::ObjectInfoArrayPtr objects_(new pharos_msgs::ObjectInfoArray);
pharos_msgs::ObjectInfoArrayPtr emergency_car(new pharos_msgs::ObjectInfoArray);
geometry_msgs::Point point;

pharos_msgs::MAP MAP_message_;
pharos_msgs::SPAT SPAT_message_;
pharos_msgs::RTCM RTCM_message_;
pharos_msgs::TIM TIM_message_;
pharos_msgs::BSM BSM_message_;


visualization_msgs::MarkerArrayPtr spat_array (new visualization_msgs::MarkerArray);

visualization_msgs::MarkerPtr spat_04 (new visualization_msgs::Marker);
visualization_msgs::MarkerPtr spat_05 (new visualization_msgs::Marker);
visualization_msgs::MarkerPtr spat_07 (new visualization_msgs::Marker);

class BehaviorPlanner
{
public:
	struct traffic_light{
		bool green;
		bool yellow;
		bool red;
		bool arrow;
	};

	struct traffic_light_info{
		int intersection_id;
		std::vector<traffic_light> light;
		// traffic_light light;
	};

	struct intersection_info{
		unsigned int lane_id;
		unsigned int road_id;
		int intersection_id;
		int traffic_light_id;
	};

	struct RPY_{
		double roll;
		double pitch;
		double yaw;
	};

	ros::NodeHandlePtr node_;
	ros::NodeHandlePtr pnode_;

	ros::Publisher DrivigState_pub_;
	ros::Publisher StopLane_pub_;
//	ros::Publisher CallSelec_RES_pub_;
	ros::Publisher GettingOn_REQ_pub_;
	ros::Publisher GettingOff_REQ_pub_;
    ros::Publisher GoalPose_pub_;
	ros::Publisher SideAreaMarker_pub_;
	ros::Publisher BackAreaMarker_pub_;
    ros::Publisher Area1_pub_;
    ros::Publisher Area2_pub_;
    ros::Publisher LeftTrun_pub_;
    ros::Publisher LaneChangeState_pub_;
    ros::Publisher StopSignal_pub_;
    ros::Publisher Workzone_pub_;
    ros::Publisher SCzone_pub_;
    ros::Publisher global_path_planning_request_pub_;
    ros::Publisher BackPoint_pub_;
    ros::Publisher LeftPoint_pub_;
    ros::Publisher emergency_state_pub_;

    ros::Publisher workzone_bool;

    ros::Publisher wk_points_pub_;
    ros::Publisher sc_zone_pub_;
    ros::Publisher spat_signal_pub_;

    ros::Subscriber Odom_sub_;
	ros::Subscriber SPAT_DATA_sub_;
	ros::Subscriber CallMatching_CONFIRM_sub_;
	ros::Subscriber GettingOnComplete_CONFIRM_sub_;
	ros::Subscriber GettingOffComplete_CONFIRM_sub_;
	ros::Subscriber CallRequestList_DATA_sub_;
    ros::Subscriber MissionStatus_sub_;
    ros::Subscriber Vehicle_sub_;
	ros::Subscriber current_roadinfo_sub_;
	ros::Subscriber reference_path_sub_;
    ros::Subscriber CallSelec_RES_sub_;
    ros::Subscriber Pose_on_the_road_sub;
    ros::Subscriber Goal_on_the_road_sub;
    ros::Subscriber Tracked_pharos_msgs_sub_;
    ros::Subscriber GettingOffComplete_CONFIRM;
    ros::Subscriber GettingOnComplete_CONFIRM;
    ros::Subscriber mission_state_sub_;

    ros::Subscriber MAP_message_sub_;
    ros::Subscriber SPAT_message_sub_;
    ros::Subscriber RTCM_message_sub_;
    ros::Subscriber BSM_message_sub_;
    ros::Subscriber TIM_message_sub_;


    ros::Time curr_t_;
    ros::Time old_t_;
    float stop_time_;
    float new_vel_;

    double vehicle_speed_;
    double lane_change_count;
	nav_msgs::Odometry vehicle_pose_;
    geometry_msgs::PointStamped goal_pose_;
    geometry_msgs::PoseStamped pose_on_the_road_;
    geometry_msgs::PoseStamped goal_on_the_road_;

	pharos_path_planner::RoadInfo current_roadinfo_;

	pharos_behavior_planner::DrivingState DrivingState_;
	pharos_behavior_planner::StopLane StopLane_;

	int number_of_lanes;
	std::vector<intersection_info> intersection_list;
	std::vector<traffic_light_info> traffic_light_list_;
	traffic_light current_traffic_light_;
    pharos_behavior_planner::LaneChangeState LaneChangeState;

    int emergency_count = 0;


	unsigned int desired_intersection_id_;
	unsigned int desired_traffic_light_id_;

	unsigned int stop_road_id_;
	unsigned int stop_lane_id_;

	unsigned int desired_direction_;
    double dist_to_spat;
	double origin_x_ = 0.0;
	double origin_y_ = 0.0;
	double origin_z_ = 0.0;

    double tim_utm_N = 0;
    double tim_utm_S = 0;
    double sc_utm_N = 0;
    double sc_utm_S = 0;

    double bsm_utm_N = 0;
    double bsm_utm_S = 0;

    int case1_2;
    int case1_3;
    int case1_4;
    int case2_5;
    int case2_3;
    int case3_2;
    int case3_4;
    int case3_5;
    int case4_3;
    int case4_5;


    double left_area_obstacle_distance_count ;
	int Mission_states_;
	bool GetCallSelecRES_;

	bool UnSafetyIntersectionPassing_;

	bool reference_path_update_;
	bool spat_400 = false;
	bool spat_500 = false;
	bool spat_700 = false;

    std_msgs::Bool emergency_mission;

	std_msgs::Float64 work_zone_x;
	std_msgs::Float64 work_zone_y;





	RPY_ QuaterToRPY(geometry_msgs::PoseStamped pose){
		tf::Quaternion quat;
		RPY_ rpy;
		tf::quaternionMsgToTF(pose.pose.orientation,quat);
		tf::Matrix3x3(quat).getRPY(rpy.roll, rpy.pitch, rpy.yaw);
		rpy.yaw = angles::normalize_angle_positive(rpy.yaw);

		return rpy;
	}

    void MAP_messageCallback(const pharos_msgs::MAPConstPtr &msg)
    {
        MAP_message_ = *msg;


    }
    void SPAT_messageCallback(const pharos_msgs::SPATConstPtr &msg)
    {
        SPAT_message_ = *msg;

    }
    void RTCM_messageCallback(const pharos_msgs::RTCMConstPtr &msg)
    {
        RTCM_message_ = *msg;
    }

    void PharosTrackedCallback(const pharos_msgs::ObjectInfoArrayConstPtr &msg)
    {
        *objects_ = *msg;





        int search_length = 30;
        int search_front_length = 10;
        int left_search_length = 15;

        double wave_vehicle_utm_S = 0;
        double wave_vehicle_utm_N = 0;

        RPY_ rpy2;
        rpy2 = QuaterToRPY(pose_on_the_road_);

        visualization_msgs::MarkerArrayPtr back_point_array (new visualization_msgs::MarkerArray);
        visualization_msgs::MarkerArrayPtr left_point_array (new visualization_msgs::MarkerArray);
        visualization_msgs::MarkerPtr back_points (new visualization_msgs::Marker);
        visualization_msgs::MarkerPtr left_points (new visualization_msgs::Marker);

        geometry_msgs::PoseStampedPtr present_vehicle_pose (new geometry_msgs::PoseStamped);
        present_vehicle_pose->header.frame_id = "/odom";
        present_vehicle_pose->header.stamp = vehicle_pose_.header.stamp;
        present_vehicle_pose->pose.position = vehicle_pose_.pose.pose.position;
        present_vehicle_pose->pose.orientation = vehicle_pose_.pose.pose.orientation;

        RPY_ rpy;
        rpy = QuaterToRPY(*present_vehicle_pose);

        geometry_msgs::Point point;
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;
        geometry_msgs::Point p3;
        geometry_msgs::Point p4;

        geometry_msgs::Point p5;
        geometry_msgs::Point p6;
        geometry_msgs::Point p7;
        geometry_msgs::Point p8;

        double p1_utm_x = 0;
        double p1_utm_y = 0;
        double p2_utm_x = 0;
        double p2_utm_y = 0;
        double p3_utm_x = 0;
        double p3_utm_y = 0;
        double p4_utm_x = 0;
        double p4_utm_y = 0;

        double p5_utm_x = 0;
        double p5_utm_y = 0;
        double p6_utm_x = 0;
        double p6_utm_y = 0;
        double p7_utm_x = 0;
        double p7_utm_y = 0;
        double p8_utm_x = 0;
        double p8_utm_y = 0;


        //// for kcity
        p1.x = pose_on_the_road_.pose.position.x-1.5*sin(rpy2.yaw)+(search_length/6)*cos(rpy2.yaw);
        p1.y = pose_on_the_road_.pose.position.y-1.5*cos(rpy2.yaw)+(search_length/6)*sin(rpy2.yaw);
        p2.x = pose_on_the_road_.pose.position.x-1.5*sin(rpy2.yaw)-(search_length/2)*cos(rpy2.yaw);
        p2.y = pose_on_the_road_.pose.position.y-1.5*cos(rpy2.yaw)-(search_length/2)*sin(rpy2.yaw);
        p3.x = pose_on_the_road_.pose.position.x+1.5*sin(rpy2.yaw)-(search_length/2)*cos(rpy2.yaw);
        p3.y = pose_on_the_road_.pose.position.y+1.5*cos(rpy2.yaw)-(search_length/2)*sin(rpy2.yaw);
        p4.x = pose_on_the_road_.pose.position.x+1.5*sin(rpy2.yaw)+(search_length/6)*cos(rpy2.yaw);
        p4.y = pose_on_the_road_.pose.position.y+1.5*cos(rpy2.yaw)+(search_length/6)*sin(rpy2.yaw);


        //// left area in kcity
        p5.x = pose_on_the_road_.pose.position.x-8*sin(rpy2.yaw)+(3*left_search_length)*cos(rpy2.yaw);
        p5.y = pose_on_the_road_.pose.position.y-8*cos(rpy2.yaw)+(3*left_search_length)*sin(rpy2.yaw);

        p6.x = pose_on_the_road_.pose.position.x-8*sin(rpy2.yaw)-(1.5*left_search_length)*cos(rpy2.yaw);
        p6.y = pose_on_the_road_.pose.position.y-8*cos(rpy2.yaw)-(1.5*left_search_length)*sin(rpy2.yaw);

        p7.x = pose_on_the_road_.pose.position.x+3*sin(rpy2.yaw)-(1.5*left_search_length)*cos(rpy2.yaw);
        p7.y = pose_on_the_road_.pose.position.y+3*cos(rpy2.yaw)-(1.5*left_search_length)*sin(rpy2.yaw);

        p8.x = pose_on_the_road_.pose.position.x+3*sin(rpy2.yaw)+(3*left_search_length)*cos(rpy2.yaw);
        p8.y = pose_on_the_road_.pose.position.y+3*cos(rpy2.yaw)+(3*left_search_length)*sin(rpy2.yaw);









        //// for kcity2kut
//        p1.x = pose_on_the_road_.pose.position.x+3*cos(rpy2.yaw)+(search_length/6)*cos(rpy2.yaw);
//        p1.y = pose_on_the_road_.pose.position.y-3*sin(rpy2.yaw)+(search_length/6)*sin(rpy2.yaw);
//
//        p2.x = pose_on_the_road_.pose.position.x+3*cos(rpy2.yaw)-(search_length/3)*cos(rpy2.yaw);
//        p2.y = pose_on_the_road_.pose.position.y-3*sin(rpy2.yaw)-(search_length/3)*sin(rpy2.yaw);
//
//        p3.x = pose_on_the_road_.pose.position.x+3*sin(rpy2.yaw)-(search_length/3)*cos(rpy2.yaw);
//        p3.y = pose_on_the_road_.pose.position.y-3*cos(rpy2.yaw)-(search_length/3)*sin(rpy2.yaw);
//
//        p4.x = pose_on_the_road_.pose.position.x+3*sin(rpy2.yaw)+(search_length/6)*cos(rpy2.yaw);
//        p4.y = pose_on_the_road_.pose.position.y-3*cos(rpy2.yaw)+(search_length/6)*sin(rpy2.yaw);


        back_points->header.frame_id = "/odom";
        back_points->header.stamp = ros::Time::now();
        back_points->ns = "back_points";
        back_points->id = 0;
        back_points->type = visualization_msgs::Marker::POINTS;
        back_points->action = visualization_msgs::Marker::ADD;
        back_points->scale.x = 1; //Line width
        back_points->scale.y = 1;
        back_points->scale.z = 0.0;
        back_points->color.a = 0.5;
        back_points->color.r = 0;
        back_points->color.g = 1;
        back_points->color.b = 1;

        left_points->header.frame_id = "/odom";
        left_points->header.stamp = ros::Time::now();
        left_points->ns = "left_points";
        left_points->id = 0;
        left_points->type = visualization_msgs::Marker::POINTS;
        left_points->action = visualization_msgs::Marker::ADD;
        left_points->scale.x = 1; //Line width
        left_points->scale.y = 1;
        left_points->scale.z = 0.0;
        left_points->color.a = 0.8;
        left_points->color.r = 1;
        left_points->color.g = 0;
        left_points->color.b = 1;

        back_points->points.push_back(p1);
        back_points->points.push_back(p2);
        back_points->points.push_back(p3);
        back_points->points.push_back(p4);

        left_points->points.push_back(p5);
        left_points->points.push_back(p6);
        left_points->points.push_back(p7);
        left_points->points.push_back(p8);

        left_point_array->markers.push_back(*left_points);
        back_point_array->markers.push_back(*back_points);
        BackPoint_pub_.publish(back_point_array);
        LeftPoint_pub_.publish(left_point_array);

        p1_utm_x = origin_x_ + p1.x;
        p1_utm_y = origin_y_ + p1.y;
        p2_utm_x = origin_x_ + p2.x;
        p2_utm_y = origin_y_ + p2.y;
        p3_utm_x = origin_x_ + p3.x;
        p3_utm_y = origin_y_ + p3.y;
        p4_utm_x = origin_x_ + p4.x;
        p4_utm_y = origin_y_ + p4.y;

        p5_utm_x = origin_x_ + p5.x;
        p5_utm_y = origin_y_ + p5.y;
        p7_utm_y = origin_y_ + p7.y;
        p8_utm_x = origin_x_ + p8.x;



        unsigned int VehicleClass = 0;

//        //// Wave Data
//        for(int i = 0; i < BSM_message_.partII.size(); i++)
//        {
////            printf(" wave data is recieved !!! \n");
//            if(BSM_message_.partII.at(i).SupplementalVehicleExtensions.at(0).vehicleClass == 69)
//            {
//                for(int i=0; i<BSM_message_.coreData.size() ; i++)
//                {
//                    double bsm_longitude = 0;
//                    double bsm_latitude  = 0;
//                    std::string utm_zone = "52";
//                    double bsm_position_x = 0;
//                    double bsm_position_y = 0;
//                    double dist_to_emer_car = 0;
//
//                    double vehicle_origin_utm_x = 0;
//                    double vehicle_origin_utm_y = 0;
//
//                    bsm_longitude = BSM_message_.coreData.at(i).longitude;
//                    bsm_latitude =  BSM_message_.coreData.at(i).lat;
//
//                    bsm_longitude = 0.0000001*bsm_longitude;
//                    bsm_latitude = 0.0000001*bsm_latitude;
//
//                    gps_common::LLtoUTM(bsm_latitude , bsm_longitude , bsm_utm_N , bsm_utm_S, utm_zone);
//
//                    printf("bsm_utm_S : %lf \n ",bsm_utm_S);
//                    printf("bsm_utm_N : %lf \n ",bsm_utm_N);
//
//                    wave_vehicle_utm_N  = bsm_utm_N;
//                    wave_vehicle_utm_S  = bsm_utm_S;
//                    vehicle_origin_utm_x = origin_x_ + vehicle_pose_.pose.pose.position.x;
//                    vehicle_origin_utm_y = origin_y_ + vehicle_pose_.pose.pose.position.y;
//
//                    dist_to_emer_car  = sqrt(pow( vehicle_origin_utm_x - bsm_utm_S, 2) + pow(vehicle_origin_utm_y - bsm_utm_N ,2));
//                    if(bsm_utm_S < p1_utm_x &&
//                       bsm_utm_S > p4_utm_x &&
//                       bsm_utm_N > p1_utm_y &&
//                       bsm_utm_N < p3_utm_y &&
//                       Mission_states_ == 5)
//                    {
//                        ROS_ERROR("emergency area satisfied!!!!\n");
//                        emergency_mission.data = true;
//                        DrivingState_.situation = 69;
//
//                    }
//                    if(Mission_states_ == 6&&
//                       (
//                               bsm_utm_S < p5_utm_x &&
//                               bsm_utm_S > p8_utm_x &&
//                               bsm_utm_N > p5_utm_y &&
//                               bsm_utm_N < p7_utm_y))
//                    {
//                        ROS_ERROR("left area warning!! \n");
//                        LaneChangeState.left = false;
//                    }else
//                    {
//                        ROS_ERROR("BSM data Out of box ! \n");
//                        if(Mission_states_ == 6)
//                        {
//                            ROS_ERROR("left is safe !! \n");
//                            LaneChangeState.left = true;
//                            DrivingState_.situation = 70;
//                        }
//                    }
//                }
//            }
//        }
//        LaneChangeState_pub_.publish(LaneChangeState);

        LaneChangeState.left = true;
        //// for tracking
        pharos_msgs::ObjectInfoArray emergency_car;
        //// tracking data
        for(int j= 0; j<msg->objects.size() ; j++)
        {
            double object_pose_x = 0;
            double object_pose_y = 0;
            double object_distance = 0;

            object_pose_x = msg->objects.at(j).pose.x;
            object_pose_y = msg->objects.at(j).pose.y;
            object_distance = sqrt(pow(vehicle_pose_.pose.pose.position.x-object_pose_x,2)+pow(vehicle_pose_.pose.pose.position.y-object_pose_y,2));

//            ROS_ERROR("Vehicle class :: %u \n ", VehicleClass);
//            ROS_ERROR("object pose x : %lf , y : %lf " , object_pose_x,object_pose_y);
//            ROS_ERROR("object distance :: %lf \n\n", object_distance);
//
//            ROS_ERROR("p1_x : %lf , p1_y : %lf ",p1.x , p1.y );
//            ROS_ERROR("p2_x : %lf , p2_y : %lf ",p2.x , p2.y );
//            ROS_ERROR("p3_x : %lf , p3_y : %lf ",p3.x , p3.y );
//            ROS_ERROR("p4_x : %lf , p4_y : %lf ",p4.x , p4.y );
//
//            ROS_ERROR("bsm_utm_S : %lf , bsm_utm_N : %lf \n\n" , wave_vehicle_utm_S , wave_vehicle_utm_N);
//
//            ROS_ERROR("p1_utm_x : %lf , p1_utm_y : %lf ",p1_utm_x , p1_utm_y );
//            ROS_ERROR("p2_utm_x : %lf , p2_utm_y : %lf ",p2_utm_x , p2_utm_y );
//            ROS_ERROR("p3_utm_x : %lf , p3_utm_y : %lf ",p3_utm_x , p3_utm_y );
//            ROS_ERROR("p4_utm_x : %lf , p4_utm_y : %lf ",p4_utm_x , p4_utm_y );

//            printf("object pose x : %lf , y : %lf " , object_pose_x,object_pose_y);
//            printf("object distance :: %lf \n", object_distance);
//            printf("p1_x : %lf , p1_y : %lf ",p1.x , p1.y );
//            printf("p2_x : %lf , p2_y : %lf ",p2.x , p2.y );
//            printf("p3_x : %lf , p3_y : %lf ",p3.x , p3.y );
//            printf("p4_x : %lf , p4_y : %lf \n\n\n\n",p4.x , p4.y );

            if(
               //// for kcity
               msg->objects.at(j).pose.x < p1.x &&
               msg->objects.at(j).pose.x > p4.x &&
               msg->objects.at(j).pose.y > p1.y &&
               msg->objects.at(j).pose.y < p3.y &&

               //// for kcity2kut
//               msg->objects.at(j).pose.x < p1.x &&
//               msg->objects.at(j).pose.x > p3.x &&
//               msg->objects.at(j).pose.y > p4.y &&
//               msg->objects.at(j).pose.y < p2.y &&

//               msg->objects.at(j).speed !=0 &&
               object_distance < 15 &&
               Mission_states_==5 /*&&(
               (bsm_utm_S < p1_utm_x &&
               bsm_utm_S > p4_utm_x &&
               bsm_utm_N > p1_utm_y &&
               bsm_utm_N < p3_utm_y)
               )*/
              )
            {
                emergency_mission.data = true;
                DrivingState_.situation = 69;
            }
            if(Mission_states_ == 6 &&
               (
                       msg->objects.at(j).pose.x < p5.x &&
                       msg->objects.at(j).pose.x > p8.x &&
                       msg->objects.at(j).pose.y > p5.y &&
                       msg->objects.at(j).pose.y < p7.y))
            {
                LaneChangeState.left = false;
            }
        }

        if(Mission_states_ == 6 && LaneChangeState.left == true)
        {
            printf("Emergency car is out of left lane !\n");
            emergency_count++;
            if(emergency_count>30)
            {
                printf("Emergency count  : %d \n", emergency_count);
                DrivingState_.situation = 70;
                emergency_count = 0;
            }
        }
        LaneChangeState_pub_.publish(LaneChangeState);
        emergency_state_pub_.publish(emergency_mission);


    }

    void BSM_messageCallback(const pharos_msgs::BSMConstPtr &msg)
    {
        BSM_message_ = *msg;

    }
    void TIM_messageCallback(const pharos_msgs::TIMConstPtr &msg)
    {
        TIM_message_ = *msg;
        double tim_longitude = 0;
        double tim_latitude = 0;
        double sc_longitude = 0;
        double sc_latitude = 0;

        std_msgs::Bool workzone_bool_;

		pharos_msgs::Workzone_xy workzone_xy;
		pharos_msgs::Workzone_list workzone_list;
		pharos_msgs::Workzone_list sc_zone_list;
        pharos_msgs::Workzone_xy sc_xy;

        visualization_msgs::MarkerArrayPtr wk_array (new visualization_msgs::MarkerArray);
        visualization_msgs::MarkerPtr wk_points (new visualization_msgs::Marker);
        visualization_msgs::MarkerArrayPtr sc_array (new visualization_msgs::MarkerArray);
        visualization_msgs::MarkerPtr sc_points (new visualization_msgs::Marker);


        wk_points->header.frame_id = "/odom";
        wk_points->header.stamp = ros::Time::now();
        wk_points->ns = "wk_points";
        wk_points->id = 0;
        wk_points->type = visualization_msgs::Marker::POINTS;
        wk_points->action = visualization_msgs::Marker::ADD;
        wk_points->scale.x = 3; //Line width
        wk_points->scale.y = 1;
        wk_points->scale.z = 0.0;
        wk_points->color.a = 0.5;
        wk_points->color.r = 0;
        wk_points->color.g = 1;
        wk_points->color.b = 1;


        sc_points->header.frame_id = "/odom";
        sc_points->header.stamp = ros::Time::now();
        sc_points->ns = "sc_points";
        sc_points->id = 0;
        sc_points->type = visualization_msgs::Marker::POINTS;
        sc_points->action = visualization_msgs::Marker::ADD;
        sc_points->scale.x = 3; //Line width
        sc_points->scale.y = 1;
        sc_points->scale.z = 0.0;
        sc_points->color.a = 0.5;
        sc_points->color.r = 0;
        sc_points->color.g = 1;
        sc_points->color.b = 1;




        geometry_msgs::Point wk_point_1;
        geometry_msgs::Point sc_point;

        tim_utm_N = 0;
        tim_utm_S = 0;
        sc_utm_N = 0;
        sc_utm_S = 0;
        std::string utm_zone;
        workzone_list.workzone_xy.clear();

        for(int i=0; i<TIM_message_.dataFrames.size();i++)
        {


                if(TIM_message_.dataFrames.at(i).presentlist.at(0).type=="sc")
                {
                    for(int j = 0; j<TIM_message_.dataFrames.at(i).list.at(0).path.at(0).ll.size(); j++)
                    {
                        sc_longitude = TIM_message_.dataFrames.at(i).list.at(0).path.at(0).ll.at(j).longitude;
                        sc_latitude = TIM_message_.dataFrames.at(i).list.at(0).path.at(0).ll.at(j).lat;
                        sc_longitude = 0.0000001*sc_longitude;
                        sc_latitude = 0.0000001*sc_latitude;

                        gps_common::LLtoUTM(sc_latitude,sc_longitude, sc_utm_N, sc_utm_S, utm_zone);

                        sc_xy.workzone_x = sc_utm_S;
                        sc_xy.workzone_y = sc_utm_N;
                        sc_xy.ref_speed = atof(TIM_message_.dataFrames.at(i).presentlist.at(0).value.c_str());

                        sc_point.x = sc_xy.workzone_x-origin_x_;
                        sc_point.y = sc_xy.workzone_y-origin_y_;
                        sc_zone_list.workzone_xy.push_back(sc_xy);
                        sc_points->points.push_back(sc_point);
                    }
                    SCzone_pub_.publish(sc_zone_list);
                    sc_array->markers.push_back(*sc_points);
                    sc_zone_pub_.publish(sc_array);
                }

                if(TIM_message_.dataFrames.at(i).presentlist.at(0).type=="wk")
                {

                tim_longitude = TIM_message_.dataFrames.at(i).list.at(0).anchor.at(0).longitude;
                tim_latitude = TIM_message_.dataFrames.at(i).list.at(0).anchor.at(0).lat;


                tim_longitude  = 0.0000001*tim_longitude;
                tim_latitude = 0.0000001*tim_latitude;



                gps_common::LLtoUTM(tim_latitude,tim_longitude , tim_utm_N,tim_utm_S, utm_zone);


                workzone_xy.workzone_x = tim_utm_S;
                workzone_xy.workzone_y = tim_utm_N;

                workzone_list.workzone_xy.push_back(workzone_xy);

                wk_point_1.x = workzone_xy.workzone_x-origin_x_;
                wk_point_1.y = workzone_xy.workzone_y-origin_y_;


                wk_points->points.push_back(wk_point_1);
                wk_array->markers.push_back(*wk_points);
                wk_points_pub_.publish(wk_array);
                }
            if(Mission_states_ > -1 && Mission_states_ < 6 )
            {
                workzone_bool_.data = true;
            }else
            {
                workzone_bool_.data = false;
            }

        }
        Workzone_pub_.publish(workzone_list);
        workzone_bool.publish(workzone_bool_);

    }



	void VehiclePoseCallback(const nav_msgs::OdometryConstPtr &msg){
		vehicle_pose_ = *msg;

	}
	void ReferenceRoadInfoCallback(const pharos_path_planner::ReferencePathConstPtr &msg){
		if(reference_path_->roadinfo.size() != msg->roadinfo.size()){
			reference_path_update_ = true;

			DrivingState_.situation = SITUATION_LANE_DRIVING;
			DrivingState_.action = NORMAL_DRIVING;
		}
		else{
			reference_path_update_ = false;
		}
		*reference_path_ = *msg;

	}

	void CurrentRoadInfoCallback(const pharos_path_planner::RoadInfoConstPtr &msg){
		current_roadinfo_ = *msg;
		if(reference_path_->roadinfo.empty()) return;

		// State Machine

		int wp_idx;
        wp_idx = current_roadinfo_.wp_index;
		tf::Point current_p;
		current_p.setValue(msg->position.x, msg->position.y,0.0);

		double distance_to_intersection = 0.0;
		static tf::Point stopping_point;
		if(DrivingState_.situation == SITUATION_LANE_DRIVING){

			if((DrivingState_.action == NORMAL_DRIVING && current_roadinfo_.road_type == 1) || (DrivingState_.action == NORMAL_DRIVING && current_roadinfo_.road_type == 3)){

			    //// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! here is matter !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ////

				tf::Point p1,p2;

				bool intersection_find = false;
				while(1){
                    if(reference_path_->roadinfo.at(wp_idx).road_type == 2 ){

						int intersection_road_id = reference_path_->roadinfo.at(wp_idx).road_number;
						int intersection_lane_id = reference_path_->roadinfo.at(wp_idx).lane_number;

						StopLane_.stop_road_id = reference_path_->roadinfo.at(wp_idx-1).road_number;
						StopLane_.stop_lane_id = reference_path_->roadinfo.at(wp_idx-1).lane_number;


						for(unsigned int i=0;i<intersection_list.size();i++){

                            if(intersection_list.at(i).road_id == intersection_road_id && intersection_list.at(i).lane_id == intersection_lane_id){

                                desired_intersection_id_ = intersection_list.at(i).intersection_id;
								desired_traffic_light_id_ = intersection_list.at(i).traffic_light_id;
								intersection_find = true;
								desired_direction_ = reference_path_->roadinfo.at(wp_idx+1).action;
								stopping_point.setValue(reference_path_->roadinfo.at(wp_idx).position.x,reference_path_->roadinfo.at(wp_idx).position.y,0.0);
								// printf("stopping point11, x:%.3f, y: %.3f\n",stopping_point.x(),stopping_point.y());
							}
						}
						if(!intersection_find){
							DrivingState_.situation = SITUATION_LANE_DRIVING;
							DrivingState_.action = NORMAL_DRIVING;

							break;
						}

						if(distance_to_intersection < 80.0){

							if(reference_path_->roadinfo.at(wp_idx).road_type == 2){
								DrivingState_.situation = SITUATION_INTERSECTION;
							}
							else if(reference_path_->roadinfo.at(wp_idx).road_type == 4){
								DrivingState_.situation = SITUATION_U_TURN;
							}


							// DrivingState_.situation = SITUATION_INTERSECTION;
							DrivingState_.action = APPROACH;
							DrivingState_.stop_wp_idx = wp_idx;
							// desired_direction_ = reference_path_->roadinfo.at(wp_idx).action;
							// stopping_point.setValue(reference_path_->roadinfo.at(wp_idx).position.x,reference_path_->roadinfo.at(wp_idx).position.y,0.0);
//							printf("roadtype: %d, wp_idx: %d, stop_road_id, stop_lane_id\n",reference_path_->roadinfo.at(wp_idx).road_type,wp_idx,StopLane_.stop_road_id,StopLane_.stop_lane_id);

						}


						break;
					}

                    if(wp_idx == reference_path_->roadinfo.size()-2) break;
					p1.setValue(reference_path_->roadinfo.at(wp_idx).position.x,reference_path_->roadinfo.at(wp_idx).position.y,reference_path_->roadinfo.at(wp_idx).position.z);
					wp_idx++;
					p2.setValue(reference_path_->roadinfo.at(wp_idx).position.x,reference_path_->roadinfo.at(wp_idx).position.y,reference_path_->roadinfo.at(wp_idx).position.z);
					distance_to_intersection += p1.distance(p2);
				}
			}


		}
		  //// SITUATION_UTURN
		else if(DrivingState_.situation == SITUATION_INTERSECTION){

		    tf::Point p3, p4;
            int wp_idx2;
            wp_idx2 = current_roadinfo_.wp_index;
            double distance_in_intersection = 0.0;


//		    ROS_INFO("We ar in SITUATION_INTERSECTION !!! \n");
			// printf("origin intersection id: %d\n",desired_intersection_id_);

			for(unsigned int i=current_roadinfo_.wp_index; i<reference_path_->roadinfo.size();i++){
				if(reference_path_->roadinfo.at(i).road_type == 2 || reference_path_->roadinfo.at(i).road_type == 4){
                    DrivingState_.stop_wp_idx = i;

					break;
				}
			}

            for(int i = wp_idx2; i<DrivingState_.stop_wp_idx; i++)
            {
                if(wp_idx2 == DrivingState_.stop_wp_idx) break;
                if(wp_idx2 == reference_path_->roadinfo.size()-2) break;
                p3.setValue(reference_path_->roadinfo.at(wp_idx2).position.x,reference_path_->roadinfo.at(wp_idx2).position.y,reference_path_->roadinfo.at(wp_idx2).position.z);
                wp_idx2++;
                p4.setValue(reference_path_->roadinfo.at(wp_idx2).position.x,reference_path_->roadinfo.at(wp_idx2).position.y,reference_path_->roadinfo.at(wp_idx2).position.z);
                distance_in_intersection += p3.distance(p4);

            }

            //// debugging spat messages
//            ROS_ERROR("distance in intersection :: %lf  \n ", distance_in_intersection);
            dist_to_spat = distance_in_intersection;

//            if(dist_to_spat < 1 && spat_400 == true && spat_500 == true)
//            {
//                spat_700 = true;
//            }
//            if(dist_to_spat < 1 && spat_400 == true)
//            {
//                spat_500 = true;
//            }
//            if(dist_to_spat < 1)
//            {
//                spat_400 = true;
//            }

//            ROS_ERROR("spat 400 bool : %d \n" , spat_400);
//            ROS_ERROR("spat 500 bool : %d \n" , spat_500);
//            ROS_ERROR("spat 700 bool : %d \n" , spat_700);

//            ROS_ERROR("situation intersection reference path size :: %lu \n" , reference_path_->roadinfo.size()-2);
//            ROS_ERROR("wp_idx 2 = %d  \n",wp_idx2);
//            ROS_ERROR("Drivingstate_stop wp idx   %d \n" , DrivingState_.stop_wp_idx);





            int new_intersection_id = IntersectionIDmapping(desired_intersection_id_);
			// printf("intersection id : %d\n",new_intersection_id);
//            ROS_ERROR("CHECKing_new_intersection_id :: %d\n",new_intersection_id);
//            ROS_ERROR("FOR_CHECKING!!!!!!!!!!!!!!!!!!!!!!!\n");

            if(new_intersection_id == -1) return;
			else if(new_intersection_id == 13 || new_intersection_id == 14){  // need for change
        		static bool first_stopping = false;
				if(DrivingState_.action == APPROACH){
					if(!first_stopping){
						DrivingState_.action = STOPPING;
					}

					if(current_roadinfo_.road_type == 2 || current_roadinfo_.road_type == 4){
						DrivingState_.action = PASS;
					}
				}
				else if(DrivingState_.action == STOPPING){
					double dist_to_stop_point = current_p.distance(stopping_point);

					if(!first_stopping){
						if(fabs(vehicle_speed_) < 3.0 && dist_to_stop_point < 12.0){
							first_stopping = true;
						}
					}
					else{
						if(UnSafetyIntersectionPassing_){
							DrivingState_.action = APPROACH;
						}
					}
					if(current_roadinfo_.road_type == 2 || current_roadinfo_.road_type == 4){
						DrivingState_.action = PASS;
					}
				}
				// else if(DrivingState_.action == STOPPED){

				// }
				else if(DrivingState_.action == PASS){
					DrivingState_.situation = SITUATION_LANE_DRIVING;
					DrivingState_.action = NORMAL_DRIVING;
					first_stopping = false;
				}
			}

			else{
				current_traffic_light_ = traffic_light_list_.at(new_intersection_id).light.at(desired_traffic_light_id_-1);

				if(DrivingState_.action == APPROACH){

					// printf("2222\n");
					if(desired_direction_ == 1){
						if(current_traffic_light_.arrow) DrivingState_.action = APPROACH;
						else if(current_traffic_light_.red || current_traffic_light_.yellow) DrivingState_.action = STOPPING;
					}
					else if(desired_direction_ == 2){
						if(current_traffic_light_.green) DrivingState_.action = APPROACH;
						else if(current_traffic_light_.red || current_traffic_light_.yellow) DrivingState_.action = STOPPING;
					}
					else if(desired_direction_ == 3){
						if(current_traffic_light_.green) DrivingState_.action = APPROACH;
						else if(current_traffic_light_.red || current_traffic_light_.yellow) DrivingState_.action = STOPPING;
						if(current_traffic_light_.arrow) DrivingState_.action = APPROACH;
					}

					if(current_roadinfo_.road_type == 2 || current_roadinfo_.road_type == 4){
						DrivingState_.action = PASS;
					}
					// printf("3333\n");
				}
				else if(DrivingState_.action == STOPPING){

					if(desired_direction_ == 1){
						if(current_traffic_light_.arrow) DrivingState_.action = APPROACH;
					}
					else if(desired_direction_ == 2){
						if(current_traffic_light_.green) DrivingState_.action = APPROACH;
					}
					else if(desired_direction_ == 3){
						if(current_traffic_light_.green) DrivingState_.action = APPROACH;
						if(current_traffic_light_.arrow) DrivingState_.action = APPROACH;
					}

					// If vehicle cannot stop before stopline then just pass
					if(current_roadinfo_.road_type == 2 || current_roadinfo_.road_type == 4){
						DrivingState_.action = PASS;
					}
				}
				// else if(DrivingState_.action == STOPPED){

				// }
				else if(DrivingState_.action == PASS){
					DrivingState_.situation = SITUATION_LANE_DRIVING;
					DrivingState_.action = NORMAL_DRIVING;
				}

			}

		}

		DrivigState_pub_.publish(DrivingState_);
		if(DrivingState_.action == STOPPING){
			StopLane_pub_.publish(StopLane_);
		}
		UnSafetyIntersectionPassing_ = false;


//// for traffic light
//		printf("Driving Situation: %d, Action: %d\n",DrivingState_.situation, DrivingState_.action);
//		printf("intersection id: %d, traffic_light id: %d, stop road: %d, stop lane: %d\n",desired_intersection_id_, desired_traffic_light_id_, StopLane_.stop_road_id, StopLane_.stop_lane_id);
//		printf("intersec_direction: %d, traffic_light: [%d %d %d %d]\n",desired_direction_,current_traffic_light_.red,current_traffic_light_.yellow,current_traffic_light_.arrow,current_traffic_light_.green);
//		printf("remain distance to intersection: %.3f\n",distance_to_intersection);
//		printf("\n");

//		// if(DrivingState_.situation == SITUATION_INTERSECTION)
		// 	printf("intersection: %d, traffic_light: %d, stop road: %d, stop lane: %d\n",intersection_id, traffic_light_id, StopLane_.stop_road_id, StopLane_.stop_lane_id);
		// 	printf("remain distance to intersection: %.3f\n",distance_to_intersection);
		// }
	}

	void SPAT_DATA_Callback(const pharos_msgs::SPATConstPtr &msg) {

        SPAT_message_ = *msg;

        int spat_data_size_ = SPAT_message_.intersections.size();

        ros::WallTime start_, end_;

        start_ = ros::WallTime::now();

        for (unsigned int i = 0; i < spat_data_size_; i++) {
            int intersection_id = 0;
            traffic_light light = {0, 0, 0, 0};
            spat_array->markers.clear();

            intersection_id = SPAT_message_.intersections.at(0).id.at(0).id;



            int new_intersection_id = IntersectionIDmapping(intersection_id);
            if (new_intersection_id == -1) continue;

            for (unsigned int j = 0; j < SPAT_message_.intersections.at(0).states.size(); j++) {
                spat_04->colors.clear();
                spat_05->colors.clear();
                spat_07->colors.clear();
                traffic_light_list_.at(new_intersection_id).light.at(j).red = false;
                traffic_light_list_.at(new_intersection_id).light.at(j).green = false;
                traffic_light_list_.at(new_intersection_id).light.at(j).yellow = false;
                traffic_light_list_.at(new_intersection_id).light.at(j).arrow = false;

                if (new_intersection_id == 0) {
                    int first_signal = SPAT_message_.intersections.at(0).states.at(j).state_time_speed.at(0).eventState;
                    if (SPAT_message_.intersections.at(0).states.at(j).signalGroupId == 1) {
                        if (first_signal == 3) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).red = true;
                        } else if (first_signal == 5) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).green = true;
                        } else if (first_signal == 7) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).yellow = true;
                        }
                    }
                } else if (new_intersection_id == 01) {
                    int first_signal = SPAT_message_.intersections.at(0).states.at(j).state_time_speed.at(0).eventState;
                    if (SPAT_message_.intersections.at(0).states.at(j).signalGroupId == 1) {
                        if (first_signal == 3) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).red = true;
                        } else if (first_signal == 5) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).green = true;
                        } else if (first_signal == 7) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).yellow = true;
                        }
                    }
                    if (SPAT_message_.intersections.at(0).states.at(j).state_time_speed.at(0).eventState == 2) {
                        if (first_signal == 3) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).red = true;
                        } else if (first_signal == 5) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).arrow = true;
                        } else if (first_signal == 7) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).yellow = true;
                        }
                    }
                } else if (new_intersection_id == 11) {
                    int first_signal = SPAT_message_.intersections.at(0).states.at(j).state_time_speed.at(0).eventState;
                    if (SPAT_message_.intersections.at(0).states.at(j).signalGroupId == 1) {
                        if (first_signal == 3) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).red = true;
                        } else if (first_signal == 5) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).green = true;
                        } else if (first_signal == 7) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).yellow = true;
                        }
                    }
                    if (SPAT_message_.intersections.at(0).states.at(j).state_time_speed.at(0).eventState == 2) {
                        if (first_signal == 3) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).red = true;
                        } else if (first_signal == 5) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).arrow = true;
                        } else if (first_signal == 7) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).yellow = true;
                        }
                    }
                } else if (new_intersection_id == 2) {
                    int first_signal = SPAT_message_.intersections.at(0).states.at(j).state_time_speed.at(0).eventState;
                    if (SPAT_message_.intersections.at(0).states.at(j).signalGroupId == 1) {
                        if (first_signal == 3) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).red = true;
                        } else if (first_signal == 5) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).green = true;
                        } else if (first_signal == 7) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).yellow = true;
                        }
                    }
                    if (SPAT_message_.intersections.at(0).states.at(j).state_time_speed.at(0).eventState == 2) {
                        if (first_signal == 3) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).red = true;
                        } else if (first_signal == 5) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).arrow = true;
                        } else if (first_signal == 7) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).yellow = true;
                        }
                    }
                } else if (new_intersection_id == 3) {
                    int first_signal = SPAT_message_.intersections.at(0).states.at(j).state_time_speed.at(0).eventState;
                    if (SPAT_message_.intersections.at(0).states.at(j).signalGroupId == 1) {
                        if (first_signal == 3) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).red = true;
                        } else if (first_signal == 5) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).green = true;
                        } else if (first_signal == 7) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).yellow = true;
                        }
                    }
                    if (SPAT_message_.intersections.at(0).states.at(j).state_time_speed.at(0).eventState == 2) {
                        if (first_signal == 3) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).red = true;
                        } else if (first_signal == 5) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).arrow = true;
                        } else if (first_signal == 7) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).yellow = true;
                        }
                    }
                } else if (new_intersection_id == 4) {
                    int first_signal = SPAT_message_.intersections.at(0).states.at(j).state_time_speed.at(0).eventState;
                    if (SPAT_message_.intersections.at(0).states.at(j).signalGroupId == 1) {

                        double drivable_dist = 0;
                        double remain_time = 0;

                        if (first_signal == 3) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).red = true;
                            spat_04->color.r = 1;
                            spat_04->color.g = 0;
                            spat_04->color.b = 0;
                        } else if (first_signal == 5) {
                                remain_time = SPAT_message_.intersections.at(0).states.at(j).state_time_speed.at(0).timing.at(0).minEndTime;
                                drivable_dist = 0.1*remain_time*(vehicle_speed_+2);

                                if(remain_time < 41)
                                {
                                    printf("4 remain time :: %lf \n"  , remain_time);
                                    printf("4 spat drivable_dist :: %lf \n" , drivable_dist);
                                    printf("vehicle speed : %lf \n" , vehicle_speed_);
                                    printf("4 dist_to_spat ::       %lf \n " , dist_to_spat);
                                    if(drivable_dist < dist_to_spat)
                                    {

                                        traffic_light_list_.at(new_intersection_id).light.at(j).red = true;
                                        spat_04->color.r = 1;
                                        spat_04->color.g = 1;
                                        spat_04->color.b = 0;
                                    }else
                                    {
                                        traffic_light_list_.at(new_intersection_id).light.at(j).green = true;
                                        spat_04->color.r = 0;
                                        spat_04->color.g = 1;
                                        spat_04->color.b = 0;
                                    }
                                }else
                                {
                                    traffic_light_list_.at(new_intersection_id).light.at(j).green = true;
                                    spat_04->color.r = 0;
                                    spat_04->color.g = 1;
                                    spat_04->color.b = 0;
                         }



                        } else if (first_signal == 7) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).yellow = true;
                            spat_04->color.r = 1;
                            spat_04->color.g = 0.65;
                            spat_04->color.b = 0;
                        }
                    }
                    if (SPAT_message_.intersections.at(0).states.at(j).state_time_speed.at(0).eventState == 2) {
                        if (first_signal == 3) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).red = true;
                        } else if (first_signal == 5) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).arrow = true;
                        } else if (first_signal == 7) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).yellow = true;
                        }
                    }
                    spat_array->markers.push_back(*spat_04);
                }
                else if (new_intersection_id == 5) {
                    int first_signal = SPAT_message_.intersections.at(0).states.at(j).state_time_speed.at(0).eventState;
                    if (SPAT_message_.intersections.at(0).states.at(j).signalGroupId == 1) {
                        double drivable_dist = 0;
                        double remain_time = 0;
                        if (first_signal == 3) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).red = true;
                            spat_05->color.r = 1;
                            spat_05->color.g = 0;
                            spat_05->color.b = 0;
                        } else if (first_signal == 5) {

                                remain_time = SPAT_message_.intersections.at(0).states.at(j).state_time_speed.at(0).timing.at(0).minEndTime;
                                drivable_dist = 0.1*remain_time*(vehicle_speed_+3);

                                if(remain_time < 41)
                                {
                                    printf("5 remain time :: %lf \n"  , remain_time);
                                    printf("5 spat drivable_dist :: %lf \n" , drivable_dist);
                                    printf("5 dist_to_spat ::       %lf \n " , dist_to_spat);
                                    printf("vehicle speed : %lf \n" , vehicle_speed_);

                                    if(drivable_dist < dist_to_spat)
                                    {
                                        traffic_light_list_.at(new_intersection_id).light.at(j).red = true;
                                        spat_05->color.r = 1;
                                        spat_05->color.g = 1;
                                        spat_05->color.b = 0;
                                    }else
                                    {
                                        traffic_light_list_.at(new_intersection_id).light.at(j).green = true;
                                        spat_05->color.r = 0;
                                        spat_05->color.g = 1;
                                        spat_05->color.b = 0;
                                    }
                                }else
                                {
                                    traffic_light_list_.at(new_intersection_id).light.at(j).green = true;
                                    spat_05->color.r = 0;
                                    spat_05->color.g = 1;
                                    spat_05->color.b = 0;
                                }



                        } else if (first_signal == 7) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).yellow = true;
                            spat_05->color.r = 1;
                            spat_05->color.g = 0.65;
                            spat_05->color.b = 0;
                        }
                    }
                    spat_array->markers.push_back(*spat_05);

                } else if (new_intersection_id == 7) {
                    int first_signal = SPAT_message_.intersections.at(0).states.at(j).state_time_speed.at(0).eventState;
                    if (SPAT_message_.intersections.at(0).states.at(j).signalGroupId == 1) {
                        double drivable_dist = 0;
                        double remain_time = 0;
                        if (first_signal == 3) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).red = true;
                            spat_07->color.r = 1;
                            spat_07->color.g = 0;
                            spat_07->color.b = 0;
                        } else if (first_signal == 5) {


                                remain_time = SPAT_message_.intersections.at(0).states.at(j).state_time_speed.at(0).timing.at(0).minEndTime;
                                drivable_dist = 0.1*remain_time*(vehicle_speed_+2);

                                if(remain_time < 41)
                                {
                                    printf("7 remain time :: %lf \n"  , remain_time);
                                    printf("7 spat drivable_dist :: %lf \n" , drivable_dist);
                                    printf("7 dist_to_spat ::       %lf \n " , dist_to_spat);
                                    printf("vehicle speed : %lf \n" , vehicle_speed_);

                                    if(drivable_dist < dist_to_spat)
                                    {
                                        traffic_light_list_.at(new_intersection_id).light.at(j).red = true;
                                        spat_07->color.r = 1;
                                        spat_07->color.g = 1;
                                        spat_07->color.b = 0;
                                    }else
                                    {
                                        traffic_light_list_.at(new_intersection_id).light.at(j).green = true;
                                        spat_07->color.r = 0;
                                        spat_07->color.g = 1;
                                        spat_07->color.b = 0;
                                    }
                                }else
                                {
                                    traffic_light_list_.at(new_intersection_id).light.at(j).green = true;
                                    spat_07->color.r = 0;
                                    spat_07->color.g = 1;
                                    spat_07->color.b = 0;
                                }

                            }
                        else if (first_signal == 7) {
                            traffic_light_list_.at(new_intersection_id).light.at(j).yellow = true;
                            spat_07->color.r = 1;
                            spat_07->color.g = 0.65;
                            spat_07->color.b = 0;
                        }
                    }

                    spat_array->markers.push_back(*spat_07);
                }
            }
            spat_signal_pub_.publish(spat_array);
        }

        end_ = ros::WallTime::now();

        double execution_time = (end_ - start_).toNSec() * 1e-6;
//        ROS_INFO_STREAM("Exectution time (ms): " << execution_time);

    }


    void LaneChangingPlanner(geometry_msgs::PoseStamped pose, pharos_msgs::ObjectInfoArrayConstPtr objects, int road_type, int lane_number)
    {
        RPY_ rpy;
        rpy = QuaterToRPY(pose);

        visualization_msgs::MarkerArrayPtr side_area_array (new visualization_msgs::MarkerArray);

        visualization_msgs::MarkerPtr left_area (new visualization_msgs::Marker);
        visualization_msgs::MarkerPtr right_area (new visualization_msgs::Marker);

        pharos_behavior_planner::LaneChangeState LaneChangeState;
        LaneChangeState.left = true;
        LaneChangeState.right = true;

        int search_lenght = 20;
        int search_front_lenght = 10;

        int dangerous_area = 15;
        float change_speed = 3;

        left_area->header.frame_id = "/odom";
        left_area->header.stamp = objects->header.stamp;
        left_area->ns = "left_area";
        left_area->id = 0;
        left_area->type = visualization_msgs::Marker::LINE_STRIP;
        left_area->action = visualization_msgs::Marker::ADD;
        left_area->scale.x = 3; //Line width
        left_area->scale.y = 0;
        left_area->scale.z = 0.0;
        left_area->color.a = 0.5;
        left_area->color.r = 0;
        left_area->color.g = 1;
        left_area->color.b = 0;

        right_area->header.frame_id = "/odom";
        right_area->header.stamp = objects->header.stamp;
        right_area->ns = "right_area";
        right_area->id = 0;
        right_area->type = visualization_msgs::Marker::LINE_STRIP;
        right_area->action = visualization_msgs::Marker::ADD;
        right_area->scale.x = 3; //Line width
        right_area->scale.y = 0;
        right_area->scale.z = 0.0;
        right_area->color.a = 0.5;
        right_area->color.r = 0;
        right_area->color.g = 1;
        right_area->color.b = 0;

        geometry_msgs::Point point;


        if(road_type==4)
        {
            point.x = pose.pose.position.x-4.5*sin(rpy.yaw);
            point.y = pose.pose.position.y+4.5*cos(rpy.yaw);
            left_area->points.push_back(point);
            point.x = pose.pose.position.x-4.5*sin(rpy.yaw)+(search_front_lenght+search_lenght)*cos(rpy.yaw);
            point.y = pose.pose.position.y+4.5*cos(rpy.yaw)+(search_front_lenght+search_lenght)*sin(rpy.yaw);
            left_area->scale.x = 6; //Line width
        }
        else{
            if(lane_number==2)
            {
                point.x = pose.pose.position.x-3*sin(rpy.yaw)+search_front_lenght*cos(rpy.yaw);
                point.y = pose.pose.position.y+3*cos(rpy.yaw)+search_front_lenght*sin(rpy.yaw);
                left_area->points.push_back(point);
                point.x = pose.pose.position.x-3*sin(rpy.yaw)-search_lenght*cos(rpy.yaw);
                point.y = pose.pose.position.y+3*cos(rpy.yaw)-search_lenght*sin(rpy.yaw);
            }
            else
            {
                point.x = pose.pose.position.x-3*sin(rpy.yaw);
                point.y = pose.pose.position.y+3*cos(rpy.yaw);
                left_area->points.push_back(point);
                point.x = pose.pose.position.x-3*sin(rpy.yaw)+search_lenght*cos(rpy.yaw);
                point.y = pose.pose.position.y+3*cos(rpy.yaw)+search_lenght*sin(rpy.yaw);
            }
        }
        left_area->points.push_back(point);

        point.x = pose.pose.position.x+3*sin(rpy.yaw)+search_front_lenght*cos(rpy.yaw);
        point.y = pose.pose.position.y-3*cos(rpy.yaw)+search_front_lenght*sin(rpy.yaw);
        right_area->points.push_back(point);

        point.x = pose.pose.position.x+3*sin(rpy.yaw)-search_lenght*cos(rpy.yaw);
        point.y = pose.pose.position.y-3*cos(rpy.yaw)-search_lenght*sin(rpy.yaw);
        right_area->points.push_back(point);

        for(int i =0;i<objects->objects.size();i++)
        {
            Eigen::RowVector4f obstacle_pose;
            obstacle_pose(0) = objects->objects[i].pose.x;
            obstacle_pose(1) = objects->objects[i].pose.y;
            obstacle_pose(2) = 1.0;
            obstacle_pose(3) = 1.0;

            Eigen::AngleAxisf init_rotation_z (rpy.yaw,Eigen::Vector3f::UnitZ ());

            Eigen::Translation3f init_translation(pose.pose.position.x,
                                                  pose.pose.position.y,
                                                  0.0);
            Eigen::Matrix4f RT = (init_translation * init_rotation_z).matrix();
            Eigen::RowVector4f transformed_pose;
            transformed_pose.transpose() = RT.inverse() * obstacle_pose.transpose();

            if(road_type == 1 || road_type == 2)
            {
                if(lane_number==1)
                {
                    if(transformed_pose(0)>0 && transformed_pose(0)<search_lenght && transformed_pose(1)>1.5 && transformed_pose(1)<4.5)
                    {
                        if(transformed_pose(0)<dangerous_area+5)
                        {
                            left_area->color.r = 1;
                            left_area->color.g = 0;
                            left_area->color.b = 0;
                            LaneChangeState.left = false;
                        }
                    }
                }
                else if(lane_number==2)
                {
                    if(transformed_pose(0)<search_front_lenght && transformed_pose(0)>-search_lenght && transformed_pose(1)>1.5 && transformed_pose(1)<4.5)
                    {
                        if(transformed_pose(0)>-dangerous_area+5 || transformed_pose(0) -change_speed+objects->objects[i].speed >-dangerous_area)
                        {
                            left_area->color.r = 1;
                            left_area->color.g = 0;
                            left_area->color.b = 0;
                            LaneChangeState.left = false;
                        }
                    }
                }

                if(transformed_pose(0)<search_front_lenght && transformed_pose(0)>-search_lenght && transformed_pose(1)<-1.5 && transformed_pose(1)>-4.5)
                {
                    if(transformed_pose(0)>-dangerous_area+5 || transformed_pose(0) -change_speed+objects->objects[i].speed >-dangerous_area)
                    {
                        right_area->color.r = 1;
                        right_area->color.g = 0;
                        right_area->color.b = 0;
                        LaneChangeState.right = false;
                    }
                }
            }
            else if(road_type==3 || road_type==5)
            {
                left_area->color.r = 1;
                left_area->color.g = 0;
                left_area->color.b = 0;
                LaneChangeState.left = false;

                right_area->color.r = 1;
                right_area->color.g = 0;
                right_area->color.b = 0;
                LaneChangeState.right = false;
            }
            else if(road_type==4)
            {
                if(transformed_pose(0)>0 && transformed_pose(0)<search_front_lenght+search_lenght && transformed_pose(1)>1.5 && transformed_pose(1)<7.5)
                {
                    if(transformed_pose(0)<dangerous_area+5)
                    {
                        left_area->color.r = 1;
                        left_area->color.g = 0;
                        left_area->color.b = 0;
                        LaneChangeState.left = false;
                    }
                }
                if(transformed_pose(0)<search_front_lenght && transformed_pose(0)>-search_lenght && transformed_pose(1)<-1.5 && transformed_pose(1)>-4.5)
                {
                    if(transformed_pose(0)>-dangerous_area+5 || transformed_pose(0) -change_speed+objects->objects[i].speed >-dangerous_area)
                    {
                        right_area->color.r = 1;
                        right_area->color.g = 0;
                        right_area->color.b = 0;
                        LaneChangeState.right = false;
                    }
                }
            }
        }

        side_area_array->markers.push_back(*left_area);
        side_area_array->markers.push_back(*right_area);

        SideAreaMarker_pub_.publish(side_area_array);
        LaneChangeState_pub_.publish(LaneChangeState);
    }


    void MissionStateCallback(const std_msgs::Int32Ptr msg)
    {
        Mission_states_ = msg->data;
    }

    void VehicleStateCallback(const pharos_msgs::StateStamped2016ConstPtr& msg){
    	new_vel_ = msg->state.velocity;
    	vehicle_speed_ = new_vel_;
	}

    void PoseOnTheRoadCallback(const geometry_msgs::PoseStampedConstPtr & msg)
    {
        pose_on_the_road_ = *msg;


    }
    void GoalOnTheRoadCallback(const geometry_msgs::PoseStampedConstPtr & msg)
    {
        goal_on_the_road_ = *msg;
    }



	void InterSectionPlanning(){


		// Upcoming intersection traffic signal

		// Traffic light

		// Stopline Position

		// Speed Command
	}

	int IntersectionIDmapping(const int intersection_id){
		int new_id = -1;
		if(intersection_id == 100){
			new_id = 11;
		}
        else if(intersection_id == 000){
            new_id = 0;
        }
        else if(intersection_id == 001){
            new_id = 01;
        }
		else if(intersection_id == 200){
			new_id = 2;
		}
		else if(intersection_id == 300){
			new_id = 3;
		}
		else if(intersection_id == 400){
			new_id = 4;
		}
		else if(intersection_id == 500){
			new_id = 5;
		}
		else if(intersection_id == 700){
			new_id = 7;
		}

		return new_id;
	}


	// Load parameters etc
	int init()
	{
		node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
		pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

		DrivigState_pub_ = node_->advertise<pharos_behavior_planner::DrivingState>("/behavior/driving_state",10);
		StopLane_pub_ = node_->advertise<pharos_behavior_planner::StopLane>("/behavior/stoplane",10);
        GoalPose_pub_ = node_->advertise<geometry_msgs::PointStamped>("/behavior/goal_pose",10);
		SideAreaMarker_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/behavior/sidearea",10);
        Area1_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/behavior/Area_array",10);
        Area2_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/behavior/Area2_pub_",10);
        LeftTrun_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/behavior/LeftTrun_array",10);
        StopSignal_pub_ = node_->advertise<std_msgs::Bool>("/behavior/StopSignal",10);
		LaneChangeState_pub_ = node_->advertise<pharos_behavior_planner::LaneChangeState>("/behavior/LaneChangeState", 10);
        Workzone_pub_ = node_->advertise<pharos_msgs::Workzone_list>("/wave/workzone_list",10);
        SCzone_pub_ = node_->advertise<pharos_msgs::Workzone_list>("/wave/sc_list",10);
        global_path_planning_request_pub_ = node_->advertise<std_msgs::Int32>("global_path_planning_request",10);
        BackPoint_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/behavior/BackPoints",10);
        LeftPoint_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/behavior/LeftPoints",10);
        emergency_state_pub_ = node_->advertise<std_msgs::Bool>("/behavior/emergency_state",10);

        workzone_bool = node_->advertise<std_msgs::Bool>("behavior/workzone_bool",10);
        wk_points_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/behavior/WkPoints",10);
        sc_zone_pub_  = node_->advertise<visualization_msgs::MarkerArray>("/behavior/ScPoints",10);
        spat_signal_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/behavior/SpatPoints",10);



		Odom_sub_ = node_->subscribe("/odom/vehicle",10,&BehaviorPlanner::VehiclePoseCallback,this);
		current_roadinfo_sub_ = node_->subscribe("/current_road_info", 10, &BehaviorPlanner::CurrentRoadInfoCallback, this);
		reference_path_sub_ = node_->subscribe("/reference_road_info", 10, &BehaviorPlanner::ReferenceRoadInfoCallback, this);
        Vehicle_sub_ = node_->subscribe<pharos_msgs::StateStamped2016>("/vehicle/state2016",1, &BehaviorPlanner::VehicleStateCallback, this);
        Pose_on_the_road_sub = node_->subscribe<geometry_msgs::PoseStamped>("pose_on_the_road",1, &BehaviorPlanner::PoseOnTheRoadCallback, this);
        Goal_on_the_road_sub = node_->subscribe<geometry_msgs::PoseStamped>("goal_on_the_road",1, &BehaviorPlanner::GoalOnTheRoadCallback, this);
        Tracked_pharos_msgs_sub_ = node_->subscribe<pharos_msgs::ObjectInfoArray>("/vlp_msg/pharos_objectinfo",1, &BehaviorPlanner::PharosTrackedCallback, this);
        mission_state_sub_ = node_->subscribe<std_msgs::Int32Ptr>("mission_state",1,&BehaviorPlanner::MissionStateCallback,this);



        MAP_message_sub_ = node_->subscribe("/wave/map_message",3,&BehaviorPlanner::MAP_messageCallback,this);
        SPAT_message_sub_ = node_->subscribe("/wave/spat_message",3,&BehaviorPlanner::SPAT_DATA_Callback,this);
        BSM_message_sub_ = node_->subscribe("/wave/bsm_message",3,&BehaviorPlanner::BSM_messageCallback,this);
        RTCM_message_sub_ = node_->subscribe("/wave/rtcm_message",3,&BehaviorPlanner::RTCM_messageCallback,this);
        TIM_message_sub_ = node_->subscribe("/wave/tim_message",3,&BehaviorPlanner::TIM_messageCallback,this);


		// vehicle_state_sub_ = node_->subscribe("/vehicle/state2016", 10, &BehaviorPlanner::VehicleStateCallback, this);
		// Get Intersection and Traffic Light Information
		pnode_->getParam("number_of_lanes",number_of_lanes);
		ros::param::get("gps/origin/x", origin_x_);
		ros::param::get("gps/origin/y", origin_y_);
		ros::param::get("gps/origin/z", origin_z_);
/////////////////////////////////////////////////////////////////////////



        GetCallSelecRES_ = false;

		intersection_list.resize(number_of_lanes);

		for(int i=0;i<number_of_lanes;i++){
			int road_id = 0;
			int lane_id = 0;
			int intersection_id = 0;
			int traffic_light_id = 0;

			std::stringstream lane_name;
			lane_name << "lane_" << i+1;


			pnode_->getParam(lane_name.str()+"/road_id",road_id);
			pnode_->getParam(lane_name.str()+"/lane_id",lane_id);
			pnode_->getParam(lane_name.str()+"/intersection_id",intersection_id);
			pnode_->getParam(lane_name.str()+"/traffic_light_id",traffic_light_id);

			intersection_info intersection;
			intersection.road_id = road_id;
			intersection.lane_id = lane_id;
			intersection.intersection_id = intersection_id;
			intersection.traffic_light_id = traffic_light_id;


			intersection_list.at(i) = intersection;
		}

		printf("Intersection yaml load finished!\n");

		traffic_light_list_.resize(16);
		for(unsigned int i=0;i<traffic_light_list_.size();i++){
			traffic_light_list_.at(i).light.resize(16);
		}
		for(unsigned int i=0;i<traffic_light_list_.size();i++){
			for(unsigned int j=0;j<traffic_light_list_.at(i).light.size();j++){
				// traffic_light_list_.at(i).light.at(j).red = true;
			}
		}

		printf("Traffic light list is initialized!\n");

		DrivingState_.situation = SITUATION_LANE_DRIVING;
		DrivingState_.action = NORMAL_DRIVING;

		UnSafetyIntersectionPassing_ = false;
		reference_path_update_ = false;

        stop_time_ = 0;
        old_t_ = ros::Time::now();

        geometry_msgs::Point p400;
        geometry_msgs::Point p500;
        geometry_msgs::Point p700;

        p400.x = spat_400_x-origin_x_;
        p400.y = spat_400_y-origin_y_;
        p500.x = spat_500_x-origin_x_;
        p500.y = spat_500_y-origin_y_;
        p700.x = spat_700_x-origin_x_;
        p700.y = spat_700_y-origin_y_;

        spat_04->points.push_back(p400);
        spat_05->points.push_back(p500);
        spat_07->points.push_back(p700);

        spat_04->header.frame_id = "/odom";
        spat_04->header.stamp = ros::Time::now();
        spat_04->ns = "spat_04";
        spat_04->id = 0;
        spat_04->type = visualization_msgs::Marker::SPHERE_LIST;
        spat_04->action = visualization_msgs::Marker::ADD;
        spat_04->scale.x = 10; //Line width
        spat_04->scale.y = 1;
        spat_04->scale.z = 0.0;
        spat_04->color.a = 0.5;
        spat_04->color.r = 0;
        spat_04->color.g = 0;
        spat_04->color.b = 0;


        spat_05->header.frame_id = "/odom";
        spat_05->header.stamp = ros::Time::now();
        spat_05->ns = "spat_05";
        spat_05->id = 0;
        spat_05->type = visualization_msgs::Marker::SPHERE_LIST;
        spat_05->action = visualization_msgs::Marker::ADD;
        spat_05->scale.x = 10; //Line width
        spat_05->scale.y = 1;
        spat_05->scale.z = 0.0;
        spat_05->color.a = 0.5;
        spat_05->color.r = 0;
        spat_05->color.g = 0;
        spat_05->color.b = 0;


        spat_07->header.frame_id = "/odom";
        spat_07->header.stamp = ros::Time::now();
        spat_07->ns = "spat_07";
        spat_07->id = 0;
        spat_07->type = visualization_msgs::Marker::SPHERE_LIST;
        spat_07->action = visualization_msgs::Marker::ADD;
        spat_07->scale.x = 10; //Line width
        spat_07->scale.y = 1;
        spat_07->scale.z = 0.0;
        spat_07->color.a = 0.5;
        spat_07->color.r = 0;
        spat_07->color.g = 0;
        spat_07->color.b = 0;

        return 0;
	}

	// Publish data
	void publish()
	{
		ros::Rate loop_rate(100);
		while (node_->ok())
		{
			ros::spinOnce();
            loop_rate.sleep();
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "behavior planner");

	BehaviorPlanner behavior_planner_node;
	if (behavior_planner_node.init())
	{
		ROS_FATAL("BehaviorPlanner initialization failed");
		return -1;
	}

	behavior_planner_node.publish();


	return 0;
}

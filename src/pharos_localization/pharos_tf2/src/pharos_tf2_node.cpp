#include <Eigen/Geometry>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <pharos_msgs/SCBADUheader.h>
#include <pharos_msgs/CAN_GWAY_header.h>
#include <pharos_msgs/StateStamped2016.h>


bool bag_;
bool use_old_CAN_format_;

geometry_msgs::TransformStamped tf_stamped_;

typedef struct tf_param_{
    std::string frame_id;
    std::string child_frame_id;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    geometry_msgs::Transform transform;
}tf_param_;


ros::Time GetTime(ros::Time stamp = ros::Time::now()){
    if(bag_) return ros::Time::now();
    else return stamp;
}


void Odometry2TF(tf2_ros::TransformBroadcaster &br, const nav_msgs::OdometryConstPtr &Odom){
    static geometry_msgs::Transform transform;

    // std::cout<<Odom->child_frame_id<<": "<<Odom->header.stamp<<std::endl;

    transform.translation.x = Odom->pose.pose.position.x;
    transform.translation.y = Odom->pose.pose.position.y;
    transform.translation.z = Odom->pose.pose.position.z;
    transform.rotation = Odom->pose.pose.orientation;

    tf_stamped_.header.stamp = GetTime(Odom->header.stamp);
    tf_stamped_.header.frame_id = Odom->header.frame_id;
    tf_stamped_.child_frame_id = Odom->child_frame_id;
    tf_stamped_.transform = transform;

    br.sendTransform(tf_stamped_);
    // std::cout<<Odom->child_frame_id<<std::endl;
}

void UpdateTF(struct tf_param_ &tf_param){
    tf_param.transform.translation.x = tf_param.x;
    tf_param.transform.translation.y = tf_param.y;
    tf_param.transform.translation.z = tf_param.z;

    tf2::Quaternion tf_RM;
    tf_RM.setRPY(tf_param.roll * M_PI/180, tf_param.pitch * M_PI/180, tf_param.yaw * M_PI/180);
    
    tf_param.transform.rotation.x = tf_RM.x();
    tf_param.transform.rotation.y = tf_RM.y();
    tf_param.transform.rotation.z = tf_RM.z();
    tf_param.transform.rotation.w = tf_RM.w();
}

void SCBADUCallback(struct tf_param_ *tf_param, const pharos_msgs::SCBADUheaderConstPtr& msg){
    tf_param[2].yaw = msg->scbadu.steering / 13.4; //gear ratio
    tf_param[3].yaw = tf_param[2].yaw;

    UpdateTF(tf_param[2]);
    UpdateTF(tf_param[3]);
}

void VehicleCallback(struct tf_param_ *tf_param, const pharos_msgs::CAN_GWAY_headerConstPtr& CAN){
    use_old_CAN_format_ = false;

    tf_param[0].yaw = CAN->GWAY2.Steering_Angle / 13.4; //gear ratio
    tf_param[1].yaw = CAN->GWAY2.Steering_Angle / 13.4;

    float rps;
    int reverse = CAN->GWAY3.GearSelDisp==7?-1:1;

    rps = reverse * CAN->GWAY1.Wheel_Velocity_FL / 3.6 / 1.8;
    for(int i=0; i<2; i++){
        tf_param[i].pitch += rps * 360 * 0.01;
        tf_param[i+2].pitch = tf_param[i].pitch;
        tf_param[i+4].pitch = tf_param[i].pitch;
        rps = reverse * CAN->GWAY1.Wheel_Velocity_FR / 3.6 / 1.8;
    }
    for(int i=0; i<6; i++){
        while(tf_param[i].pitch < 0) tf_param[i].pitch += 360;
        while(tf_param[i].pitch > 360) tf_param[i].pitch -= 360;
        UpdateTF(tf_param[i]);
    }
}

void OldVehicleCallback(struct tf_param_ *tf_param, const pharos_msgs::StateStamped2016ConstPtr& CAN){
    if(!use_old_CAN_format_) return;

    tf_param[0].yaw = CAN->state.wheel_angle / 13.4; //gear ratio
    tf_param[1].yaw = CAN->state.wheel_angle / 13.4;

    float rps;
    int reverse = CAN->state.gear==7?-1:1;

    rps = reverse * CAN->state.front_left_wheel / 3.6 / 1.8;
    for(int i=0; i<2; i++){
        tf_param[i].pitch += rps * 360 * 0.01;
        tf_param[i+2].pitch = tf_param[i].pitch;
        tf_param[i+4].pitch = tf_param[i].pitch;
        rps = reverse * CAN->state.front_right_wheel / 3.6 / 1.8;
    }
    for(int i=0; i<6; i++){
        while(tf_param[i].pitch < 0) tf_param[i].pitch += 360;
        while(tf_param[i].pitch > 360) tf_param[i].pitch -= 360;
        UpdateTF(tf_param[i]);
    }
}

void VehicleMarkerInit(visualization_msgs::MarkerArray *Array){
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time();
    marker.id = 1;
    marker.ns = "vehicle";
    marker.header.frame_id = "vehicle_frame";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 1.29;
    marker.pose.position.z = -0.01;
    marker.pose.orientation.z = 1;
    marker.scale.x = 0.0248;
    marker.scale.y = 0.025;
    marker.scale.z = 0.021;
    marker.color.a = 0.2; // Don't forget to set the alpha!
    marker.color.r = 0.2;
    marker.color.g = 0.4;
    marker.color.b = 0.8;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pharos_tf2/config/SantaFe.stl";

    Array->markers.push_back(marker);
    Array->markers[0].id = 1;
}

void WheelMarkerInit(visualization_msgs::MarkerArray *Array){
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time();
    marker.ns = "wheel";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.25;
    marker.color.g = 0.25;
    marker.color.b = 0.25;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pharos_tf2/config/tire.stl";

    for(int i=1; i<7; i++){
        Array->markers.push_back(marker);
        Array->markers[i].id = i;
    }
    Array->markers[1].header.frame_id = "FL_wheel";
    Array->markers[2].header.frame_id = "FR_wheel";

    Array->markers[3].ns = "wheel2";
    Array->markers[4].ns = "wheel2";
    Array->markers[3].header.frame_id = "FL_wheel2";
    Array->markers[4].header.frame_id = "FR_wheel2";
    
    Array->markers[5].header.frame_id = "RL_wheel";
    Array->markers[6].header.frame_id = "RR_wheel";
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pharos_tf_broadcaster");

    ros::NodeHandle node;
    ros::NodeHandle pnode("~");

    pnode.param("bag", bag_, false);

    int rate_;
    pnode.param("rate", rate_, 100);
    ros::Rate rate(rate_);

    int tf_num_;
    pnode.getParam("tf_num", tf_num_);
    tf_param_ *tf_param = new tf_param_[tf_num_];


    static tf2_ros::TransformBroadcaster br;
    static tf2_ros::StaticTransformBroadcaster static_br;

    ros::Subscriber SCBADU_sub = node.subscribe<pharos_msgs::SCBADUheader>("/master/interface", 1, 
        boost::bind(&SCBADUCallback, boost::ref(tf_param), _1));

    ros::Subscriber CAN_GWAY_sub = node.subscribe<pharos_msgs::CAN_GWAY_header>("/CAN_Gateway", 1, 
        boost::bind(&VehicleCallback, boost::ref(tf_param), _1));
    // ros::Subscriber Old_CAN_GWAY_sub = node.subscribe<pharos_msgs::StateStamped2016>("/vehicle/state2016", 1, 
    //     boost::bind(&OldVehicleCallback, boost::ref(tf_param), _1));

    ros::Subscriber Vehicle_Odom_sub = node.subscribe<nav_msgs::Odometry>("/odom/vehicle", 10, 
        boost::bind(&Odometry2TF, boost::ref(br), _1));
    ros::Subscriber Novatel_Odom_sub = node.subscribe<nav_msgs::Odometry>("/odom/novatel", 10, 
        boost::bind(&Odometry2TF, boost::ref(br), _1));
    ros::Subscriber Ublox_Odom_sub = node.subscribe<nav_msgs::Odometry>("/odom/ublox", 10, 
        boost::bind(&Odometry2TF, boost::ref(br), _1));
    ros::Subscriber MCL_Odom_sub = node.subscribe<nav_msgs::Odometry>("/odom/mcl", 10, 
        boost::bind(&Odometry2TF, boost::ref(br), _1));
    ros::Subscriber EKF_Odom_sub = node.subscribe<nav_msgs::Odometry>("/odom/ekf", 10, 
        boost::bind(&Odometry2TF, boost::ref(br), _1));

    ros::Publisher Vehicle_pub = node.advertise<visualization_msgs::MarkerArray>( "/vehicle_marker", 0 );

    for(int i=0; i<tf_num_; i++){
        //loading tf_config.yaml
        std::stringstream tf_name;
        tf_name << "tf_" << i+1;
        pnode.getParam(tf_name.str()+"/frame_id", tf_param[i].frame_id);
        pnode.getParam(tf_name.str()+"/child_frame_id", tf_param[i].child_frame_id);
        pnode.getParam(tf_name.str()+"/x", tf_param[i].x);
        pnode.getParam(tf_name.str()+"/y", tf_param[i].y);
        pnode.getParam(tf_name.str()+"/z", tf_param[i].z);
        pnode.getParam(tf_name.str()+"/roll", tf_param[i].roll);
        pnode.getParam(tf_name.str()+"/pitch", tf_param[i].pitch);
        pnode.getParam(tf_name.str()+"/yaw", tf_param[i].yaw);

        UpdateTF(tf_param[i]);
    }

    visualization_msgs::MarkerArray vehicle_marker;
    VehicleMarkerInit(&vehicle_marker);
    WheelMarkerInit(&vehicle_marker);

    for(int i=0; i<tf_num_; i++){
        tf_stamped_.header.stamp = ros::Time::now();
        tf_stamped_.header.frame_id = tf_param[i].frame_id;
        tf_stamped_.child_frame_id = tf_param[i].child_frame_id;
        tf_stamped_.transform = tf_param[i].transform;

        static_br.sendTransform(tf_stamped_);

    }

    while(ros::ok()){

        for(int i=0; i<6; i++){
            tf_stamped_.header.stamp = ros::Time::now();
            tf_stamped_.header.frame_id = tf_param[i].frame_id;
            tf_stamped_.child_frame_id = tf_param[i].child_frame_id;
            tf_stamped_.transform = tf_param[i].transform;

            static_br.sendTransform(tf_stamped_);
        }

        Vehicle_pub.publish(vehicle_marker);
        // FR_pub.publish(markerFR);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};

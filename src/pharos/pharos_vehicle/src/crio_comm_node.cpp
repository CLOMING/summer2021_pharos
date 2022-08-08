// ROS
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <algorithm>
#include <functional>
#include <cstdlib>

#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <pharos_msgs/CAN_GWAY_header.h>
#include <pharos_msgs/MotionCommandStamped3.h>
#include <std_msgs/Float32.h>
#include <pharos_msgs/LaneOffset.h>
#include "vehicle_kinematics.hpp"

#include <pharos_msgs/SCBADUheader.h>

using boost::asio::ip::udp;

// Default publish rate
const int32_t PUBLISH_RATE = 30; // 100Hz

class CompactRIOCommunicationNode
{
    enum
    {
        max_length = 1024
    };
    char data_[max_length];

public:
    int32_t publish_rate_;
    int32_t socket_timeout_;
    float steer_ratio_;
    std_msgs::Float32 lookahead_dist_;

    ros::NodeHandlePtr node_;
    // Private node for parameters
    ros::NodeHandlePtr pnode_;

    ros::Publisher odom_pub2016_;
    ros::Publisher state_pub2016_;
    ros::Publisher lookahead_dist_pub_;

    ros::Publisher can_gway_pub_;

    // subscribe to control message
    ros::Subscriber sub_;
    ros::Subscriber sub2_;
    ros::Subscriber sub_teleCmd_;
    pharos_msgs::SCBADU scbadu_;
    bool teleCmdOn = false;
    double teleLatency_ = 0.0;

    std::string tf_prefix_;

    // Odometry fixed frame
    std::string fixed_frame_id_;
    // CompactRIO IP address and port
    std::string crio_ip_;
    std::string crio_cmd_port_;
    std::string crio_state_port_;

    geometry_msgs::Pose pose_;
    geometry_msgs::Twist twist_;

    // Transform for odometry
    geometry_msgs::TransformStamped odom_tf2016_;
    boost::shared_ptr<tf::TransformBroadcaster> tf_br_;

    nav_msgs::Odometry odom_msg2016_;
    CAN_GWAY_header CAN;

    boost::asio::io_service io_service_;
    udp::endpoint send_ep_;
    udp::endpoint receive_ep_;
    udp::socket socket_;
    boost::asio::deadline_timer deadline_;
    boost::posix_time::seconds timeout_;
    boost::system::error_code error_;

    boost::shared_ptr<VehicleKinematics> kin_;

    boost::mutex mutex_;

    std::string parent_frame_id;
    std::string vehicle_odom_frame_id;

    bool publish_odom_tf_;

    CompactRIOCommunicationNode() :
            publish_rate_(30), socket_timeout_(10), socket_(io_service_), deadline_(io_service_),
            timeout_(socket_timeout_), publish_odom_tf_(true)
    {
    }

    int init()
    {
      // Use global namespace for node
      node_ = ros::NodeHandlePtr(new ros::NodeHandle());
      // Get tf_prefix from global namespace
      node_->param("tf_prefix", tf_prefix_, std::string(""));

      // Use private namespace for parameters
      pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));
      pnode_->param("publish_rate", publish_rate_, PUBLISH_RATE);

      pnode_->param("fixed_frame_id", fixed_frame_id_, std::string("odom"));
      fixed_frame_id_ = tf::resolve(tf_prefix_, fixed_frame_id_);

      pnode_->param("publish_odom_tf", publish_odom_tf_, true);

      pnode_->param("crio/ip", crio_ip_, std::string("127.0.0.1"));
      pnode_->param("crio/command_port", crio_cmd_port_, std::string("39000"));
      pnode_->param("crio/state_port", crio_state_port_, std::string("39001"));
      pnode_->param("crio/socket_timeout", socket_timeout_, 10);


      pnode_->param<std::string>("parent_frame_id", parent_frame_id, "");
      pnode_->param<std::string>("vehicle_odom_frame_id", vehicle_odom_frame_id, "");


      pnode_->param("teleLatency", teleLatency_, 0.1);


      VehicleKinematics::Parameters kin_params;
      tfScalar minimum_radius_outer;

      if (!pnode_->getParam("kinematics/frame_id", kin_params.frame_id))
      {
        ROS_WARN_STREAM(pnode_->getNamespace() << "/kinematics/frame_id parameter is not set");
      }
      else
      {
        kin_params.frame_id = tf::resolve(tf_prefix_, kin_params.frame_id);
      }

      if (!pnode_->getParam("kinematics/wheelbase", kin_params.wheelbase_length))
      {
        ROS_FATAL_STREAM(pnode_->getNamespace() << "/kinematics/wheelbase parameter is not set");
        return -1;
      }
      if (!pnode_->getParam("kinematics/track", kin_params.track_width))
      {
        ROS_FATAL_STREAM( pnode_->getNamespace() << "/kinematics/track parameter is not set");
        return -1;
      }
      if (!pnode_->getParam("kinematics/rotation_center", kin_params.rotation_center))
      {
        ROS_WARN_STREAM(
                pnode_->getNamespace()
                << "/kinematics/rotation_center parameter is not set. Using default: wheelbase/2 = "
                << kin_params.wheelbase_length / 2);
        kin_params.rotation_center = kin_params.wheelbase_length / 2;
      }
      if (!pnode_->getParam("kinematics/minimum_radius_outer", minimum_radius_outer))
      {
        ROS_FATAL_STREAM(pnode_->getNamespace() << "/kinematics/minimum_radius_outer parameter is not set");
        return -1;
      }
      else
      {
        kin_params.minimum_radius = minimum_radius_outer - kin_params.track_width / 2;
      }
      if (!pnode_->getParam("kinematics/steering_ratio", kin_params.steering_ratio))
      {
        ROS_FATAL_STREAM(pnode_->getNamespace() << "/kinematics/steering_ratio parameter is not set");
        return -1;
      }

      kin_ = boost::make_shared<VehicleKinematics>(kin_params);

      sub_ = node_->subscribe<pharos_msgs::MotionCommandStamped3>("motion_command", 200, &CompactRIOCommunicationNode::motionCommand, this);
      sub2_ = node_->subscribe<pharos_msgs::LaneOffset>("lane_offset", 200, &CompactRIOCommunicationNode::steering_ratio_callback, this);
      sub_teleCmd_ = node_->subscribe<pharos_msgs::SCBADUheader>("master/interface", 1, &CompactRIOCommunicationNode::teleCommand, this, ros::TransportHints().udp());

      odom_pub2016_ = node_->advertise<nav_msgs::Odometry>("odometry2016", 200);
      state_pub2016_ = node_->advertise<pharos_msgs::StateStamped2016>("state2016", 200);
      can_gway_pub_ = node_->advertise<pharos_msgs::CAN_GWAY_header>("CAN_Gateway", 200);
      lookahead_dist_pub_ = node_->advertise<std_msgs::Float32>("lookahead_dist", 200);

      tf_br_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster);

      timeout_ = boost::posix_time::seconds(socket_timeout_);
      send_ep_ = udp::endpoint(boost::asio::ip::address::from_string(crio_ip_), std::atoi(crio_cmd_port_.c_str()));
      receive_ep_ = udp::endpoint(udp::v4(), std::atoi(crio_state_port_.c_str()));

      socket_.open(udp::v4());

      deadline_.expires_at(boost::posix_time::pos_infin);
      this->deadlineCallback(deadline_, socket_);

      return 0;
    }

    void deadlineCallback(boost::asio::deadline_timer& t, udp::socket& s)
    {
      if (t.expires_at() <= boost::asio::deadline_timer::traits_type::now())
      {
        s.cancel();
        t.expires_at(boost::posix_time::pos_infin);
      }
      t.async_wait(boost::bind(&CompactRIOCommunicationNode::deadlineCallback, this, boost::ref(t), boost::ref(s)));
    }

    void handleRead(const boost::system::error_code& ec, std::size_t ln)
    {
      ros::Time cur_time = ros::Time::now();

      error_ = ec;
      if (!socket_.is_open())
        return;

      if (!ec)
      {
        std::string msg(data_);
        std::istringstream CANstring2019;
        std::string gway1check;
        std::string gway2check;
        std::string gway3check;
        std::string gway4check;
        std::string ETCcheck;


        if (!msg.empty())
        {
          boost::mutex::scoped_lock lock(mutex_);

          CAN.header.stamp = cur_time;
          CAN.header.frame_id = "i30";

          // // GWAY1
          // CAN.GWAY1.Wheel_Velocity_FR = 0.0;
          // CAN.GWAY1.Wheel_Velocity_RL = 0.0;
          // CAN.GWAY1.Wheel_Velocity_RR = 0.0;
          // CAN.GWAY1.Wheel_Velocity_FL = 0.0;
          // // GWAY2
          // CAN.GWAY2.Lateral_Accel_Speed = 0.0;
          // CAN.GWAY2.Parking_Brake_Active = 0.0;
          // CAN.GWAY2.AirConditioner_On = 0.0;
          // CAN.GWAY2.Steering_Angle = 0.0;
          // CAN.GWAY2.Steering_Speed = 0.0;
          // CAN.GWAY2.Steering_Tq = 0.0;
          // // GWAY3
          // CAN.GWAY3.Accel_Pedal_Position = 0.0;
          // CAN.GWAY3.Brake_Active = 0.0;
          // CAN.GWAY3.BrakeMasterCylinder_Pressure = 0.0;
          // CAN.GWAY3.Engine_Speed = 0.0;
          // CAN.GWAY3.Gear_Target_Change = 0.0;
          // CAN.GWAY3.GearSelDisp = 0.0;
          // CAN.GWAY3.Throttle_Position = 0.0;
          // // GWAY4
          // CAN.GWAY4.Cluster_Odometer = 0.0;
          // CAN.GWAY4.Longitudinal_Accel_Speed = 0.0;
          // CAN.GWAY4.Vehicle_Speed_Engine = 0.0;
          // CAN.GWAY4.Yaw_Rate_Sensor = 0.0;
          // // GWAY5
          // CAN.GWAY5.Input_Error7 = 0;
          // CAN.GWAY5.Input_Error6 = 0;
          // CAN.GWAY5.Input_Error5 = 0;
          // CAN.GWAY5.Input_Error4 = 0;
          // CAN.GWAY5.Input_Error3 = 0;
          // CAN.GWAY5.Input_Error2 = 0;
          // CAN.GWAY5.Input_Error1 = 0;
          // CAN.GWAY5.Input_Error0 = 0;
          // // ETC
          double emergency_state = 0;
          double mission_state = 0;

          // For the state2016
          CANstring2019.str(msg);
          // printf("%s\n", msg.c_str());
          CANstring2019
          >> gway1check
          >> CAN.GWAY1.Wheel_Velocity_FR
          >> CAN.GWAY1.Wheel_Velocity_RL
          >> CAN.GWAY1.Wheel_Velocity_RR
          >> CAN.GWAY1.Wheel_Velocity_FL
          >> gway2check
          >> CAN.GWAY2.Lateral_Accel_Speed
          >> CAN.GWAY2.Parking_Brake_Active
          >> CAN.GWAY2.AirConditioner_On
          >> CAN.GWAY2.Steering_Angle
          >> CAN.GWAY2.Steering_Speed
          >> CAN.GWAY2.Steering_Tq
          >> gway3check
          >> CAN.GWAY3.Accel_Pedal_Position
          >> CAN.GWAY3.Brake_Active
          >> CAN.GWAY3.BrakeMasterCylinder_Pressure
          >> CAN.GWAY3.Engine_Speed
          >> CAN.GWAY3.Gear_Target_Change
          >> CAN.GWAY3.GearSelDisp
          >> CAN.GWAY3.Throttle_Position
          >> gway4check
          >> CAN.GWAY4.Cluster_Odometer
          >> CAN.GWAY4.Longitudinal_Accel_Speed
          >> CAN.GWAY4.Vehicle_Speed_Engine
          >> CAN.GWAY4.Yaw_Rate_Sensor
          >> ETCcheck
          >> emergency_state
          >> mission_state
          >> lookahead_dist_.data;
          // >> CAN.GWAY5.Input_Error7 >> CAN.GWAY5.Input_Error6 >> CAN.GWAY5.Input_Error5 >> CAN.GWAY5.Input_Error4 >> CAN.GWAY5.Input_Error3 >> CAN.GWAY5.Input_Error2 >> CAN.GWAY5.Input_Error1 >> CAN.GWAY5.Input_Error0;
		      // printf("lookahead : %.3f, ,lat_acc:%.3f,gear: %.3f, emergency: %.3f\n",lookahead_dist,lat_acc,gear_position, emergency_state);

          // std::cout<<gway1check<<gway2check<<gway3check<<gway4check<<ETCcheck<<"\n";

          double velocity = (CAN.GWAY1.Wheel_Velocity_FL + CAN.GWAY1.Wheel_Velocity_FR
            + CAN.GWAY1.Wheel_Velocity_RL + CAN.GWAY1.Wheel_Velocity_RR)/4/3.6;
          double wheel_angle = CAN.GWAY2.Steering_Angle / 13.4 * M_PI / 180.0;
          double lat_acc = CAN.GWAY2.Lateral_Accel_Speed;
          double lon_acc = CAN.GWAY4.Longitudinal_Accel_Speed;
          double yaw_rate = CAN.GWAY4.Yaw_Rate_Sensor;
          double front_left_wheel = CAN.GWAY1.Wheel_Velocity_FL;
          double front_right_wheel = CAN.GWAY1.Wheel_Velocity_FR;
          double rear_left_wheel = CAN.GWAY1.Wheel_Velocity_RL;
          double rear_right_wheel = CAN.GWAY1.Wheel_Velocity_RR;
          //double lookahead_dist = 0.0;
          double gear_position = CAN.GWAY3.GearSelDisp; // 1-P, 2-R, 3-N, 4-D
          // Emergency: 1 - ok, 0 - fault


          //------Vehicle Kinematic Odometry
          kin_->updateState_2016(cur_time, velocity, front_left_wheel, front_right_wheel, rear_left_wheel,rear_right_wheel,wheel_angle, lat_acc, lon_acc, yaw_rate, gear_position, emergency_state); // new state

          //--------------Odometry
          nav_msgs::Odometry vehicle_odom_msg;
          VehicleKinematics::stateStampedMsgToOdometryMsg_2016(kin_->getState_2016(), vehicle_odom_msg);

          // Resolve frame names ver 2016----
          vehicle_odom_msg.header.frame_id = tf::resolve(tf_prefix_, vehicle_odom_msg.header.frame_id);
          vehicle_odom_msg.child_frame_id = tf::resolve(tf_prefix_, vehicle_odom_msg.child_frame_id);

          odom_tf2016_.header = vehicle_odom_msg.header;
          odom_tf2016_.child_frame_id = vehicle_odom_msg.child_frame_id;

          odom_tf2016_.transform.translation.x = vehicle_odom_msg.pose.pose.position.x;
          odom_tf2016_.transform.translation.y = vehicle_odom_msg.pose.pose.position.y;
          odom_tf2016_.transform.translation.z = vehicle_odom_msg.pose.pose.position.z;

          odom_tf2016_.transform.rotation = vehicle_odom_msg.pose.pose.orientation;

          odom_msg2016_ = vehicle_odom_msg;
        }
        else
        {
          ROS_WARN_STREAM("Empty message");
        }
      }
    }

    void steering_ratio_callback(const pharos_msgs::LaneOffsetConstPtr& cmd)
    {
        steer_ratio_ = cmd->steering_ratio;
    }

    void teleCommand(const pharos_msgs::SCBADUheaderConstPtr& cmd)
    {
      std::cout<<cmd->scbadu.steering<<"\t"<<cmd->scbadu.accel<<"\t"<<cmd->scbadu.brake<<std::endl;
      if( ros::Time::now() > cmd->header.stamp + ros::Duration(teleLatency_) )
      {
        ROS_WARN("TeleCmd Latancy Warn(%.0f ms)", (ros::Time::now() - cmd->header.stamp).toSec()*1000);
        teleCmdOn = false;
      }else{
        teleCmdOn = true;
      }

      scbadu_ = cmd->scbadu;


      std::ostringstream ss;

      ss.unsetf(std::ios::floatfield);
      // set %7.7f
      ss.precision(7);
      ss.width(7);

      ss << 0.0
      << " " << 0.0 // angle_difference
      << " " << 0.0 // curvature
      << " " << 0.0
      << " " << 0.0
      << " " << 0.0
      << " " << 0.0 // current x
      << " " << 0.0 // current y
      << " " << 0.0 // waypoint x
      << " " << 0.0
      << " " << 0.0 // second lateral offset
      << " " << 0.0
      << " " << 0.0
      << " " << teleCmdOn
      << " " << scbadu_.steering
      << " " << scbadu_.accel
      << " " << scbadu_.brake
      << "\n";
      // printf("ratio : %f\n",steer_ratio_);
      // printf("velocity: %.3f \n",cmd->velocity_limit);

      try
      {
        socket_.send_to(boost::asio::buffer(std::string(ss.str())), send_ep_);
      }
      catch (std::exception& e)
      {
        ROS_ERROR_STREAM(e.what());
      }

    }

    void motionCommand(const pharos_msgs::MotionCommandStamped3ConstPtr& cmd)
    {
      std::ostringstream ss;

      ss.unsetf(std::ios::floatfield);
      // set %7.7f
      ss.precision(7);
      ss.width(7);

      ss << cmd->lateral_offset
      << " " << cmd->heading_difference // angle_difference
      << " " << cmd->curvature // curvature
      << " " << cmd->velocity_limit
      << " " << cmd->goal_distance
      << " " << cmd->inclination
      << " " << 0.0 // current x
      << " " << 0.0 // current y
      << " " << 0.0 // waypoint x
      << " " << steer_ratio_
      << " " << cmd->lateral_offset_origin // second lateral offset
      << " " << cmd->sudden_stop
      << " " << cmd->mission_state
      << " " << teleCmdOn
      << " " << scbadu_.steering
      << " " << scbadu_.accel
      << " " << scbadu_.brake
      << "\n";
      // printf("ratio : %f\n",steer_ratio_);
      // printf("velocity: %.3f \n",cmd->velocity_limit);

      try
      {
        socket_.send_to(boost::asio::buffer(std::string(ss.str())), send_ep_);
      }
      catch (std::exception& e)
      {
        ROS_ERROR_STREAM(e.what());
      }

    }


    void readCRIO()
    {
      udp::socket socket2(io_service_, receive_ep_);
      while (node_->ok())
      {
        try
        {
          deadline_.expires_from_now(timeout_);
          error_ = boost::asio::error::would_block;

          socket2.async_receive_from(boost::asio::buffer(data_, max_length), receive_ep_,
                                     boost::bind(&CompactRIOCommunicationNode::handleRead, this, _1, _2));

          do
          {
            io_service_.run_one();
          } while (error_ == boost::asio::error::would_block);

          if (error_)
          {
            if (error_ == boost::asio::error::operation_aborted)
            {
              ROS_WARN("Socket receive timed out");
            }
            else
            {
              throw boost::system::system_error(error_);
            }
          }
        }
        catch (std::exception& e)
        {
          ROS_ERROR_STREAM(e.what());
        }
      }
    }

    void publish()
    {
      ros::Rate loop_rate(publish_rate_);
      ros::AsyncSpinner spinner(2);
      // handle communication with cRIO in separate thread
      boost::thread crio_read_thread(&CompactRIOCommunicationNode::readCRIO, this);

      spinner.start();

      while (node_->ok())
      {
        // lock variables from being modified
        // by cRIO communication thread
        {
          boost::mutex::scoped_lock lock(mutex_);

          //TF publish
          // static tf::TransformBroadcaster br;
          // tf::Transform transform;
          // transform.setOrigin(tf::Vector3(odom_msg2016_.pose.pose.position.x,odom_msg2016_.pose.pose.position.y,0));

          // tf::Quaternion q;
          // tf::Pose pose;
          // tf::poseMsgToTF(odom_msg2016_.pose.pose, pose);
          // double Roll,Pitch, Yaw;
          // pose.getBasis().getRPY(Roll, Pitch, Yaw);
          // q.setRPY(0, 0, Yaw);
          // transform.setRotation(q);

          // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_id, vehicle_odom_frame_id));

          //odom_pub2016_.publish(odom_msg2016_);
          state_pub2016_.publish(kin_->getState_2016());
          can_gway_pub_.publish(CAN);
          lookahead_dist_pub_.publish(lookahead_dist_);
        }
        //    ros::spinOnce();
        loop_rate.sleep();
      }
      crio_read_thread.join();
    }
};

int main(int argc, char** argv)
{
  //-- Init ROS node
  ros::init(argc, argv, "crio_comm");
  CompactRIOCommunicationNode crio_ros;

  if(!crio_ros.init()) crio_ros.publish();
  else return -1;

  return 0;
}

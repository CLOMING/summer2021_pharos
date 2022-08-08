/*
 * vehicle_kinematics.hpp
 *
 *  Created on: May 31, 2013
 *      Author: 0xff
 */

#ifndef VEHICLE_KINEMATICS_HPP_
#define VEHICLE_KINEMATICS_HPP_

#include <pharos_msgs/StateStamped2016.h>

using namespace pharos_msgs;

class VehicleKinematics
{
public:
    typedef struct Parameters_
    {
        // Frame ID of c.g. (or center of rotation) of the car
        std::string frame_id;
        // Wheelbase length
        tfScalar wheelbase_length;
        // Track width
        tfScalar track_width;
        // Distance from front axle to c.r.
        tfScalar rotation_center;
        // Minimum turning radius (average)
        tfScalar minimum_radius;
        // Steering wheel to wheels rotation ratio
        tfScalar steering_ratio;

        friend std::ostream& operator<<(std::ostream& os, const Parameters_& p)
        {
          os << "Constant parameters: " << "\n" << "\t" << "frame_id: " << p.frame_id << "\n" << "\t" << "wheelbase: "
          << p.wheelbase_length << "\n" << "\t" << "track: " << p.track_width << "\n" << "\t" << "rotation_center: "
          << p.rotation_center << "\n" << "\t" << "minimum_radius_avg: " << p.minimum_radius << "\n" << "\t"
          << "steering_ratio: " << p.steering_ratio << "\n";

          return os;
        }
    } Parameters;

private:
    //StateStamped state_;
    //StateStamped2 state2_;
    StateStamped2016 state_2016_;

    Parameters params_;
public:
    VehicleKinematics(const VehicleKinematics::Parameters p) :
            params_(p)
    {
      state_2016_.header.frame_id = params_.frame_id;
    }

    VehicleKinematics(const pharos_msgs::StateStamped2016& s, const VehicleKinematics::Parameters& p) :
            state_2016_(s), params_(p)
    {
      state_2016_.header.frame_id = params_.frame_id;
    }

    // Set new parameters. Resets the state variables
    inline void setParams(const VehicleKinematics::Parameters& p)
    {
      params_ = p;
      // reset the state as it will be inconsistent from now
      state_2016_ = StateStamped2016();
      state_2016_.header.frame_id = params_.frame_id;
    }

    inline const Parameters& getParams()
    {
      return params_;
    }

//  inline const StateStamped& getState()
//  {
//    return state_;
//  }

//  inline const StateStamped2& getState_new()
//  {
//    return state2_;
//  }
    inline const StateStamped2016& getState_2016()
    {
      return state_2016_;
    }
    inline void setState(const pharos_msgs::StateStamped2016& s)
    {
      state_2016_ = s;
      ROS_WARN_COND(params_.frame_id != state_2016_.header.frame_id,
                    "Setting new frame_id: '%s'", state_2016_.header.frame_id.c_str());
    }

    // Reset all state variables to zero
    inline void resetState()
    {
      std::string frame_id;

      state_2016_ = StateStamped2016();
      state_2016_.header.frame_id = frame_id;

      ROS_WARN_COND(
              params_.frame_id != state_2016_.header.frame_id,
              "state.frame_id is not equal to params.frame_id: '%s' vs '%s'", state_2016_.header.frame_id.c_str(), params_.frame_id.c_str());
    }
/*
  inline void updateState(const ros::Time& time_stamp, const tfScalar& velocity, const tfScalar& steering_angle,
                          const tfScalar& lateral_acc, const tfScalar& longitudal_acc, const tfScalar& yaw_rate,
                          const bool& emergency_state)
  {
    State s = state_.state;
	//  State s2 = state_.state;

	Parameters p = params_;
    ros::Duration dt = time_stamp - state_.header.stamp;

    // necessary for first iteration
//    if (dt == ros::Duration(0))
    if (state_.header.stamp == ros::Time(0))
    {
      state_.header.stamp = time_stamp;
      return;
    }

    // Distance from rear axle to c.r.
    tfScalar Lr = p.wheelbase_length - p.rotation_center;
    tfScalar R_1;

    //s.left_wheel = left_wheel;
    //s.right_wheel = right_wheel;
    //s.gear = gear_position;

    s.lat_acc = lateral_acc;
    s.lon_acc = longitudal_acc;
    s.yaw_rate = yaw_rate;

    s.emergency = emergency_state;

    s.velocity = velocity;
    s.theta = steering_angle;
    s.beta = atan2(Lr * tan(s.theta), p.wheelbase_length);
    // 1 / R
    R_1 = cos(s.beta) * tan(s.theta) / p.wheelbase_length;
    // if steering angle is 0 then turning radius is R = inf;
    s.radius = 1 / R_1;
    // angular velocity around c.r.
    s.psi_dot = s.velocity * R_1;
    // integrate angular velocity
    s.psi += s.psi_dot * dt.toSec();
    s.x_dot = s.velocity * cos(s.psi + s.beta);
    s.y_dot = s.velocity * sin(s.psi + s.beta);
    // integrate Cartesian velocities
    s.x += s.x_dot * dt.toSec();
    s.y += s.y_dot * dt.toSec();

    // update state variable
    state_.header.stamp = time_stamp;
    state_.state = s;
  }
  */
/*
  inline void updateState_new(const ros::Time& time_stamp, const tfScalar& velocity, const tfScalar& steering_angle,
                          const tfScalar& left_wheel, const tfScalar& right_wheel, const tfScalar& yaw_rate, const int& gear_position,
                          const bool& emergency_state)
  {
    State2 s2 = state2_.state;

	Parameters p = params_;
    ros::Duration dt = time_stamp - state2_.header.stamp;

    // necessary for first iteration
//    if (dt == ros::Duration(0))
    if (state2_.header.stamp == ros::Time(0))
    {
    	state2_.header.stamp = time_stamp;
      return;
    }

    // Distance from rear axle to c.r.
    tfScalar Lr = p.wheelbase_length - p.rotation_center;
    tfScalar R_1;
    tfScalar r;

    s2.velocity_can = velocity;
    s2.steering_angle = steering_angle;

    s2.left_wheel = left_wheel;
    s2.right_wheel = right_wheel;
    s2.velocity_wheel = (s2.left_wheel + s2.right_wheel)/2;

    s2.angular_velocity = (s2.right_wheel - s2.left_wheel)/p.track_width;
    s2.heading += s2.angular_velocity * dt.toSec();

    s2.x_dot = s2.velocity_wheel*cos(s2.heading);
    s2.y_dot = s2.velocity_wheel*sin(s2.heading);

    r = p.wheelbase_length/s2.angular_velocity;

    s2.x += s2.x_dot * dt.toSec();
    s2.y += s2.y_dot * dt.toSec();

    s2.yaw_rate = yaw_rate;
    s2.gear = gear_position;
    s2.emergency = emergency_state;

    // update state variable
    state2_.header.stamp = time_stamp;
    state2_.state = s2;
  }*/

    inline void updateState_2016(const ros::Time& time_stamp, const tfScalar& velocity, const tfScalar& front_left_wheel_velocity, const tfScalar& front_right_wheel_velocity, const tfScalar& rear_left_wheel_velocity, const tfScalar& rear_right_wheel_velocity,const tfScalar& wheel_angle,
                                 const tfScalar& lateral_acc, const tfScalar& longitudal_acc, const tfScalar& yaw_rate, const int& gear_position,
                                 const bool& emergency_state)
    {
      State2016 s_2016 = state_2016_.state;

      //  State s2 = state_.state;

      Parameters p = params_;
      ros::Duration dt = time_stamp - state_2016_.header.stamp;

      // necessary for first iteration
//    if (dt == ros::Duration(0))
      if (state_2016_.header.stamp == ros::Time(0))
      {
        state_2016_.header.stamp = time_stamp;
        return;
      }

      // Distance from rear axle to c.r.
      tfScalar Lr = p.wheelbase_length - p.rotation_center;
      tfScalar R_1;
      tfScalar beta;
      //s.left_wheel = left_wheel;
      //s.right_wheel = right_wheel;
      //s.gear = gear_position;
      s_2016.front_left_wheel = front_left_wheel_velocity;
      s_2016.front_right_wheel = front_right_wheel_velocity;
      s_2016.rear_left_wheel = rear_left_wheel_velocity;
      s_2016.rear_right_wheel = rear_right_wheel_velocity;

      s_2016.lat_acc = lateral_acc;
      s_2016.lon_acc = longitudal_acc;
      s_2016.yaw_rate = yaw_rate;
      s_2016.gear = gear_position;
      s_2016.emergency = emergency_state;

      s_2016.velocity = velocity;
      s_2016.wheel_angle = wheel_angle;
      beta = atan2(Lr * tan(s_2016.wheel_angle), p.wheelbase_length);
      // 1 / R
      R_1 = cos(beta) * tan(s_2016.wheel_angle) / p.wheelbase_length;
      // if steering angle is 0 then turning radius is R = inf;
      s_2016.turning_radius = 1 / R_1;
      // angular velocity around c.r.

      //s_2016.angular_velocity = s_2016.velocity * R_1;
      s_2016.angular_velocity = s_2016.velocity/p.wheelbase_length*tan(wheel_angle);
      // integrate angular velocity
      s_2016.heading += s_2016.angular_velocity * dt.toSec();
      //s_2016.x_dot = s_2016.velocity * cos(s_2016.heading + beta);
      //s_2016.y_dot = s_2016.velocity * sin(s_2016.heading + beta);
      s_2016.x_dot = s_2016.velocity * cos(s_2016.heading + beta);
      s_2016.y_dot = s_2016.velocity * sin(s_2016.heading + beta);

      // integrate Cartesian velocities
      s_2016.x += s_2016.x_dot * dt.toSec();
      s_2016.y += s_2016.y_dot * dt.toSec();

      // update state variable
      state_2016_.header.stamp = time_stamp;
      state_2016_.state = s_2016;
    }
/*
  static inline void stateMsgToOdometryMsg(const pharos_vehicle::State& s, nav_msgs::Odometry& msg)
  {
    tf::Quaternion q;
    geometry_msgs::Quaternion q_msg;

//    msg.header.stamp = s.stamp;

    msg.pose.pose.position.x = s.x;
    msg.pose.pose.position.y = s.y;
    msg.pose.pose.position.z = 0.0;

    q.setRPY(0.0, 0.0, s.psi);
    tf::quaternionTFToMsg(q, q_msg);
    msg.pose.pose.orientation = q_msg;

    // msg.pose.covariance;

    msg.twist.twist.linear.x = s.x_dot;
    msg.twist.twist.linear.y = s.y_dot;
    msg.twist.twist.linear.z = 0.0;

    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = s.psi_dot;

    // msg.twist.covariance;
  }
  */
    /*
  static inline void stateMsgToOdometryMsg_new(const pharos_vehicle::State2& s, nav_msgs::Odometry& msg)
  {
    tf::Quaternion q;
    geometry_msgs::Quaternion q_msg;

//    msg.header.stamp = s.stamp;

    msg.pose.pose.position.x = s.x;
    msg.pose.pose.position.y = s.y;
    msg.pose.pose.position.z = 0.0;

    q.setRPY(0.0, 0.0, s.heading);
    tf::quaternionTFToMsg(q, q_msg);
    msg.pose.pose.orientation = q_msg;

    // msg.pose.covariance;

    msg.twist.twist.linear.x = s.x_dot;
    msg.twist.twist.linear.y = s.y_dot;
    msg.twist.twist.linear.z = 0.0;

    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = s.angular_velocity;

    // msg.twist.covariance;
  }*/

    static inline void stateMsgToOdometryMsg_2016_steering_base(const pharos_msgs::State2016& s, nav_msgs::Odometry& msg)
    {
      tf::Quaternion q;
      geometry_msgs::Quaternion q_msg;

//    msg.header.stamp = s.stamp;

      msg.pose.pose.position.x = s.x;
      msg.pose.pose.position.y = s.y;
      msg.pose.pose.position.z = 0.0;

      q.setRPY(0.0, 0.0, s.heading);
      tf::quaternionTFToMsg(q, q_msg);
      msg.pose.pose.orientation = q_msg;

      // TODO
      // msg.pose.covariance;

      msg.twist.twist.linear.x = s.x_dot;
      msg.twist.twist.linear.y = s.y_dot;
      msg.twist.twist.linear.z = 0.0;

      msg.twist.twist.angular.x = 0.0;
      msg.twist.twist.angular.y = 0.0;
      msg.twist.twist.angular.z = s.angular_velocity;

      // TODO
      // msg.twist.covariance;
    }


    /*
  static inline void stateStampedMsgToOdometryMsg(const pharos_vehicle::StateStamped& s,
                                                  nav_msgs::Odometry& msg,
                                                  const std::string& frame_id = std::string("odom"))
  {
    stateMsgToOdometryMsg(s.state, msg);
    msg.header = s.header;

    // Odometry child_frame_id is the frame attached to odometry sensor and
    // frame_id is the fixed frame where odometry data is zero
    msg.header.frame_id = frame_id;
    msg.child_frame_id = s.header.frame_id;
  }
*/
    /*
  static inline void stateStampedMsgToOdometryMsg_new(const pharos_vehicle::StateStamped2& s,
                                                  nav_msgs::Odometry& msg,
                                                  const std::string& frame_id = std::string("odom"))
  {
    stateMsgToOdometryMsg_new(s.state, msg);
    msg.header = s.header;

    // Odometry child_frame_id is the frame attached to odometry sensor and
    // frame_id is the fixed frame where odometry data is zero
    msg.header.frame_id = frame_id;
    msg.child_frame_id = s.header.frame_id;
  }
*/
    static inline void stateStampedMsgToOdometryMsg_2016(const pharos_msgs::StateStamped2016& s,
                                                         nav_msgs::Odometry& msg,
                                                         const std::string& frame_id = std::string("odom"))
    {
      stateMsgToOdometryMsg_2016_steering_base(s.state, msg);
      msg.header = s.header;

      // Odometry child_frame_id is the frame attached to odometry sensor and
      // frame_id is the fixed frame where odometry data is zero
      msg.header.frame_id = frame_id;
      msg.child_frame_id = s.header.frame_id;
    }
};

#endif /* VEHICLE_KINEMATICS_HPP_ */

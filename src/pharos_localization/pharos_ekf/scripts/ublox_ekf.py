#!/usr/bin/env python3
import numpy as np
from math import *

import rospy
import roslib
import tf
import PyKDL as kdl
# Messages
from nav_msgs.msg import Odometry
from ublox_msgs.msg import NavVELNED
# from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from pharos_msgs.msg import CAN_GWAY_header as VehicleState
from pharos_msgs.msg import MotionCommandStamped3
from pharos_msgs.msg import LaneOffset
from tf import transformations
from std_msgs.msg import Int32

from sensor_msgs.msg import Imu

import copy
import math
import time

###############################################################################
# def TicTocGenerator():
#     # Generator that returns time differences
#     ti = 0           # initial time
#     tf = time.time() # final time
#     while True:
#         ti = tf
#         tf = time.time()
#         yield tf-ti # returns the time difference

# TicToc = TicTocGenerator() # create an instance of the TicTocGen generator

# # This will be the main function through which we define both tic() and toc()
# def toc(tempBool=True):
#     # Prints the time difference yielded by generator instance TicToc
#     tempTimeInterval = next(TicToc)
#     if tempBool:
#         print( "Elapsed time: %f seconds.\n" %tempTimeInterval )

# def tic():
#     # Records a time in TicToc, marks the beginning of a time interval
#     toc(False)
#################################################################################


def normalizeHeading(heading):
    # print("tuned x:", heading)
    if heading > math.pi:
        heading -= 2*math.pi
    elif heading < -math.pi:
        heading += 2*math.pi
    # print("tuned o:", heading)
    return heading

class RosGpsEkf:
    """docstring for RosGpsEkf"""
    gps_ekf_odom_pub = rospy.Publisher('/odom/ekf', Odometry, queue_size=1)

    relative_gps = False
    publish_odom_tf = True

    frame_id = ''
    child_frame_id = ''


    def __init__(self):
        self.ekf_reset()

    def ekf_reset(self):
        print('enter reset')
        # EKF variables
        self.stop_index = True
        self.heading_init = False
        self.Q = np.mat(np.diag([0.01, 0.01, 0.01, 0.1]))
        self.Q_init = copy.deepcopy(self.Q)
        self.P = np.mat(np.diag([0.5, 0.5, 10, 0]))
        # self.P = np.mat(np.diag([0, 0, 0]))
        self.R = np.mat(np.diag([5.0, 5.0, 0.5, 10.0]))
        self.R_init = copy.deepcopy(self.R)

        # self.R = np.mat(np.diag([2, 2, 3]))

        self.F = np.mat(np.diag([1.0, 1.0, 1.0, 1.0]))
        self.H = np.mat(np.diag([1, 1, 1, 0]))
        self.X = np.zeros((4, 1))
        # self.old_X = np.mat('0.0; 0.0; 0.0; 0.0')

    # initialize variables
        self.x_meas, self.y_meas, self.theta_meas = [0.0] * 3
        # self.stop_x, self.stop_y, self.x_bias, self.y_bias = [0.0] * 4
        #self.last_time = rospy.get_rostime()
        self.filter_initialized = False
        self.old_gps = np.mat([0, 0, 0, 0]).T
        self.new_gps = np.mat([0, 0, 0, 0]).T
        # self.particle_odom = [0.0, 0.0, 0.0]
        self.old_vel = 0.0
        self.old_alpha = 0.0
        self.back = False
        # self.restart = False
        self.L = 0
        # self.delta = 0
        self.inclination = 0
        # self.current_wp_index = 1000

        self.origin_gps_x = 0.0
        self.origin_gps_y = 0.0

        # self.lane_offset = 0.0
        # self.heading_difference = 0.0
        # self.lateral_offset = 0.0
        # self.curvature = 0.0

        self.covariance = np.zeros(16)


        self.steer_bias = 15.0
        self.azimuth = 0.0
        self.yaw_rate = 0.0

        self.Y = []
        self.vscount = 0
        self.time4bagplay = 0
        self.use_const_ratio = True
        self.stear_const = 13.4
        self.alpha_gain = 1
        self.vel_gain = 1.0
        self.theta_gain = 0.7
        self.cov_a = 0.2

        self.imu_init = False;
        self.prev_stamp = rospy.Time.now();
        self.prev_stamp2 = rospy.Time.now();
        self.prev_stamp3 = rospy.Time.now();

        self.Brake_Active = 1
        self.Brake_Cnt = 0

        self.last_time_gps = rospy.get_rostime()
        self.last_time_vehicle = rospy.get_rostime() 
        self.veh_cnt = 0
        self.rpy_cnt = 0
        self.old_time = rospy.get_rostime()
        self.new_time = rospy.get_rostime()

        self.current_gps_time = rospy.get_rostime()
        self.current_vehilcle_time = rospy.get_rostime()
        self.gps_cycle = 0
        self.vehicle_cycle = 0

        print('init')

    def bicycle_model(self, x, y, theta, z, vel, alpha, dt):
        L = 2.65
        d = vel*self.vel_gain * dt
        beta = d / L * tan(alpha*self.alpha_gain)
        delta = self.inclination
        # gamma = d / L / cos(alpha)
        x = x + d * cos(theta)*cos(delta)
        y = y + d * sin(theta)*cos(delta)
        z = z + d * sin(delta)

        if(self.imu_init):
            print("use imu", dt)
            theta = theta + self.yaw_rate * dt * self.theta_gain
        else:
            print("use wheel angle")
            theta = theta + beta
        #print("dt:", dt)
        print("beta:", beta)
        print("yaw_rate*dt", self.yaw_rate*dt)

        x_dot = - d * sin(theta) * cos(delta)
        y_dot = d * cos(theta) * cos(delta)

        # print(x, y, theta)

        return x, y, theta, z, x_dot, y_dot

    # def vehicle_state_stack(self, new_vel, new_alpha, dt, crio_time):
    #     # print len(self.Y)
    #     self.Y.append([self.X[0,0], self.X[1,0], self.X[2,0], self.X[3,0], self.old_vel, self.old_alpha, dt, crio_time])
    #     # print crio_time


    def ekf_init(self, init_x, init_y, init_theta, init_z):
        self.X = np.mat([init_x, init_y, init_theta, init_z]).T

    def ekf_predict(self, new_vel, new_alpha, dt):
        x, y, theta, z, x_dot, y_dot = self.bicycle_model(self.X[0,0], self.X[1,0], self.X[2,0], self.X[3,0],
                                                       self.old_vel, self.old_alpha, dt)

    # model linearization
        self.F[0, 2] = x_dot
        self.F[1, 2] = y_dot

        self.P = self.F * self.P * self.F.T + self.Q
        K = self.P * self.H.T * (self.H * self.P * self.H.T + self.R).I

        if new_vel < 0.5:
            if abs(self.old_gps[2,0] - self.new_gps[2,0]) > 0.5:
                self.Q[2, 2] = 0
        if new_vel == 0 :
            self.Q[0,0] = 0; self.Q[1,1] = 0
        else:
            self.stop_index = True

        # if self.restart == True:
        #     self.R = np.mat(np.diag([1000.0, 1000.0, 1000.0, 1000.0]))
        #     if self.L >= 10:
        #         self.R = self.R_init
        #         self.restart = False

        self.P = (np.eye(4) - K * self.H) * self.P

        dL = new_vel * dt
        self.L += dL

        self.X = np.mat([x, y, theta, z]).T

        return K

    # GPS update
    def ekf_update(self, x_meas, y_meas, theta_meas, z_meas, K):
        if theta_meas > 0 and self.X[2,0] < 0 and theta_meas > 1.5:
            theta_meas-=math.pi*2
        elif theta_meas < 0 and self.X[2,0] > 0 and theta_meas <-1.5:
            theta_meas+=math.pi*2

        if self.back == True:
            if theta_meas < 0:
                theta_meas += math.pi
            elif theta_meas > 0:
                theta_meas -= math.pi

        Z = np.mat([x_meas, y_meas, theta_meas, z_meas]).T
        S = Z - self.X
        S[2, 0] = normalizeHeading(S[2,0])
        self.X = self.X + K * S
        self.X[2,0] = normalizeHeading(self.X[2,0])



    def gps_odom_callback(self, msg):

        print("gps_odom_callback_start : ", msg.header.stamp)
        # To know when it runs
        gps_odom_time = time.gmtime((rospy.Time.now()).to_sec())
        print("############################################# \n \n \n gps_odom_CB : ",gps_odom_time.tm_year,'-',gps_odom_time.tm_mon,'-',gps_odom_time.tm_mday,'-',gps_odom_time.tm_hour,':',gps_odom_time.tm_min,':',gps_odom_time.tm_sec)
        
        print("real time : ",(rospy.Time.now()).to_sec())
        print("\n bag time : ", (msg.header.stamp).to_sec())

        self.vscount = 0;

        self.covariance = msg.pose.covariance

        euler = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        print(euler[2])

        if not self.filter_initialized:
            self.ekf_reset()

            # We always calculate in relative coordinates, but then displace result to global frame
            self.origin_gps_x = msg.pose.pose.position.x
            self.origin_gps_y = msg.pose.pose.position.y
            self.origin_altitude = msg.pose.pose.position.z
            # first calculation of relative pos measurement
            new_gps = np.mat([msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              euler[2],
                              msg.pose.pose.position.z]).T

            self.ekf_init(new_gps[0,0], new_gps[1,0], new_gps[2,0], new_gps[3,0])

            if self.relative_gps:
                print('origin')
                rospy.set_param('~origin/x', self.origin_gps_x)
                rospy.set_param('~origin/y', self.origin_gps_y)
                rospy.loginfo("Using relative GPS coordinates. Origin stored in ~/origin")

            self.filter_initialized = True
            self.last_time = rospy.get_rostime()

            self.previous_callback_time = msg.header.stamp
            self.last_bag_gps_time = msg.header.stamp
            self.last_bag_vehicle_time = msg.header.stamp
            self.bag_gps_time = msg.header.stamp
            self.bag_vehilcle_time = msg.header.stamp
        else:

            new_gps = np.mat([msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              euler[2],
                              msg.pose.pose.position.z]).T

            # predict_vect, predict_time = self.gps_predict(msg.header.stamp)

            # predicted_stamp = msg.header.stamp + predict_time

            # x = predict_vect[0]
            # y = predict_vect[1]
            # new_gps[0] += sqrt(x*x+y*y)*cos(atan2(y, x) + new_gps[2])
            # new_gps[1] += sqrt(x*x+y*y)*sin(atan2(y, x) + new_gps[2])
            # new_gps[2] += predict_vect[2]

            # print(new_gps[2], ' measurement')
            covx=self.covariance[0]/0.05
            covy=self.covariance[7]/0.05
            covt=self.covariance[35]/0.0005
            self.Q[0,0]=self.Q_init[0,0]*(1-self.cov_a)+self.Q_init[0,0]*self.cov_a/covx
            self.Q[1,1]=self.Q_init[1,1]*(1-self.cov_a)+self.Q_init[1,1]*self.cov_a/covy
            self.Q[2,2]=self.Q_init[2,2]*(1-self.cov_a)+self.Q_init[2,2]*self.cov_a/covt
            self.R[0,0]=self.R_init[0,0]*(1-self.cov_a)+self.R_init[0,0]*self.cov_a*covx
            self.R[1,1]=self.R_init[1,1]*(1-self.cov_a)+self.R_init[1,1]*self.cov_a*covy
            self.R[2,2]=self.R_init[2,2]*(1-self.cov_a)+self.R_init[2,2]*self.cov_a*covt
            # print("---------------------------------")
            # print("%.4f"%self.Q[0,0],"%.4f"%self.Q[1,1],"%.4f"%self.Q[2,2])
            # print("%.4f"%self.R[0,0],"%.4f"%self.R[1,1],"%.2f"%self.R[2,2])

            #if self.time_state:
            #    dt = (self.time4bagplay-self.last_time).to_sec()
            #else:
            #    dt = (rospy.Time.now()-self.last_time).to_sec()

            #if dt < 0.005 or dt > 0.15:
            #    dt = 0.01;

            if self.time_state:

                print("time_state : ", self.time_state)
                self.current_gps_time = rospy.Time.now()
                self.bag_gps_time = msg.header.stamp

                print("bag_gps_time : ", (self.bag_gps_time).to_sec())

                dt = (self.bag_gps_time - self.previous_callback_time).to_sec()

                print("previous_callback_time : ", (self.previous_callback_time).to_sec())

                print("dt between current CB & previous CB : " , dt)

                # to calculate time step of gps_odom_callback and velocity of gps
                dt_gps = (self.bag_gps_time - self.last_bag_gps_time).to_sec()

            else:
                print("time_state : ", self.time_state)

                # to calculate time step of current and previous callback
                self.new_time = rospy.Time.now()
                dt = (self.new_time - self.old_time).to_sec()   
                print("dt between current CB & previous CB : " , dt)
                
                # it works without restriction as it is for real-time running
                # Be sure that if time_state is False
                #if dt > 0.02:
                #    dt = 0.01
                
                # to calculate time step of gps_odom_callback and velocity of gps    
                dt_gps = (rospy.Time.now() - self.last_time_gps).to_sec()
                
            
            print("gps_odom_callback_dt : ", dt_gps)
            print("gps_velocity : ", math.sqrt(math.pow((self.new_gps[0,0]-self.old_gps[0,0])/dt_gps,2) + math.pow((self.new_gps[1,0]-self.old_gps[1,0])/dt_gps,2)))


            K = self.ekf_predict(self.old_vel, self.old_alpha, dt)

            # print(self.X[2], ' predict')

            # print('gps_dt : ', dt)
            self.Brake_Cnt += 1

            self.Brake_Flag = False
            if(self.Brake_Active == 2 and self.vel < 1/3.6):
                self.Brake_Flag = True
                self.Brake_Cnt = 0

            #     #############################################################################################################################
            if((self.Brake_Flag == False) and self.Brake_Cnt > 5):
            # if(False):
                if abs(self.old_gps[2,0] - self.new_gps[2,0])< 0.5:# and self.covariance[0] < 15:
                    if not self.old_vel == 0:
                        self.ekf_update(new_gps[0,0], new_gps[1,0], new_gps[2,0], new_gps[3,0], K)
            # if(self.Brake_Flag == True):
            #     self.Brake_Cnt = 0
                #############################################################################################################################

            # print(self.X[2], ' estimate')

            #if self.time_state:
            #    self.last_time = self.time4bagplay
            #else:
            #    self.last_time = rospy.Time.now()


            if self.time_state:
                # variables that two functions(gps_odom_callback, vehicle_state_callback) keep updating at their turn
                self.previous_callback_time = self.bag_gps_time
        
                # variable that only gps_odom_callback can update
                # using this variable help us to know how often gps_odom_callback is made at each time
                self.last_bag_gps_time = self.bag_gps_time

            else:
                # variables that two functions(gps_odom_callback, vehicle_state_callback) keep updating at their turn
                self.old_time = self.new_time

                # variable that only gps_odom_callback can update
                # using this variable help us to know how often gps_odom_callback is made at each time
                self.last_time_gps = rospy.Time.now()                

        self.publish_odom()

        print("time after publish : ", msg.header.stamp)

        # self.old_X = self.X
        self.old_gps = self.new_gps
        self.new_gps = new_gps


        # to see how many times each function(rpy_rate_callback, vehicle_state_callback) is called during a cycle of gps_odom_callback
        print("rpy_cnt : " , self.rpy_cnt)
        print("veh_cnt : " , self.veh_cnt, "\n \n \n #############################################")
        self.veh_cnt = 0
        self.rpy_cnt = 0


    def rpy_rate_callback(self, msg):

        # To know when it runs
        rpy_time = time.gmtime((rospy.Time.now()).to_sec())
        print("rpy_rate_CB : ",rpy_time.tm_year,'-',rpy_time.tm_mon,'-',rpy_time.tm_mday,'-',rpy_time.tm_hour,':',rpy_time.tm_min,':',rpy_time.tm_sec)
        self.imu_init = True
        self.yaw_rate = msg.angular_velocity.z
        self.rpy_cnt += 1

    def vehicle_state_callback(self, msg):

        # To know when it runs
        vehicle_time = time.gmtime((rospy.Time.now()).to_sec())
        print("vehicle_state_CB : ",vehicle_time.tm_year,'-',vehicle_time.tm_mon,'-',vehicle_time.tm_mday,'-',vehicle_time.tm_hour,':',vehicle_time.tm_min,':',vehicle_time.tm_sec)
        
        #self.time4bagplay = msg.header.stamp

        self.Brake_Active = msg.GWAY3.Brake_Active

        self.gear = msg.GWAY3.GearSelDisp
        if not self.filter_initialized:
            return

        if msg.GWAY3.GearSelDisp == 7 and abs(self.new_gps[2, 0] - self.old_gps[2, 0]) > 2.0:
            self.back = True
        elif msg.GWAY3.GearSelDisp == 5 and abs(self.new_gps[2, 0] - self.old_gps[2, 0]) > 2.0:
            self.back = False

        # steering_bias = 0.0
        # steer_angle = msg.GWAY2.Steering_Angle / self.stear_const
        #     self.back = False

        # p1 = -1.184*pow(10,-5)
        # p2 = 0.0003041
        # p3 = -0.002027
        # p4 = -0.01141
        # p5 = 0.2303
        # p6 = -1.303
        # p7 = 3.497
        # p8 = -4.567
        # p9 = 17.4

        # steer_angle = abs(steer_angle)
        # if steer_angle >= math.pi / 3:
        #     self.ratio = p1 * pow(steer_angle, 8) + p2 * pow(steer_angle, 7) + p3 * pow(steer_angle, 6) + p4 * pow(
        #         steer_angle, 5) + p5 * pow(steer_angle, 4) + p6 * pow(steer_angle, 3) + p7 * pow(steer_angle, 2) + p8 * steer_angle + p9
        # else:
        #     self.ratio = (self.stear_const - self.steer_bias) * 3 / math.pi * steer_angle + self.steer_bias

        # if self.use_const_ratio:
        #     new_alpha = msg.GWAY2.steering_bias
        # else:
        #     new_alpha = msg.GWAY2.steering_bias / self.ratio  # + steering_bias

        new_alpha = (msg.GWAY2.Steering_Angle * math.pi/180.) / self.stear_const

        new_vel = (msg.GWAY1.Wheel_Velocity_RL + msg.GWAY1.Wheel_Velocity_RR)/2/3.6

        self.vel = new_vel

        #dt = (msg.header.stamp - self.last_time).to_sec()
        # print("veh_dt : ", dt)

        if self.time_state:
            print("time_state : ", self.time_state)

            self.current_vehicle_time = rospy.Time.now()
            self.bag_vehicle_time = self.bag_gps_time + (self.current_vehicle_time - self.current_gps_time)

            if (self.current_vehicle_time - self.current_gps_time).to_sec() > 0.2:
                self.bag_vehicle_time = self.last_bag_vehicle_time + self.vehicle_cycle

            print("bag_vehicle_time : ", (self.bag_vehicle_time).to_sec())

            dt = (self.bag_vehicle_time - self.previous_callback_time).to_sec()
            print("dt between current CB & previous CB : " , dt)

            # to calculate time step of vehicle_state_callback
            dt_vehicle = (self.bag_vehicle_time - self.last_bag_vehicle_time).to_sec()
            self.vehicle_cycle = (self.bag_vehicle_time - self.last_bag_vehicle_time)
            print("last_bag_vehicle_time : ", (self.last_bag_vehicle_time).to_sec())
            print("vehicle_state_callback_dt : ", dt_vehicle)

        else:

            print("time_state : ", self.time_state)

            # to calculate time step of current and previous callback
            self.new_time = rospy.Time.now()
            dt = (self.new_time - self.old_time).to_sec()
            print("dt between current CB & previous CB : " , dt)

            #if dt > 0.02:
            #    dt = 0.01

            # to calculate time step of vehicle_state_callback
            dt_vehicle = (rospy.Time.now() - self.last_time_vehicle).to_sec()
            print("vehicle_state_callback_dt : ", dt_vehicle)

        print("vehicle_velocity : " , new_vel)


        # R meas
        if self.heading_init == True:
            if abs(new_vel) <= 0.5:
                self.R = np.mat(np.diag([-513.8 * new_vel * new_vel + 1000, -513.8 * new_vel * new_vel + 1000,
                                         -508.6 * new_vel * new_vel + 1000, 100]))
                self.Q = np.mat(np.diag([0.0, 0.0, 0.0, 0.0]))
                # print('low speed')
            else:
                self.R = copy.deepcopy(self.R_init)
                self.Q = copy.deepcopy(self.Q_init)
            # for covariance
        covariance_x = self.covariance[0];
        covariance_y = self.covariance[7];
        covariance_theta = self.covariance[14]
        if covariance_x > 8 or covariance_y > 8:
            self.Q[0,0] = 0.2; self.Q[1,1] = 0.2; self.Q[2,2] = 0.01
            self.R[0,0] = pow(covariance_x-8,2)+15.0
            self.R[1,1] = pow(covariance_y-8,2)+15.0
            self.R[2,2] = pow(covariance_theta-5,2)+10.0
        else:
            self.Q = copy.deepcopy(self.Q_init)           

        # Heading initializing
        # if self.current_wp_index < 570:
        #     self.heading_init = True
        if self.heading_init == False:
            if new_vel < 1.388:
                self.Q[2, 2] = 100
            else:
                self.heading_init = True
        else:
            self.Q = copy.deepcopy(self.Q_init)

        #if dt < 0.005 or dt > 0.15:
        #    dt = 0.01;

        K = self.ekf_predict(new_vel, new_alpha, dt)
        # self.vehicle_state_stack(new_vel, new_alpha, dt, msg.header.stamp)


        # self.vscount = self.vscount+1
        # if (self.vscount%6) == 0:
        #     self.publish_odom()


        if self.time_state:
            self.previous_callback_time = self.bag_vehicle_time
            self.last_bag_vehicle_time = self.bag_vehicle_time 
        else:
            self.old_time = self.new_time
            self.last_time_vehicle = rospy.Time.now()
        
        self.publish_odom()

        # self.old_X = self.X
        self.old_vel = new_vel
        self.old_alpha = new_alpha
        #self.last_time = msg.header.stamp
        self.veh_cnt += 1

    # def motion_command_callback(self, msg):
    #     self.inclination = msg.inclination
    #     self.heading_difference = msg.heading_difference
    #     self.lateral_offset = msg.lateral_offset
    #     self.curvature = msg.curvature

    # def current_wp_index_callback(self, msg):
    #     self.current_wp_index = msg.data

    def publish_odom(self):
        msg = Odometry()
        
        if self.time_state:
            msg.header.stamp = self.previous_callback_time
        
        else:
            msg.header.stamp = self.new_time

        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id

        ekf_heading = self.X[2,0]
        if self.relative_gps:
            msg.pose.pose.position = Point(self.X[0,0], self.X[1,0], self.X[3,0])
        else:
            msg.pose.pose.position = Point(self.X[0,0] + self.origin_gps_x, self.X[1,0] + self.origin_gps_y, 0.0)

        msg.pose.pose.orientation = Quaternion(*(kdl.Rotation.RPY(0.0, -self.inclination, ekf_heading).GetQuaternion()))
        p_cov = np.array([0.0] * 36).reshape(6, 6)
    # position covariance
        p_cov[0:2, 0:2] = self.P[0:2, 0:2]
    # x and Yaw
        p_cov[5, 0] = p_cov[0, 5] = self.P[2, 0]
    # y and Yaw
        p_cov[5, 1] = p_cov[1, 5] = self.P[2, 1]
    # Yaw and Yaw
        p_cov[5, 5] = self.P[2, 2]

        msg.pose.covariance = tuple(p_cov.ravel().tolist())
        # pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        # ori = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        if(msg.header.stamp != self.prev_stamp and msg.header.stamp != self.prev_stamp2 and msg.header.stamp != self.prev_stamp3):
            self.gps_ekf_odom_pub.publish(msg)
        self.prev_stamp3 = self.prev_stamp2
        self.prev_stamp2 = self.prev_stamp
        self.prev_stamp = msg.header.stamp

if __name__ == '__main__':
    rospy.init_node('gps_ekf_node')

    gps_ekf = RosGpsEkf()


    gps_ekf.time_state = rospy.get_param('~bagfile', True)
    gps_ekf.frame_id = rospy.get_param('~frame_id', 'odom')
    gps_ekf.child_frame_id = rospy.get_param('~child_frame_id', 'ekf')
    gps_odom = rospy.get_param('~gps_odom_topic', '/odom/ublox')
    vehicle_state = rospy.get_param('~vehicle_state_topic', '/CAN_Gateway')

    # Set GPS position relative to first measurement, i.e. first measurement will be at (0,0)
    gps_ekf.relative_gps = rospy.get_param('~relative_gps', True)
    # gps_ekf.publish_odom_tf = rospy.get_param('~publish_odom_tf', True)

    rospy.Subscriber(gps_odom, Odometry , gps_ekf.gps_odom_callback, queue_size=1)
    rospy.Subscriber(vehicle_state, VehicleState, gps_ekf.vehicle_state_callback, queue_size=10)
    rospy.Subscriber('/xsens/imu/data', Imu, gps_ekf.rpy_rate_callback, queue_size=10)
    # rospy.Subscriber('/vehicle/motion_command', MotionCommandStamped3, gps_ekf.motion_command_callback, queue_size=1)

    rospy.spin()

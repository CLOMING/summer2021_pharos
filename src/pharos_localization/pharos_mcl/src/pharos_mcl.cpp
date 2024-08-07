#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <angles/angles.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pharos_msgs/StateStamped2016.h>

#include <numeric>
#include <iostream>
#include <algorithm>
#include <fstream>

#include <stdio.h>


typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

struct RPY_{
    double roll;
    double pitch;
    double yaw;
};

struct xytheta_{
    double x;
    double y;
    double theta;
    double weight;
};

double *MeasInMapCUDA(int N, float *point_x, float *point_y , int *point_i, int *Map, float Map_resolution,
                      unsigned int Map_width, unsigned int Map_height, double Map_origin_x, double Map_origin_y, float Tx, float Ty, float theta, double *w, std::string type);
void CopyRMapCUDA(int *Map, unsigned int Map_width, unsigned int Map_height);
void CopyVMapCUDA(int *Map, unsigned int Map_width, unsigned int Map_height);
void CUDAFree();

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

int *roadmap_data_;
int *verticalmap_data_;
bool use_cuda = true;
bool use_roadinfo = true;
bool use_verticalinfo = true;
bool use_novatel = false;

std::clock_t measmap_begin_, measmap_end_, filter_begin_, filter_end_, resample_begin_, resample_end_, cuda_begin_, cuda_end_;

sensor_msgs::PointCloud2Ptr roadinfo_velodyne_points_ (new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2Ptr verticalinfo_velodyne_points_ (new sensor_msgs::PointCloud2);
nav_msgs::OccupancyGridPtr roadinfo_map_ (new nav_msgs::OccupancyGrid);
nav_msgs::OccupancyGridPtr verticalinfo_map_ (new nav_msgs::OccupancyGrid);
PointCloudPtr input_Vcloud_(new PointCloud);
PointCloudPtr input_Rcloud_(new PointCloud);


tf::StampedTransform transform_;
double yaw_trans_;
double mcl_theta_;
double cov_x_, cov_y_, cov_t_;

ros::NodeHandlePtr node_;
ros::NodeHandlePtr pnode_;

ros::Publisher particle_pub_;
ros::Publisher odom_pub_;

ros::Publisher a_pub_; // del

ros::Subscriber vehicle_sub_;
ros::Subscriber roadinfo_velodyne_point_sub_;
ros::Subscriber verticalinfo_velodyne_point_sub_;
ros::Subscriber roadinfo_map_sub_;
ros::Subscriber verticalinfo_map_sub_;
ros::Subscriber odom_sub_;
ros::Subscriber novatel_sub_;

ros::Time curr_t_;
ros::Time past_t_;
ros::Time rinfo_time;
ros::Time vinfo_time;

int num_particle_ = 100;
int num_gps_particle_ = 5;
float vertical_gain_;

bool particle_init_;
bool odom_init_;
bool roadmap_road_ = false;
bool verticalmap_road_ = false;

geometry_msgs::PoseArray::Ptr particles_(new geometry_msgs::PoseArray);
nav_msgs::Odometry MCL_map_;
nav_msgs::Odometry MCL_odom_;
std::vector<xytheta_> particle_poses_;

double new_vel_ = 0.0;
double new_steering_ = 0.0;
double old_vel_ = 0.0;
double old_steering_ = 0.0;

double odom_x_, odom_y_, odom_z_, odom_roll_, odom_pitch_, odom_yaw_, odom_theta_;
double novatel_x_, novatel_y_, novatel_theta_;


float *point_x_r, *point_x_v;
float *point_y_r, *point_y_v;
int *point_i_r, *point_i_v;

bool cmp(const xytheta_ &p1, const xytheta_ &p2) {
    if (p1.weight < p2.weight) {
        return true;
    } else
        return false;
}

double RangeRandom(double range_min, double range_max)
{
    return (double)rand()/(RAND_MAX)*(range_max-range_min) + range_min;
}

double sample(double b)
{
    double sum=0;
    for(int i=0;i<12;i++)
    {
        sum+=RangeRandom(-b,b);
    }

    return sum/2;
}

double gaussianRandom(double average, double stdev) {
    double v1, v2, s, temp;

    do {
        v1 =  2 * ((double) rand() / RAND_MAX) - 1;      // -1.0 ~ 1.0 까지의 값
        v2 =  2 * ((double) rand() / RAND_MAX) - 1;      // -1.0 ~ 1.0 까지의 값
        s = v1 * v1 + v2 * v2;
    } while (s >= 1 || s == 0);

    s = sqrt( (-2 * log(s)) / s );

    temp = v1 * s;
    temp = (stdev * temp) + average;

    return temp;
}

void PublishParticle(geometry_msgs::PoseArray particles){
    particles.header.frame_id = "map";
    particles.header.stamp = curr_t_;
    particle_pub_.publish(particles);
}

RPY_ QuaterToRPY(nav_msgs::Odometry odom){
    tf::Quaternion quat;
    RPY_ rpy;
    tf::quaternionMsgToTF(odom.pose.pose.orientation,quat);
    tf::Matrix3x3(quat).getRPY(rpy.roll, rpy.pitch, rpy.yaw);
    rpy.yaw = angles::normalize_angle_positive(rpy.yaw);

    return rpy;
}

void tf_broadcaster(nav_msgs::Odometry odom_){

    static tf::TransformBroadcaster br;
    RPY_ rpy;
    rpy = QuaterToRPY(odom_);
    tf::Quaternion quat;
    quat.setRPY(rpy.roll,rpy.pitch,rpy.yaw);
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z) );

    transform.setRotation(quat);

    br.sendTransform(tf::StampedTransform(transform, odom_.header.stamp, odom_.header.frame_id,odom_.child_frame_id));
}

geometry_msgs::Pose tf2poseOrientation(tf::Quaternion quat){
    geometry_msgs::Pose pose;
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    return pose;
}

xytheta_ vehicle_motion_update(xytheta_ particle_pose, float vehicle_speed, float steering_angle, float dt){
    double x = particle_pose.x;
    double y = particle_pose.y;
    double theta = particle_pose.theta;
    double speed_noise_bandwidth = 3/3.6;
    double steering_noise_bandwidth = 3*M_PI/180;

    // double speed_noise = sample(sqrt(1*pow(vehicle_speed,2)+0.1*pow(steering_angle,2)));
    // double steering_noise =sample(sqrt(0.00005*pow(vehicle_speed,2)+1*pow(steering_angle,2)));

    // vehicle_speed *= 1.5;
    // vehicle_speed += speed_noise;
    // steering_angle += steering_noise;

    float L = 2.7;
    double d = vehicle_speed*dt;
    double beta = d/L*tanf(steering_angle);

    if(fabs(beta)<0.001){
        x += d*cos(theta);
        y += d*sin(theta);
        theta += beta;
    }
    else{
        double R = d/beta;
        double Cx = x-sin(theta)*R;
        double Cy = y+cos(theta)*R;
        theta += beta;
        x = Cx+sin(theta)*R;
        y = Cy-cos(theta)*R;
    }

    particle_pose.x=x+gaussianRandom(0,0.02);
    particle_pose.y=y+gaussianRandom(0,0.02);
    particle_pose.theta=theta+gaussianRandom(0,0.2)*0.1;

    return particle_pose;
}

double V_WeightFunction(int mapmeas, int z)
{
    double w, b1=0, b2=0;
    b1 = std::bitset<8>(mapmeas&z).count();
    b2 = b1*b1;
    if(mapmeas == z)
        b2=b2*2;
    w = b2;
    return w;
}

double MeasInMap(const PointCloudPtr& PointcloudTransformed, nav_msgs::OccupancyGridPtr Map, xytheta_ particle_pose, int *map_data_,std::string type){


    double resolutionInverse = 1/Map->info.resolution;

    int mapmeas;
    float mapmeas_sum = 0;
    float sensormeas_sum = 0;
    double w = 0;
    double w_cuda = 0;

    double *w_cu= new double[PointcloudTransformed->size()];


    if(!use_cuda)
    {
        PointCloud::const_iterator iter = PointcloudTransformed->begin();

        for(int i=0; i<PointcloudTransformed->size(); i++)
        {
            if(i>=PointcloudTransformed->size())
                ROS_FATAL("Out of Map size!!!!!");
            int xIndex, yIndex;

            xIndex = (int)((PointcloudTransformed->at(i).x - Map->info.origin.position.x)*resolutionInverse);
            yIndex = (int)((PointcloudTransformed->at(i).y - Map->info.origin.position.y)*resolutionInverse);

            int mapIndex = MAP_IDX(Map->info.width,xIndex,yIndex);
            mapmeas = Map->data[mapIndex];
            if(mapmeas < 0)
                mapmeas +=256;

            mapmeas_sum+=((float)mapmeas);
            sensormeas_sum+=PointcloudTransformed->at(i).intensity;

            w += V_WeightFunction(mapmeas, (int)PointcloudTransformed->at(i).intensity);
            iter++;
        }
        w = w/PointcloudTransformed->size();
    }

    else{
        if(type.compare("road")==0)
        MeasInMapCUDA(PointcloudTransformed->size(), point_x_r, point_y_r, point_i_r, map_data_, Map->info.resolution,
                      Map->info.width, Map->info.height, Map->info.origin.position.x, Map->info.origin.position.y, particle_pose.x, particle_pose.y, particle_pose.theta, w_cu, type);
        else if(type.compare("vertical")==0)
        MeasInMapCUDA(PointcloudTransformed->size(), point_x_v, point_y_v, point_i_v, map_data_, Map->info.resolution,
                      Map->info.width, Map->info.height, Map->info.origin.position.x, Map->info.origin.position.y, particle_pose.x, particle_pose.y, particle_pose.theta, w_cu, type);

        for(int i=0; i<PointcloudTransformed->size(); i++){
            w_cuda +=w_cu[i];
        }

        w = w_cuda/PointcloudTransformed->size();
    }

    delete(w_cu);

    return w;
}

PointCloudPtr Transform(PointCloudPtr source_cloud, double target_x, double target_y, double target_yaw){
    filter_begin_ = clock();

    PointCloudPtr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform (0,3) = target_x;
    transform (1,3) = target_y;

    transform (0,0) = cosf(target_yaw);
    transform (0,1) = -sinf(target_yaw);
    transform (1,0) = sinf(target_yaw);
    transform (1,1) = cosf(target_yaw);

    pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform);
    filter_end_ = clock();

    return transformed_cloud;
}

void Resampling(std::vector<double> w){
    resample_begin_ = clock();
    geometry_msgs::Pose pose;
    geometry_msgs::PoseArray::Ptr particles(new geometry_msgs::PoseArray);
    tf::Quaternion quat;
    float beta=0.0, w_max, w_sum=0.0;
    xytheta_ *particle_pose(new xytheta_);
    std::vector<xytheta_> new_particles;
    int Index = (rand() * (int)(num_particle_)/RAND_MAX);

    w_max = *std::max_element(w.begin(), w.end());

    for(int i=0; i<num_particle_; i++){

        beta += (1+gaussianRandom(0,0.2))*w_max;
        while(beta>w[Index]){
            beta -= w[Index];
            Index=(Index+1)%num_particle_;
        }

        new_particles.push_back(particle_poses_[Index]);
        quat = tf::createQuaternionFromRPY(0.0,0.0,particle_poses_[Index].theta);
        pose = tf2poseOrientation(quat);
        pose.position.x = particle_poses_[Index].x;
        pose.position.y = particle_poses_[Index].y;
        pose.position.z = w[Index];
        particles->poses.push_back(pose);
        w_sum += w[Index];

    }




// mcl
    particles_ = particles;

    float mcl_x=0.0, mcl_y=0.0, mcl_theta=0.0, mcl_theta_x=0.0, mcl_theta_y=0.0;

    for(int i=0; i<num_particle_; i++){
        mcl_x += particles->poses[i].position.x;
        mcl_y += particles->poses[i].position.y;
        mcl_theta_x +=cos(new_particles[i].theta);
        mcl_theta_y +=sin(new_particles[i].theta);
    }
    mcl_theta_x = mcl_theta_x/num_particle_;
    mcl_theta_y = mcl_theta_y/num_particle_;
    mcl_theta=atan2f(mcl_theta_y, mcl_theta_x);
    mcl_x = mcl_x/num_particle_;
    mcl_y = mcl_y/num_particle_;


// covariance calculation
    double cov_x=0, cov_y=0, cov_theta_x=0 ,cov_theta_y=0, cov_theta=0;
    for(int i=0; i<num_particle_; i++){
        cov_x += pow(particles->poses[i].position.x-mcl_x,2);
        cov_y += pow(particles->poses[i].position.y-mcl_y,2);
        cov_theta_x += pow(cos(new_particles[i].theta)-cosf(mcl_theta),2);
        cov_theta_y += pow(sin(new_particles[i].theta)-sinf(mcl_theta),2);
    }
    cov_x=cov_x/num_particle_;
    cov_y=cov_y/num_particle_;
    cov_theta_x=cov_theta_x/num_particle_;
    cov_theta_y=cov_theta_y/num_particle_;
    cov_theta=sqrt(pow(cov_theta_x,2)+pow(cov_theta_y,2));

    boost::array<double, 36> covariance = {{
                                                   cov_x,
                                                   0,
                                                   0,
                                                   0, 0, 0,
                                                   0,
                                                   cov_y,
                                                   0,
                                                   0, 0, 0,
                                                   0,
                                                   0,
                                                   0,
                                                   0, 0, 0,
                                                   0, 0, 0, 0, 0, 0,
                                                   0, 0, 0, 0, 0, 0,
                                                   0, 0, 0, 0, 0, cov_theta
                                           }};
    cov_x_ = cov_x;
    cov_y_ = cov_y;
    cov_t_ = cov_theta;

    quat = tf::createQuaternionFromRPY(0.0,0.0,mcl_theta);
    mcl_theta_ = mcl_theta;
    MCL_map_.pose.pose = tf2poseOrientation(quat);
    MCL_map_.pose.pose.position.x = mcl_x;
    MCL_map_.pose.pose.position.y = mcl_y;
    MCL_map_.pose.covariance = covariance;

    particle_poses_.clear();

    std::stable_sort(new_particles.begin(), new_particles.end(), cmp);

    for(int i=0;i<num_particle_; i++){
        if(i<num_gps_particle_){
            particle_pose->x = odom_x_ + gaussianRandom(0,0.2)*0.5;
            particle_pose->y = odom_y_+ gaussianRandom(0,0.2)*0.5;
            particle_pose->theta = odom_theta_ + gaussianRandom(0,0.2)*M_PI/180;
            particle_poses_.push_back(*particle_pose);
        }
        else{
            particle_poses_.push_back(new_particles[i]);
        }
    }

}

void MCL()
{
    xytheta_ particle_pose;
    geometry_msgs::Pose pose;
    tf::Quaternion quat;
    double road_w=0.0 ,vertical_w=0.0,w_sum=0.0;

    double road_w_sum=0.0, vertical_w_sum=0.0, road_w_max=0.0, vertical_w_max=0.0, road_w_min=0.0, vertical_w_min=0.0;

    std::vector<double> road_w_;
    std::vector<double> vertical_w_;
    std::vector<double> weights;
    PointCloudPtr pointcloudTransformed(new PointCloud);

    point_x_r = new float[input_Rcloud_->size()];
    point_y_r = new float[input_Rcloud_->size()];
    point_i_r = new int[input_Rcloud_->size()];
    point_x_v = new float[input_Vcloud_->size()];
    point_y_v = new float[input_Vcloud_->size()];
    point_i_v = new int[input_Vcloud_->size()];

    if(!particle_init_){
        for(int i=0;i<num_particle_;i++){
            particle_pose.x = odom_x_ + gaussianRandom(0,0.2);
            particle_pose.y = odom_y_+ gaussianRandom(0,0.2);
            particle_pose.theta = odom_theta_ + gaussianRandom(0,0.2)*M_PI/180;
            quat = tf::createQuaternionFromRPY(0.0,0.0,particle_pose.theta);

            pose = tf2poseOrientation(quat);
            pose.position.x = particle_pose.x;
            pose.position.y = particle_pose.y;

            particles_->poses.push_back(pose);
            particle_poses_.push_back(particle_pose);
        }
// printf("PP_\n");
        PublishParticle(*particles_);

        old_vel_ = new_vel_;
        old_steering_ = new_steering_;
        past_t_ = curr_t_;

        particle_init_ = true;
    }
    else{

        measmap_begin_ = clock();

        double dt = (curr_t_ - past_t_).toSec();

///
        for(int i =0; i<input_Rcloud_->size(); i++){
            point_x_r[i] = input_Rcloud_->points[i].x;
            point_y_r[i] = input_Rcloud_->points[i].y;
            point_i_r[i] = (int)input_Rcloud_->points[i].intensity;
        }
        for(int i =0; i<input_Vcloud_->size(); i++){
            point_x_v[i] = input_Vcloud_->points[i].x;
            point_y_v[i] = input_Vcloud_->points[i].y;
            point_i_v[i] = (int)input_Vcloud_->points[i].intensity;
        }
///
        for(int i=0;i<num_particle_;i++){

            // -------------- Motion update -----------------
            particle_pose = vehicle_motion_update(particle_poses_[i], old_vel_, old_steering_, dt);
            particle_poses_[i] = particle_pose;

            // ---------------- Meas In map -----------------

            if(roadinfo_velodyne_points_->width > 10 and use_roadinfo){
                if(!use_cuda){
                    pointcloudTransformed = Transform(input_Rcloud_, particle_pose.x, particle_pose.y, particle_pose.theta);
                    road_w = MeasInMap(pointcloudTransformed, roadinfo_map_, particle_pose, roadmap_data_, "road");
                }
                else
                    road_w = MeasInMap(input_Rcloud_, roadinfo_map_, particle_pose, roadmap_data_, "road");
            }

            if(verticalinfo_velodyne_points_->width > 10 and use_verticalinfo){
                if(!use_cuda){
                    pointcloudTransformed = Transform(input_Vcloud_, particle_pose.x, particle_pose.y, particle_pose.theta);
                    vertical_w = MeasInMap(pointcloudTransformed, verticalinfo_map_, particle_pose, verticalmap_data_, "vertical");
                }
                else
                    vertical_w = MeasInMap(input_Vcloud_, verticalinfo_map_, particle_pose, verticalmap_data_, "vertical");
            }

            // weights.push_back((road_w+vertical_w)/2);
            // w_sum+=(road_w+vertical_w)/2;
            // particle_poses_[i].weight = road_w+vertical_w;
//            std::cout << vertical_w << "  " << road_w << std::endl;

            // weight array
// if(road_w>1)
// {
//     printf("%f\n",road_w);    
// }
            
            road_w_.push_back(road_w);
            vertical_w_.push_back(vertical_w);
            road_w_sum += road_w;
            vertical_w_sum += vertical_w;
        }

        road_w_max = *std::max_element(road_w_.begin(), road_w_.end());
        vertical_w_max = *std::max_element(vertical_w_.begin(), vertical_w_.end());
        // road_w_min = *std::min_element(road_w_.begin(), road_w_.end());
        // vertical_w_min = *std::min_element(vertical_w_.begin(), vertical_w_.end());

// printf("%f\t\t%f\n",road_w_sum,road_w_max);

        // weights covariance calculation

        // for(int i=0;i<num_particle_;i++){
        //     road_w_[i]/=road_w_max;
        //     vertical_w_[i]/=vertical_w_max;
        // }
        double road_w_average = road_w_sum / num_particle_/* / road_w_max*/;
        double vertical_w_average = vertical_w_sum / num_particle_/* / vertical_w_max*/;

        // double road_w_cov = 0.0, vertical_w_cov = 0.0;
        // for(int i=0; i<num_particle_; i++){
        //     road_w_cov+=pow(road_w_[i]-road_w_average,2);
        //     vertical_w_cov+=pow(vertical_w_[i]-vertical_w_average,2);
        //     if(i%10 == 0)printf("%f\n", vertical_w_[i]);
        // }

        // road_w_cov/=num_particle_;
        // vertical_w_cov/=num_particle_;

        printf("rwm:%f vwm:%f\n",road_w_max,vertical_w_max);
printf("v_g:%.1f v_w:%.2f%%\n",vertical_gain_,
    vertical_w_average*vertical_gain_/(road_w_average*(1-vertical_gain_) + vertical_w_average*vertical_gain_)*100);
        for(int i=0; i<num_particle_; i++){
            weights.push_back(road_w_[i]*(1-vertical_gain_) + vertical_w_[i]*vertical_gain_);
        }
        

        // ---------------- Resampling --------------------
        if(roadinfo_velodyne_points_->width != 0 and verticalinfo_velodyne_points_->width !=0)
            Resampling(weights);
        else{
            particles_->poses.clear();

            for(int i=0;i<num_particle_;i++){
                quat = tf::createQuaternionFromRPY(0.0,0.0,particle_poses_[i].theta);
                pose = tf2poseOrientation(quat);
                pose.position.x = particle_poses_[i].x;
                pose.position.y = particle_poses_[i].y;
                pose.position.z = 0.0; 
                particles_->poses.push_back(pose);
            }
        }

// printf("PP\n");
        PublishParticle(*particles_);

    // Transform to /odom START


        tf::Quaternion quat;
        quat.setRPY(0,0,mcl_theta_ + yaw_trans_);
        MCL_odom_.pose.pose = tf2poseOrientation(quat);

        double x_temp, y_temp;
        x_temp = MCL_map_.pose.pose.position.x*cos(yaw_trans_) - MCL_map_.pose.pose.position.y*sin(yaw_trans_);
        y_temp = MCL_map_.pose.pose.position.x*sin(yaw_trans_) + MCL_map_.pose.pose.position.y*cos(yaw_trans_);

        MCL_odom_.pose.pose.position.x = x_temp + transform_.getOrigin().x();
        MCL_odom_.pose.pose.position.y = y_temp + transform_.getOrigin().y();

        double cov_x, cov_y, cov_theta;
        cov_x = cov_x_*cos(yaw_trans_) - cov_y*sin(yaw_trans_);
        cov_y = cov_x_*sin(yaw_trans_) + cov_y*cos(yaw_trans_);
        cov_theta = cov_t_;

        boost::array<double, 36> covariance = {{
                                               cov_x,
                                               0,
                                               0,
                                               0, 0, 0,
                                               0,
                                               cov_y,
                                               0,
                                               0, 0, 0,
                                               0,
                                               0,
                                               0,
                                               0, 0, 0,
                                               0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, cov_theta
                                       }};

        MCL_odom_.pose.covariance = covariance;


    // Transform to /odom END
        MCL_odom_.header.frame_id = "odom";
        MCL_odom_.child_frame_id = "mcl";
        MCL_odom_.header.stamp = curr_t_;
        odom_pub_.publish(MCL_odom_);

        old_vel_ = new_vel_;
        old_steering_ = new_steering_;
        past_t_ = curr_t_;

        delete(point_x_r); delete(point_y_r); delete(point_i_r);
        delete(point_x_v); delete(point_y_v); delete(point_i_v);

//        std::cout << (long double)(cuda_end_-cuda_begin_)/CLOCKS_PER_SEC << std::endl;

//        std::cout << "filter & measmap : " << (long double)(measmap_end_-measmap_begin_)/CLOCKS_PER_SEC  << std::endl;
//        std::cout << "resample : " << (long double)(resample_end_-resample_begin_)/CLOCKS_PER_SEC << std::endl;

    }
}

void RoadinfoMapCallback(const nav_msgs::OccupancyGridConstPtr &map){
    *roadinfo_map_ = *map;

    roadmap_data_ = new int[map->info.width*map->info.height];

    for(unsigned long i=0;i<map->info.width*map->info.height; i++){
        roadmap_data_[i]= map->data.at(i);
    }

    CopyRMapCUDA(roadmap_data_, map->info.width, map->info.height);
    ROS_INFO("RMap init");

    roadmap_road_ = true;
}
void VerticalinfoMapCallback(const nav_msgs::OccupancyGridConstPtr &map){
    *verticalinfo_map_ = *map;

    verticalmap_data_ = new int[map->info.width*map->info.height];

    for(unsigned long i=0;i<map->info.width*map->info.height; i++){
        verticalmap_data_[i]= map->data.at(i);
    }

    CopyVMapCUDA(verticalmap_data_, map->info.width, map->info.height);
    ROS_INFO("VMap init");
    verticalmap_road_ = true;
}

void RoadinfoVelodyneCallback(const sensor_msgs::PointCloud2ConstPtr &input){
    *roadinfo_velodyne_points_ = *input;
    pcl::fromROSMsg(*input,*input_Rcloud_);

    rinfo_time = input->header.stamp;

    if(!odom_init_)
        ROS_FATAL("odom initialize fail");
    if(odom_init_ && verticalmap_road_ && roadmap_road_ && vinfo_time==rinfo_time)
        MCL();

    // printf("RT:%d \n",roadinfo_velodyne_points_->header.stamp.sec%10,roadinfo_velodyne_points_->header.stamp.nsec);
}

void VerticalinfoVelodyneCallback(const sensor_msgs::PointCloud2ConstPtr &input){
    *verticalinfo_velodyne_points_ = *input;

    pcl::fromROSMsg(*input,*input_Vcloud_);

    curr_t_ = input->header.stamp;
    vinfo_time = input->header.stamp;

    if(!odom_init_)
        ROS_FATAL("odom initialize fail");
    if(odom_init_ && verticalmap_road_ && roadmap_road_ && vinfo_time==rinfo_time)
        MCL();

    // printf("VT:%d.%d\n",verticalinfo_velodyne_points_->header.stamp.sec%10,verticalinfo_velodyne_points_->header.stamp.nsec);
}

void VehicleStateCallback(const pharos_msgs::StateStamped2016ConstPtr& msg){
    new_vel_ = msg->state.velocity;
    new_steering_ = msg->state.wheel_angle;

    // printf("CT:%d.%d\n",msg->header.stamp.sec%10,msg->header.stamp.nsec);
}

void OdomCallback(const nav_msgs::OdometryConstPtr &msg){
    static tf::TransformListener listener;
    static bool is_TF_init = false;
    if(!is_TF_init){
        try{
          listener.lookupTransform("odom", "map", ros::Time(0), transform_);
          is_TF_init = true;
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          return;
        }
        double r, p;
        tf::Matrix3x3(transform_.getRotation()).getRPY(r, p, yaw_trans_);
        std::cout<<"map TF init: "<<transform_.getOrigin().x()<<","<<transform_.getOrigin().y()<<","<<yaw_trans_<<std::endl;
    }


    static double x_temp, y_temp;
    x_temp = msg->pose.pose.position.x - transform_.getOrigin().x();
    y_temp = msg->pose.pose.position.y - transform_.getOrigin().y();

    odom_x_ = x_temp*cos(-yaw_trans_) - y_temp*sin(-yaw_trans_);
    odom_y_ = x_temp*sin(-yaw_trans_) + y_temp*cos(-yaw_trans_);
    odom_z_ = msg->pose.pose.position.z;

    RPY_ rpy;
    rpy = QuaterToRPY(*msg);
    odom_roll_ = rpy.roll;
    odom_pitch_ = rpy.pitch;
    odom_theta_ = rpy.yaw - yaw_trans_;
    odom_theta_ = angles::normalize_angle_positive(odom_theta_);
    odom_init_ = true;

}


int Init(){
    node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
    pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    pnode_->param("use_cuda", use_cuda, true);
    pnode_->param("use_roadinfo", use_roadinfo, true);
    pnode_->param("use_verticalinfo", use_verticalinfo, true);
    pnode_->param<int>("num_particle", num_particle_, 1000);
    pnode_->param<int>("num_gps_particle", num_gps_particle_, 5);
    pnode_->param<float>("vertical_gain", vertical_gain_, 0.5);


    particle_pub_ = node_->advertise<geometry_msgs::PoseArray>("particle_posearray",1);
    odom_pub_ = node_->advertise<nav_msgs::Odometry>("/odom/mcl",1);

    roadinfo_map_sub_ = node_->subscribe<nav_msgs::OccupancyGrid>("/roadinfo_map",1, RoadinfoMapCallback);
    roadinfo_velodyne_point_sub_ = node_->subscribe<sensor_msgs::PointCloud2>("/lidar_pcl/mcl/roadinfo",1,RoadinfoVelodyneCallback);
    verticalinfo_map_sub_ = node_->subscribe<nav_msgs::OccupancyGrid>("/verticalinfo_map",1, VerticalinfoMapCallback);
    verticalinfo_velodyne_point_sub_ = node_->subscribe<sensor_msgs::PointCloud2>("/lidar_pcl/mcl/verticalinfo",1, VerticalinfoVelodyneCallback);

    vehicle_sub_ = node_->subscribe<pharos_msgs::StateStamped2016>("/vehicle/state2016",1, VehicleStateCallback);
    odom_sub_ = node_->subscribe<nav_msgs::Odometry>("/odom/ublox",1,OdomCallback);

    return 0;
}


int main(int argc, char** argv){
    ros::init(argc,argv,"MCL_kut");

    if(Init()){
        ROS_FATAL("pharos_MCL initialization failed");
        return -1;
    }

    std::string use_cuda_s, use_roadinfo_s, use_verticalinfo_s;
    if(use_cuda)        use_cuda_s = "true";
    else        use_cuda_s = "false";
    if(use_roadinfo)        use_roadinfo_s = "true";
    else        use_roadinfo_s = "false";
    if(use_verticalinfo)        use_verticalinfo_s = "true";
    else        use_verticalinfo_s = "false";

    printf("|---------------------------|\n");
    printf("    num_partile : %d\n    num_gps_particle : %d\n    use_cuda : %s\n    use_roadinfo : %s\n    use_verticalinfo : %s",num_particle_,num_gps_particle_,use_cuda_s.c_str(),use_roadinfo_s.c_str(),use_verticalinfo_s.c_str());
    printf("\n|----------------------------|\n");

    ros::spin();


    CUDAFree();

    return 0;
}

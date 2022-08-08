#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <pharos_msgs/StateStamped2016.h>
#include <pharos_msgs/ObjectInfoArray.h>
#include <pharos_msgs/ObjectPose.h>
#include <pharos_msgs/ObjectSize.h>
#include <pharos_msgs/ObjectInfo.h>
#include <pharos_msgs/ObjectCollision.h>
#include <string>
#include <vector>



using std::string;
std::vector<int> matching_type_1;
std::vector<int> matching_type_2;
std::vector<int> matching_type_3;
std::vector<int> matching_type_4;
std::vector<int> matching_type_5;
pharos_msgs::ObjectInfoArrayPtr old_Infoarray(new pharos_msgs::ObjectInfoArray);
nav_msgs::Odometry Odom_;
pharos_msgs::ObjectInfoArrayPtr last_added_Infoarray(new pharos_msgs::ObjectInfoArray);
pharos_msgs::ObjectInfoPtr last_added_objectpose(new pharos_msgs::ObjectInfo);
geometry_msgs::Pose pose;
std_msgs::Header header;
std::vector<int> added_size_vec;
std::vector<int> matching_type1_size_vec;
std::vector<int> matching_type2_size_vec;
std::vector<int> matching_type3_size_vec;
std::vector<int> matching_type4_size_vec;
std::vector<int> matching_type5_size_vec;

ros::Time curr_t_;
ros::Time past_t_;
int new_ID = 100;
int step = 0;
bool init = false;
float camera_roll,camera_pitch,camera_yaw, camera_x , camera_y , camera_z;
double dmin_deadline,maintaincount_deadline,step_deadline;

class tracking_ {
public:


    tracking_() {
        nh = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pnh = ros::NodeHandlePtr(new ros::NodeHandle(""));


        Odom_sub = nh->subscribe("/odom/ekf", 10, &tracking_::tracking_odomCB, this);
        msg_sub = nh->subscribe("/pharos_transformed_msg", 10, &tracking_::tracking_CB, this); //require modification
        center_pub = nh->advertise<sensor_msgs::PointCloud2>("/one_clustering", 10);
        msg_pub = nh->advertise<pharos_msgs::ObjectInfoArray>("/pharos_tracked2_msg", 10);
        vis_pub = nh->advertise<visualization_msgs::Marker>("/tracking_marker",10);
        vis2_pub = nh->advertise<visualization_msgs::MarkerArray>("/tracking_markerarray",10);
        vis3_pub = nh->advertise<visualization_msgs::MarkerArray>("/speed_markerarray",10);

        nh->param<float>("/pharos_tf_broadcaster_node/camera_roll",camera_roll, 0);
        nh->param<float>("/pharos_tf_broadcaster_node/camera_pitch",camera_pitch, 0);
        nh->param<float>("/pharos_tf_broadcaster_node/camera_yaw",camera_yaw, 0);
        nh->param<float>("/pharos_tf_broadcaster_node/camera_x",camera_x, 0);
        nh->param<float>("/pharos_tf_broadcaster_node/camera_y",camera_y, 0);
        nh->param<float>("/pharos_tf_broadcaster_node/camera_z",camera_z, 0);

        pnh->param<double>("dmin_deadline",dmin_deadline, 0);
        pnh->param<double>("maintaincount_deadline",maintaincount_deadline, 0);
        pnh->param<double>("step_deadline",step_deadline, 0);
    }

    void tracking_odomCB(const nav_msgs::OdometryPtr msg) //subscribe odom
    {
        Odom_ = *msg; // 오돔 받기
    }

    struct point {
        double x, y;
    };

    double Distance(point p1, point p2) //set distance
    {
        double distance;
        distance = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2)); // 거리
        return distance;
    }

    void tracking_CB(const pharos_msgs::ObjectInfoArrayPtr msg) {

        // std::cout << "1" << std::endl;

        //객체수가 0일때 coredump가 뜨는 것을 방지한다.
        if(msg->objects.size()==0)
        {
            return;
        }



        // cout << " -------------new object --------------" << endl;
        pcl::PointCloud<pcl::PointXYZI>::Ptr newtransformed_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr expected_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr centerpose_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);

        pharos_msgs::ObjectInfoArrayPtr new_Infoarray (new pharos_msgs::ObjectInfoArray);
        new_Infoarray = msg;
        pharos_msgs::ObjectInfoArrayPtr newtransformed_Infoarray(new pharos_msgs::ObjectInfoArray);
        pharos_msgs::ObjectInfoArrayPtr oldtransformed_Infoarray(new pharos_msgs::ObjectInfoArray);
        pharos_msgs::ObjectInfoArrayPtr expected_Infoarray(new pharos_msgs::ObjectInfoArray);
        pharos_msgs::ObjectInfoArrayPtr odom_Infoarray(new pharos_msgs::ObjectInfoArray);
        pharos_msgs::ObjectInfoArrayPtr undefinded_Infoarray(new pharos_msgs::ObjectInfoArray);
        pharos_msgs::ObjectInfoArrayPtr undefinded2_Infoarray(new pharos_msgs::ObjectInfoArray);
        pharos_msgs::ObjectInfoArrayPtr not_tracked_Infoarray(new pharos_msgs::ObjectInfoArray);
        pharos_msgs::ObjectInfoArrayPtr added_Infoarray(new pharos_msgs::ObjectInfoArray);
        pharos_msgs::ObjectInfoArrayPtr real_added_Infoarray(new pharos_msgs::ObjectInfoArray);
        pharos_msgs::ObjectInfoArrayPtr publish_Infoarray(new pharos_msgs::ObjectInfoArray);
        pharos_msgs::ObjectInfoPtr real_added_objectpose(new pharos_msgs::ObjectInfo);
        pharos_msgs::ObjectInfoPtr added_objectpose(new pharos_msgs::ObjectInfo);
        pharos_msgs::ObjectInfoPtr undefinded_objectpose(new pharos_msgs::ObjectInfo);
        pharos_msgs::ObjectInfoPtr undefinded2_objectpose(new pharos_msgs::ObjectInfo);
        pharos_msgs::ObjectInfoPtr objectpose(new pharos_msgs::ObjectInfo);
        pharos_msgs::ObjectInfoPtr expected_objectpose(new pharos_msgs::ObjectInfo);
        pharos_msgs::ObjectInfoPtr expected_objectInfo(new pharos_msgs::ObjectInfo);
        pharos_msgs::ObjectInfoPtr not_tracked_objectpose(new pharos_msgs::ObjectInfo);
        pharos_msgs::ObjectInfoPtr publish_objectpose(new pharos_msgs::ObjectInfo);
        pharos_msgs::ObjectInfoPtr fixed_objectpose(new pharos_msgs::ObjectInfo);

        int maintained_id_count = 0;
        int added_size = 0;
        int matching_type1_size=0; // size  나중에 커지면 자를라고 변수선
        int matching_type2_size=0;
        int matching_type3_size=0;
        int matching_type4_size=0;
        int matching_type5_size=0;

        curr_t_ = msg->header.stamp;
        double dt = (curr_t_ - past_t_).toSec(); // dt 선언

        //transform to odom tf
        int object_ID = 0;
        for (int i = 0; i < new_Infoarray->objects.size(); i++) {

            Eigen::RowVector4f cam_pose, cam_min, cam_max, cam_colmin, cam_colmax;
            cam_pose(0) = new_Infoarray->objects[i].pose.x;
            cam_pose(1) = new_Infoarray->objects[i].pose.y;
            cam_pose(2) = new_Infoarray->objects[i].pose.z;
            cam_pose(3) = 1.0;

            cam_min(0) = new_Infoarray->objects[i].size.min_x;
            cam_min(1) = new_Infoarray->objects[i].size.min_y;
            cam_min(2) = new_Infoarray->objects[i].size.min_z;
            cam_min(3) = 1.0;

            cam_max(0) = new_Infoarray->objects[i].size.max_x;
            cam_max(1) = new_Infoarray->objects[i].size.max_y;
            cam_max(2) = new_Infoarray->objects[i].size.max_z;
            cam_max(3) = 1.0;

            cam_colmin(0) = new_Infoarray->objects[i].collision.min_x;
            cam_colmin(1) = new_Infoarray->objects[i].collision.min_y;
            cam_colmin(2) = new_Infoarray->objects[i].collision.min_z;
            cam_colmin(3) = 1.0;

            cam_colmax(0) = new_Infoarray->objects[i].collision.max_x;
            cam_colmax(1) = new_Infoarray->objects[i].collision.max_y;
            cam_colmax(2) = new_Infoarray->objects[i].collision.max_z;
            cam_colmax(3) = 1.0;

            Eigen::AngleAxisf init_rotation01_x((0.0), Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf init_rotation01_y((0.0), Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf init_rotation01_z((0.0), Eigen::Vector3f::UnitZ());  // velo <-> imu

            Eigen::Quaternionf q;
            q.x() = Odom_.pose.pose.orientation.x;
            q.y() = Odom_.pose.pose.orientation.y;
            q.z() = Odom_.pose.pose.orientation.z;
            q.w() = Odom_.pose.pose.orientation.w;
            auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

            Eigen::AngleAxisf init_rotation02_x(euler[0], Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf init_rotation02_y(euler[1], Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf init_rotation02_z(euler[2], Eigen::Vector3f::UnitZ()); // imu <-> odom

            Eigen::AngleAxisf init_rotation03_x ((1.07*M_PI/2),Eigen::Vector3f::UnitX ());
            Eigen::AngleAxisf init_rotation03_y ((M_PI),Eigen::Vector3f::UnitY ());
            Eigen::AngleAxisf init_rotation03_z ((0.935*M_PI/2),Eigen::Vector3f::UnitZ ()); // velo <-> cam

            Eigen::AngleAxisf init_rotation04_x ((camera_roll),Eigen::Vector3f::UnitX ());
            Eigen::AngleAxisf init_rotation04_y ((camera_pitch),Eigen::Vector3f::UnitY ());
            Eigen::AngleAxisf init_rotation04_z ((camera_yaw),Eigen::Vector3f::UnitZ ()); // imu <-> cam

            Eigen::Translation3f init_translation01(0.0,
                                                    0.0,
                                                    0.7);

            Eigen::Translation3f init_translation02((Odom_.pose.pose.position.x),
                                                    (Odom_.pose.pose.position.y),
                                                    (Odom_.pose.pose.position.z));

            Eigen::Translation3f init_translation03 (1,0,-0.8);

            Eigen::Translation3f init_translation04 (camera_x,camera_y,camera_z);

            Eigen::Matrix4f RT01 = (init_translation01 * init_rotation01_z * init_rotation01_y *
                                    init_rotation01_x).matrix();
            Eigen::Matrix4f RT02 = (init_translation02 * init_rotation02_z * init_rotation02_y *
                                    init_rotation02_x).matrix();
            Eigen::Matrix4f RT03 = (init_translation03 * init_rotation03_z * init_rotation03_y *
                                    init_rotation03_x).matrix();
            Eigen::Matrix4f RT04 = (init_translation04 * init_rotation04_z * init_rotation04_y *
                                    init_rotation04_x).matrix();
            Eigen::Matrix4f RT12 = RT02*RT04;

            Eigen::RowVector4f odom_pose, odom_min, odom_max,odom_colmin,odom_colmax;
            odom_pose.transpose() = RT12 * cam_pose.transpose();
            odom_min.transpose() = RT12 * cam_min.transpose();
            odom_max.transpose() = RT12 * cam_max.transpose();
            odom_colmin.transpose() = RT12 * cam_colmin.transpose();
            odom_colmax.transpose() = RT12 * cam_colmax.transpose();

            *objectpose = msg->objects[i];

            objectpose->pose.x = odom_pose(0);
            objectpose->pose.y = odom_pose(1);
            objectpose->pose.z = odom_pose(2);
            objectpose->pose.theta = 0;

            objectpose->size.min_x = odom_min(0);
            objectpose->size.min_y = odom_min(1);
            objectpose->size.min_z = odom_min(2);

            objectpose->size.max_x = odom_max(0);
            objectpose->size.max_y = odom_max(1);
            objectpose->size.max_z = odom_max(2);

            objectpose->collision.min_x=0;
            objectpose->collision.min_y=0;
            objectpose->collision.min_z=0;

            objectpose->collision.max_x=0;
            objectpose->collision.max_y=0;
            objectpose->collision.max_z=0;

            objectpose->id=object_ID;
            object_ID ++;


            objectpose->maintain_count = 0;
            odom_Infoarray->objects.push_back(*objectpose);



            pcl::PointXYZI newpoint; //make pointcloud
            newpoint.x = odom_Infoarray->objects[i].pose.x;
            newpoint.y = odom_Infoarray->objects[i].pose.y;
            newpoint.z = odom_Infoarray->objects[i].pose.z;
            newpoint.intensity=i*5;

            newtransformed_pointcloud->points.push_back(newpoint);

        }

        if (!init) {// !init = init == false // 처음은 아무 데이터가 없으니까 일단 저장부터
            init = true;

            for(int i=0; i<odom_Infoarray->objects.size(); i++) { //  초기값 설정

                odom_Infoarray->objects[i].id = i+1;
                old_Infoarray = odom_Infoarray;
            }
        }
        else
        {

        // std::cout << "2" << std::endl;


            //  Calcualte expected_infoarray
            for (int i = 0; i < old_Infoarray->objects.size(); i++) // 익스펙트 입력
            {

                expected_Infoarray = old_Infoarray;
                expected_Infoarray->objects[i].pose.x += expected_Infoarray->objects[i].pose.velo_x*dt;
                expected_Infoarray->objects[i].pose.y += expected_Infoarray->objects[i].pose.velo_y*dt;

//                cout << "expected_Infoarray_id : " <<expected_Infoarray->objects[i].id <<endl;
//                cout << "expected_Infoarray_x : " <<expected_Infoarray->objects[i].pose.x <<endl;
//                cout << "expected_Infoarray_y : " <<expected_Infoarray->objects[i].pose.y<<endl;
//                cout << "expected_Infoarray_type : " <<expected_Infoarray->objects[i].type <<endl;

            }

            // cout<<"expected_size = " <<expected_Infoarray->objects.size() <<endl;

              // Calcuate distance of new_infoarray to expected_infoarray
            float d_deadline = 3; // 3미터 제한 거리 가까운걸로 하려고
            std::vector<std::vector<std::vector<double>>> vec_odom_array;

            std::vector<int> tracked_index_array;// 트래킹 된 인덱스 저장 하려는 저장소
            for (int i = 0; i < odom_Infoarray->objects.size(); i++) // 오돔 전체에 대한 반복문
            {
                double d_min = 10000;
                int tracked_index = 0;

                for (int j = 0; j < expected_Infoarray->objects.size(); j++)
                {

                    point a = {odom_Infoarray->objects[i].pose.x, odom_Infoarray->objects[i].pose.y};
                    point b = {expected_Infoarray->objects[j].pose.x, expected_Infoarray->objects[j].pose.y};

                    double d = Distance(a, b);

                    //d_min= minimum d
                    if (d < d_min)
                    {
                        d_min = d;

                        tracked_index = j; // 트랙킹 완료되엇으면 j에 저장
                    }
                }
                //odom pose영역안에 들어온 ex_infoarray 중에 maintain이 가장 높은 것을 골라낸다.
                int max_expected_index = 0;
                int max_maintain_count = 0;
                double x_max = odom_Infoarray->objects[i].pose.x+2.5; // 5사이에에 있으면 같ㅇ느 물체 하려고
                double x_min = odom_Infoarray->objects[i].pose.x-2.5;
                double y_max = odom_Infoarray->objects[i].pose.y+2.5;
                double y_min = odom_Infoarray->objects[i].pose.y-2.5;
                int ex_in_odom_count = 0;

                for (int k = 0; k < expected_Infoarray->objects.size(); k++) {

                    if
                    (
                            expected_Infoarray->objects[k].pose.x>x_min &&
                            expected_Infoarray->objects[k].pose.x<x_max &&
                            expected_Infoarray->objects[k].pose.y>y_min &&
                            expected_Infoarray->objects[k].pose.y<y_max
                    )
                    {
                        ex_in_odom_count ++;
                        if (expected_Infoarray->objects[k].maintain_count > max_maintain_count)
                        {
                            max_maintain_count=expected_Infoarray->objects[k].maintain_count;

                double x_min = odom_Infoarray->objects[i].pose.x-2.5;
                            max_expected_index = k;
                        }
                    }
                }
                if(ex_in_odom_count == 0 or 1) // 0이나 1이면 할 필요가 없으니까 이거 했음
                {max_expected_index = tracked_index;}

                // cout << "들어온 expected_maintain_count :: " << max_maintain_count <<endl;



                tracked_index_array.push_back(expected_Infoarray->objects[tracked_index].id); // 트래킹된 배열들을 저장함.


                std::vector<double> vec_d_min; // 클래스로 나중에 선언할 추상화 할 객체들
                vec_d_min.push_back(d_min);
                std::vector<double> vec_odom_Infoarray_id;
                vec_odom_Infoarray_id.push_back(odom_Infoarray->objects[i].id);
                std::vector<double> vec_expected_Infoarray_id;
                vec_expected_Infoarray_id.push_back(expected_Infoarray->objects[max_expected_index].id);
                std::vector<double> vec_odom_Infoarray_pose_x;
                vec_odom_Infoarray_pose_x.push_back(odom_Infoarray->objects[i].pose.x);
                std::vector<double> vec_odom_Infoarray_pose_y;
                vec_odom_Infoarray_pose_y.push_back(odom_Infoarray->objects[i].pose.y);
                std::vector<double> vec_old_Infoarray_pose_x;
                vec_old_Infoarray_pose_x.push_back(expected_Infoarray->objects[max_expected_index].pose.x);
                std::vector<double> vec_old_Infoarray_pose_y;
                vec_old_Infoarray_pose_y.push_back(expected_Infoarray->objects[max_expected_index].pose.y);
                std::vector<double> vec_0;
                vec_0.push_back(0);
                std::vector<double> vec_expected_Infoarray_velo_x;
                vec_expected_Infoarray_velo_x.push_back(expected_Infoarray->objects[max_expected_index].pose.velo_x);
                std::vector<double> vec_expected_Infoarray_velo_y;
                vec_expected_Infoarray_velo_y.push_back(expected_Infoarray->objects[max_expected_index].pose.velo_x);
                std::vector<double> vec_expected_Infoarray_maintain_count;
                vec_expected_Infoarray_maintain_count.push_back(expected_Infoarray->objects[max_expected_index].maintain_count);
                std::vector<double> vec_expected_Infoarray_type;
                //이거 나중에 하기로 해쎄었다
                if(expected_Infoarray->objects[max_expected_index].type==0&&odom_Infoarray->objects[i].type!=0)
                {
                    vec_expected_Infoarray_type.push_back(odom_Infoarray->objects[i].type);
                }
                else
                    vec_expected_Infoarray_type.push_back(expected_Infoarray->objects[max_expected_index].type);

                std::vector<double> vec_odom_Infoarray_type;
                vec_odom_Infoarray_type.push_back(odom_Infoarray->objects[i].type);
                std::vector<double> vec_expected_Infoarray_theta;
                vec_expected_Infoarray_theta.push_back(expected_Infoarray->objects[max_expected_index].pose.theta);
                std::vector<double> vec_expected_Infoarray_collision_min_y;
                vec_expected_Infoarray_collision_min_y.push_back(expected_Infoarray->objects[max_expected_index].collision.min_y);
                std::vector<double> vec_expected_Infoarray_theta_count;
                vec_expected_Infoarray_theta_count.push_back(expected_Infoarray->objects[max_expected_index].theta_count);
                std::vector<double> vec_expected_Infoarray_speed;
                vec_expected_Infoarray_speed.push_back(expected_Infoarray->objects[max_expected_index].speed);

                std::vector<std::vector<double>> vec_odom; // 하나의 객체의 정보

                vec_odom.push_back(vec_d_min);//0
                vec_odom.push_back(vec_odom_Infoarray_id);//1
                vec_odom.push_back(vec_expected_Infoarray_id);//2
                vec_odom.push_back(vec_odom_Infoarray_pose_x);//3
                vec_odom.push_back(vec_odom_Infoarray_pose_y);//4
                vec_odom.push_back(vec_old_Infoarray_pose_x);//5
                vec_odom.push_back(vec_old_Infoarray_pose_y);//6
                vec_odom.push_back(vec_expected_Infoarray_velo_x); //7velo_x,y 담을 그릇
                vec_odom.push_back(vec_expected_Infoarray_velo_y);//8
                vec_odom.push_back(vec_0); //undefined x,y 담을 그릇 9
                vec_odom.push_back(vec_0);//10
                vec_odom.push_back(vec_0); //11 undefined2 x,y 담을 그릇
                vec_odom.push_back(vec_0); //12
                vec_odom.push_back(vec_expected_Infoarray_maintain_count); //13 id유지하면 1추가 , 안하면 0 추가
                vec_odom.push_back(expected_Infoarray->objects[tracked_index].x_velo); //x_velo담자14
                vec_odom.push_back(expected_Infoarray->objects[tracked_index].y_velo); //x_velo담자15
                vec_odom.push_back(vec_0); //16 이전 스텝의 id
                vec_odom.push_back(vec_0);//17 greedy strategy 적용되었는가
                vec_odom.push_back(vec_expected_Infoarray_type);//18
                vec_odom.push_back(vec_odom_Infoarray_type);//19
                vec_odom.push_back(vec_expected_Infoarray_theta);//20
                vec_odom.push_back(expected_Infoarray->objects[tracked_index].theta);//21
                vec_odom.push_back(vec_expected_Infoarray_collision_min_y);//22
                vec_odom.push_back(vec_expected_Infoarray_theta_count);//23
                vec_odom.push_back(vec_expected_Infoarray_speed);//24

                vec_odom_array.push_back(vec_odom);

            }

            //tracking이 안된 객체 거르기 not트랙 인포어레이에 보관
            for (int i=0; i<expected_Infoarray->objects.size();i++)
            {
                int not_tracked_index;
                for (int k = 0; k < tracked_index_array.size(); k++)
                {
                    if (expected_Infoarray->objects[i].id != tracked_index_array[k])
                    {
                        not_tracked_index = i;
                    }
                    else
                    {
                        not_tracked_index = -1;
                        break;
                    }

                }
                if (not_tracked_index != -1)
                {
                    *not_tracked_objectpose = expected_Infoarray->objects[not_tracked_index];
                    not_tracked_Infoarray->objects.push_back(*not_tracked_objectpose);
                }
            }
            // std::cout << "3" << std::endl;


            //not_tracked_infoarray를 unde2에 , unde2를 added에 담는다 ! (조건: id유지횟수 10회)
            for (int i=0; i < not_tracked_Infoarray->objects.size(); i++)
            {
                undefinded2_Infoarray->objects.push_back(not_tracked_Infoarray->objects[i]);
            }

            for(int i=0; i<undefinded2_Infoarray->objects.size();i++)
            {
                if (undefinded2_Infoarray->objects[i].maintain_count > 9)
                {
                    *undefinded2_objectpose = undefinded2_Infoarray->objects[i];
                    added_Infoarray->objects.push_back(*undefinded2_objectpose);
                }
            }


            //sort를 통한 오름차순정렬
            for (int k = 0; k < vec_odom_array.size(); k++)
            {
                for (int l = 0; l < vec_odom_array[k].size(); l++)
                {
                    std::sort(vec_odom_array.begin(), vec_odom_array.end()); // 처음 부터 vec odom 끝까지 오름차순  정렬
                }
            }


            // 비교한  객체를  비교대상에서  제외시킨다. + 속도를 부여 한다.(대신, d_min이 3보다 작을때만)
            /*
             * 속도 계산해서 넣어주고
             */
            for (int k = 0; k < vec_odom_array.size(); k++)
            {
                double expected_id;
                double velo_x;
                double velo_y;
                double undefined_id;
                double undefined_x;
                double undefined_y;
                expected_id = vec_odom_array[k][2][0];

                if (vec_odom_array[k][0][0] < d_deadline) // 아주 잘 된 경우
                {
                    velo_x = (vec_odom_array[k][3][0] - vec_odom_array[k][5][0]) / dt;
                    velo_y = (vec_odom_array[k][4][0] - vec_odom_array[k][6][0]) / dt;
                    undefined_x = 10000;
                    undefined_y = 10000;
                    undefined_id = 0;
                    vec_odom_array[k][13][0] += 1;
                }
                else // 데드라인 보다 커졌네
                {
                    // cout << "new__expected_d_min : " << vec_odom_array[k][0][0] << endl;
                    undefined_id = expected_id;
                    vec_odom_array[k][2][0] = new_ID;
                    // cout << "new__expected_id : " << new_ID << endl;
                    new_ID++;

                    if (vec_odom_array[k][13][0] < 9) // 9 이하 즉 버릴거
                    {
                        undefined_x = 10000;
                        undefined_y = 10000;
                        velo_x = 0;
                        velo_y = 0;
                        vec_odom_array[k][13][0] = 0;
                        vec_odom_array[k][22][0] = 0;
                        vec_odom_array[k][23][0] = 0;
                    }

                    else
                    { // 포즈 일단 저장, 내비두는 거
                        undefined_x = vec_odom_array[k][5][0];
                        undefined_y = vec_odom_array[k][6][0];
                        velo_x = vec_odom_array[k][7][0];
                        velo_y = vec_odom_array[k][8][0];
                        vec_odom_array[k][13][0] += 0;
                    }

                }
//                for (int l = k+1; l <vec_odom_array.size(); l++) {
//                    if(vec_odom_array[l][2][0]==expected_id)
//                    {
//                        cout << "greedy work :" << expected_id << "-->"<< new_ID<< endl;
//                    vec_odom_array[l][2][0]=new_ID;
//                    new_ID++;
//                    velo_x = 0;개
//                    velo_y = 0;
//                    undefined_id = 0;
//                    undefined_x = 10000;
//                    undefined_y = 10000;
//                    vec_odom_array[l][13][0] = 0; //id유지 횟수 초기화
//                    vec_odom_array[l][18][0] = 0;
//                    vec_odom_array[l][20][0] = 0;
//                    vec_odom_array[l][22][0] = 0;
//                    vec_odom_array[l][23][0] = 0;
//                    }
//                }

                vec_odom_array[k][7][0]=velo_x;
                vec_odom_array[k][8][0]=velo_y; // 마지막 최종 정리
                vec_odom_array[k][9][0]=undefined_x;
                vec_odom_array[k][10][0]=undefined_y;
                vec_odom_array[k][16][0]=undefined_id;

            }

            for (int i = 0; i < odom_Infoarray->objects.size(); i++)
            {

                odom_Infoarray->objects[vec_odom_array[i][1][0]/*0의 의미는 제일 가까운것을 의미 */].id = vec_odom_array[i][2][0];
                odom_Infoarray->objects[vec_odom_array[i][1][0]].pose.x = vec_odom_array[i][3][0];
                odom_Infoarray->objects[vec_odom_array[i][1][0]].pose.y = vec_odom_array[i][4][0];
                odom_Infoarray->objects[vec_odom_array[i][1][0]].pose.velo_x = vec_odom_array[i][7][0];
                odom_Infoarray->objects[vec_odom_array[i][1][0]].pose.velo_y = vec_odom_array[i][8][0];
                odom_Infoarray->objects[vec_odom_array[i][1][0]].maintain_count = vec_odom_array[i][13][0];
                odom_Infoarray->objects[vec_odom_array[i][1][0]].x_velo = vec_odom_array[i][14];
                odom_Infoarray->objects[vec_odom_array[i][1][0]].y_velo = vec_odom_array[i][15];
                odom_Infoarray->objects[vec_odom_array[i][1][0]].type = vec_odom_array[i][18][0];
                odom_Infoarray->objects[vec_odom_array[i][1][0]].pose.theta = vec_odom_array[i][20][0];
                odom_Infoarray->objects[vec_odom_array[i][1][0]].theta = vec_odom_array[i][21];
                odom_Infoarray->objects[vec_odom_array[i][1][0]].collision.min_y = vec_odom_array[i][22][0];
                odom_Infoarray->objects[vec_odom_array[i][1][0]].theta_count = vec_odom_array[i][23][0];
                odom_Infoarray->objects[vec_odom_array[i][1][0]].speed = vec_odom_array[i][24][0];
                odom_Infoarray->objects[vec_odom_array[i][1][0]].vehicle_state = 1; //BSM = 0 , TRACKING = 1
                // cout << "vec_odom_id : " <<vec_odom_array[i][2][0]<<endl;
                // cout << "vec_odom_theta : " <<vec_odom_array[i][20][0]<<endl;
                // cout << "vec_odom_maintain : " <<vec_odom_array[i][13][0]<<endl;
                // cout << "vec_odom_x : " <<vec_odom_array[i][3][0]<<endl;
                // cout << "vec_odom_y : " <<vec_odom_array[i][4][0]<<endl;
//                odom_Infoarray.objects[vec_odom_array[i][1][0]]. = vec_odom_array[i][15];

                int matching_count_1,matching_count_2,matching_count_3,matching_count_4,matching_count_5;
                // 매칠 카운트 세기
                matching_count_1=0; // 매칭 카운트 아마 종류를 구분하는 것
                matching_count_2=0;
                matching_count_3=0;
                matching_count_4=0;
                matching_count_5=0;
                //type을 해당 id에 저장

                if(odom_Infoarray->objects[vec_odom_array[i][1][0]].type==1) // 타입이 1일 때,
                {
                    matching_type1_size ++;
                    matching_type_1.push_back(odom_Infoarray->objects[vec_odom_array[i][1][0]].id);
                }
                for(int j = 0; j < matching_type_1.size(); j++)
                {
                    if(odom_Infoarray->objects[vec_odom_array[i][1][0]].id==matching_type_1[j])
                    {
                        matching_count_1 ++;
                    }
                }

                if(odom_Infoarray->objects[vec_odom_array[i][1][0]].type==2)
                {
                    matching_type2_size ++;
                    matching_type_2.push_back(odom_Infoarray->objects[vec_odom_array[i][1][0]].id);
                }
                for(int j = 0; j < matching_type_2.size(); j++)
                {
                    if(odom_Infoarray->objects[vec_odom_array[i][1][0]].id==matching_type_2[j])
                    {
                        matching_count_2 ++;
                    }
                }

                if(odom_Infoarray->objects[vec_odom_array[i][1][0]].type==3)
                {
                    matching_type3_size ++;
                    matching_type_3.push_back(odom_Infoarray->objects[vec_odom_array[i][1][0]].id);
                }
                for(int j = 0; j < matching_type_3.size(); j++)
                {
                    if(odom_Infoarray->objects[vec_odom_array[i][1][0]].id==matching_type_3[j])
                    {
                        matching_count_3 ++;
                    }
                }

                if(odom_Infoarray->objects[vec_odom_array[i][1][0]].type==4)
                {
                    matching_type4_size ++;
                    matching_type_4.push_back(odom_Infoarray->objects[vec_odom_array[i][1][0]].id);
                }
                for(int j = 0; j < matching_type_4.size(); j++)
                {
                    if(odom_Infoarray->objects[vec_odom_array[i][1][0]].id==matching_type_4[j])
                    {
                        matching_count_4 ++;
                    }
                }

                if(odom_Infoarray->objects[vec_odom_array[i][1][0]].type==5)
                {
                    matching_type5_size ++; // 100개 이상 넘어가면 자르려고 만듦
                    matching_type_5.push_back(odom_Infoarray->objects[vec_odom_array[i][1][0]].id);
                }
                for(int j = 0; j < matching_type_5.size(); j++)
                {
                    if(odom_Infoarray->objects[vec_odom_array[i][1][0]].id==matching_type_5[j])
                    {
                        matching_count_5 ++;
                    }
                }

                // 이때 한꺼번에 대입
                if(matching_count_1 > 2)
                {
                    odom_Infoarray->objects[vec_odom_array[i][1][0]].type=1;
                }
                if(matching_count_2 > 2)
                {
                    odom_Infoarray->objects[vec_odom_array[i][1][0]].type=2;
                }
                if(matching_count_3 > 2)
                {
                    odom_Infoarray->objects[vec_odom_array[i][1][0]].type=3;
                }
                if(matching_count_4 > 2)
                {
                    odom_Infoarray->objects[vec_odom_array[i][1][0]].type=4;
                }
                if(matching_count_5 > 2)
                {
                    odom_Infoarray->objects[vec_odom_array[i][1][0]].type=5;
                }

                *undefinded_objectpose = odom_Infoarray->objects[vec_odom_array[i][1][0]];
                undefinded_objectpose->pose.x = vec_odom_array[i][9][0];
                undefinded_objectpose->pose.y = vec_odom_array[i][10][0];
                undefinded_objectpose->id = vec_odom_array[i][16][0]; // 거릿값이 3보다 크고  maintain이 10 이상인 것들

                undefinded_Infoarray->objects.push_back(*undefinded_objectpose);

                //현재 odom만을 퍼블리시하자.
                *publish_objectpose = odom_Infoarray->objects[vec_odom_array[i][1][0]];
//                fixed_objectpose->pose.x = 10000;
//                fixed_objectpose->pose.y = 10000;

                publish_Infoarray->objects.push_back(*publish_objectpose);
//                publish_Infoarray->objects.push_back(*fixed_objectpose);



                string s;
                int x = odom_Infoarray->objects[vec_odom_array[i][1][0]].id;
                float xx = odom_Infoarray->objects[vec_odom_array[i][1][0]].pose.x;
                float vx = odom_Infoarray->objects[vec_odom_array[i][1][0]].pose.velo_x;
                float vy = odom_Infoarray->objects[vec_odom_array[i][1][0]].pose.velo_y;
                float ex_pose_x = vec_odom_array[i][5][0];
                float pose_x = vec_odom_array[i][3][0];
                float d_min = vec_odom_array[i][0][0];
                float ex_theta = vec_odom_array[i][22][0];
                float speed = vec_odom_array[i][24][0];
                int ty = odom_Infoarray->objects[vec_odom_array[i][1][0]].type;
                s = "ID : "+std::to_string(x)+"\nSPEED : "
                        +std::to_string(speed)+"\nTYPE : "
                        +std::to_string(ty) +"\nD_MIN : "
                        +std::to_string(d_min);



//visualization_msgs::Marker marker;
                visualization_msgs::Marker marker;
                marker.header.frame_id = "odom";
                marker.header.stamp = ros::Time::now();
                marker.ns = "my_namespace";
                marker.id = i;
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker.text = s;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = odom_Infoarray->objects[vec_odom_array[i][1][0]].pose.x;
                marker.pose.position.y = odom_Infoarray->objects[vec_odom_array[i][1][0]].pose.y;
                marker.pose.position.z = odom_Infoarray->objects[vec_odom_array[i][1][0]].pose.z+2;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 5;
                marker.scale.y = 0.5;
                marker.scale.z = 0.5;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
                vis_pub.publish( marker );


                visualization_msgs::MarkerArray markerarray;
                markerarray.markers.push_back(marker);
                vis2_pub.publish(markerarray);

                visualization_msgs::Marker speed_marker;
                speed_marker.header.frame_id = "odom";
                speed_marker.header.stamp = ros::Time();
                speed_marker.ns = "my_namespace";
                speed_marker.id = i;
                speed_marker.type = visualization_msgs::Marker::ARROW;
                speed_marker.text = s;
                speed_marker.action = visualization_msgs::Marker::ADD;
                speed_marker.pose.position.x = odom_Infoarray->objects[vec_odom_array[i][1][0]].pose.x;
                speed_marker.pose.position.y = odom_Infoarray->objects[vec_odom_array[i][1][0]].pose.y;
                speed_marker.pose.position.z = odom_Infoarray->objects[vec_odom_array[i][1][0]].pose.z+2;
                speed_marker.pose.orientation.x = 0.0;
                speed_marker.pose.orientation.y = 0.0;
                speed_marker.pose.orientation.z = 0.0;
                speed_marker.pose.orientation.w = 1.0;
                if(odom_Infoarray->objects[vec_odom_array[i][1][0]].speed != 0)
                {speed_marker.scale.x = odom_Infoarray->objects[vec_odom_array[i][1][0]].speed;}
                else
                {speed_marker.scale.x = 0.5;}
                speed_marker.scale.y = 0.5;
                speed_marker.scale.z = 0.5;
                speed_marker.color.a = 1.0;
                speed_marker.color.r = 1.0;
                speed_marker.color.g = 1.0;
                speed_marker.color.b = 0.0;
                speed_marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
                vis_pub.publish( speed_marker );


                visualization_msgs::MarkerArray speed_markerarray;
                speed_markerarray.markers.push_back(speed_marker);
                vis3_pub.publish(speed_markerarray);


            }
            // std::cout << "4" << std::endl;

            //비교객체 중 id를 부여하지 못한 expected객체를 따로 담아둔다.
            for(int i=0; i<undefinded_Infoarray->objects.size(); i++)
            {
                if(undefinded_Infoarray->objects[i].pose.x!=10000) // x가 만이 아닌 경우
                {
                    *undefinded_objectpose = undefinded_Infoarray->objects[i]; //
                    added_Infoarray->objects.push_back(*undefinded_objectpose); // 애드에 담아둠
                }
            }

        //added_infoarray 중에 odom_infoarray와 겹치는 것을 제외시켜준다.
            for (int i=0; i < added_Infoarray->objects.size(); i++)
            {
                int overlapped_added_index;
                for (int h = 0; h < odom_Infoarray->objects.size(); h++)
                {
                    if (added_Infoarray->objects[i].id == odom_Infoarray->objects[h].id)
                    {
                        overlapped_added_index = -1;
                        break;
                    }
                    else
                    {
                        overlapped_added_index = i; // i 일때는 필요가 없어 그냥 가는 거야 이거 나중에 안써
                    }
                }
                if (overlapped_added_index != -1) // 겹친 것만 버리는 if문 겹ㅊ치면 반복문 종료되고 푸시백된다.
                {
                    real_added_Infoarray->objects.push_back(added_Infoarray->objects[i]);
                }
            }

            //real_added_infoarray 중에 last_added_infoarray와 겹치는 것을 제외시켜준다. 그 알고리즘은 i와 -1의 관계에 있다. 증가 for문이므로
            for (int i=0; i<real_added_Infoarray->objects.size();i++) // id 제외
            {
                int overlapped_added_index=0;
                for (int h = 0; h<last_added_Infoarray->objects.size(); h++)
                { // last 지난번 스ㅌ텝에 추가된 것들

                    if (real_added_Infoarray->objects[i].id == last_added_Infoarray->objects[h].id)
//                    &&  real_added_Infoarray.objects[i].pose.x == last_added_Infoarray.objects[h].pose.x)
                    {
                        last_added_Infoarray->objects[h].pose.x = real_added_Infoarray->objects[i].pose.x;
                        last_added_Infoarray->objects[h].pose.y = real_added_Infoarray->objects[i].pose.y;
                        overlapped_added_index = -1;
                        break;

                    }
                    else
                    {
                        overlapped_added_index = i;
                    }
                }

                if (overlapped_added_index == -1)
                {
                    real_added_Infoarray->objects[i].pose.x = 10000;
                }
            }



            //따로 담긴 객체들을 다음 스텝에 사용할 수 있게 odom에 추가해준다.
            //추가할 때에 real_added에 겹치는 것은 제외하고 추가한다.
            for(int i=0; i<real_added_Infoarray->objects.size(); i++)
            {
                if(real_added_Infoarray->objects[i].pose.x != 10000)
                {
                    *last_added_objectpose = real_added_Infoarray->objects[i];
                    last_added_Infoarray->objects.push_back(*last_added_objectpose);
                    added_size ++;
                }
            }

            //last중에 odom과 겹치는 id를 제거해준다. 즉, 재활용에 성공하면 버림.
            for (int i=0; i<last_added_Infoarray->objects.size();i++)
            {
                int overlapped_added_index=0;
                for (int h = 0; h<odom_Infoarray->objects.size(); h++) {

                    if (last_added_Infoarray->objects[i].id == odom_Infoarray->objects[h].id)
                    {
                        overlapped_added_index = -1;
                        break;

                    }
                    else
                    {
                        overlapped_added_index = i;
                    }
                }
                if (overlapped_added_index == -1)
                {
                    last_added_Infoarray->objects[i].pose.x = 10000;
                }
            }

//            for (int j = 0; j < real_added_Infoarray.objects.size(); j++)
//            {cout<<"real_added_id : " <<real_added_Infoarray.objects[j].id<<endl;}
//            for(int i=0; i<last_added_Infoarray.objects.size(); i++)
//            {for (int j = 0; j < real_added_Infoarray.objects.size(); j++)
//                {
//                if (last_added_Infoarray.objects[i].pose.x == 10000)
//                {
//
//                        if (last_added_Infoarray.objects[i].id == real_added_Infoarray.objects[j].id)
//                        {
//                            last_added_Infoarray.objects[i].pose.x = real_added_Infoarray.objects[j].pose.x;
//                            last_added_Infoarray.objects[i].pose.y = real_added_Infoarray.objects[j].pose.y;
//                        }
//                    }
//                }
//            }
            //delay 방지를 위한 갯수제한
            /*
             * 너무 느려져서 지울 필요가 있음.
             */
            if (last_added_Infoarray->objects.size()>100)
            {
                int limited_infoarray_count = last_added_Infoarray->objects.size() - 100;
                last_added_Infoarray->objects.erase(last_added_Infoarray->objects.begin(),last_added_Infoarray->objects.begin()+limited_infoarray_count);
            }

            for(int i=0; i<last_added_Infoarray->objects.size(); i++)
            {
                if (last_added_Infoarray->objects[i].pose.x != 10000)
                {
                    *last_added_objectpose = last_added_Infoarray->objects[i];
                    odom_Infoarray->objects.push_back(*last_added_objectpose);
                }
            }


            //속도에 관한것임 속도가 너무 튀는 것을 방지하기 위한
            for(int i=0; i<odom_Infoarray->objects.size(); i++)
            {
                // cout << "ex_theta : " <<odom_Infoarray.objects[i].collision.min_y << endl;
                double ex_velo_x = odom_Infoarray->objects[i].pose.velo_x;
                double ex_velo_y = odom_Infoarray->objects[i].pose.velo_y;
                double a = 0.2;

                odom_Infoarray->objects[i].x_velo.push_back(odom_Infoarray->objects[i].pose.velo_x);
                odom_Infoarray->objects[i].y_velo.push_back(odom_Infoarray->objects[i].pose.velo_y);

                if(odom_Infoarray->objects[i].maintain_count<=10)
                {
                    odom_Infoarray->objects[i].pose.theta=atan2(odom_Infoarray->objects[i].pose.velo_y,odom_Infoarray->objects[i].pose.velo_x);
                }
                odom_Infoarray->objects[i].theta.push_back(odom_Infoarray->objects[i].pose.theta);

                if (odom_Infoarray->objects[i].x_velo.size() >9)
                {odom_Infoarray->objects[i].x_velo.erase(odom_Infoarray->objects[i].x_velo.begin());}
                if (odom_Infoarray->objects[i].y_velo.size() >9)
                {odom_Infoarray->objects[i].y_velo.erase(odom_Infoarray->objects[i].y_velo.begin());}

                if(odom_Infoarray->objects[i].maintain_count >9)
                {
                    double now_velo_x = accumulate(odom_Infoarray->objects[i].x_velo.begin(), odom_Infoarray->objects[i].x_velo.end(), 0.0)/ odom_Infoarray->objects[i].x_velo.size(); //accumulate math 함수
                    double now_velo_y = accumulate(odom_Infoarray->objects[i].y_velo.begin(), odom_Infoarray->objects[i].y_velo.end(), 0.0)/ odom_Infoarray->objects[i].y_velo.size();
                    odom_Infoarray->objects[i].pose.velo_x= a*now_velo_x+(1-a)*ex_velo_x;
                    odom_Infoarray->objects[i].pose.velo_y= a*now_velo_y+(1-a)*ex_velo_y;
                    accumulate(odom_Infoarray->objects[i].y_velo.begin(), odom_Infoarray->objects[i].y_velo.end(), 0.0)/ odom_Infoarray->objects[i].y_velo.size();

                }
                else
                    {
                        odom_Infoarray->objects[i].pose.velo_x=0;
                        odom_Infoarray->objects[i].pose.velo_y=0;
                    }
//                cout << "odom_id : " <<odom_Infoarray.objects[i].id << endl;
//
//                cout << "odom_theta : " <<atan2(odom_Infoarray.objects[i].pose.velo_y,odom_Infoarray.objects[i].pose.velo_x) << endl;
                odom_Infoarray->objects[i].speed=sqrt(pow(odom_Infoarray->objects[i].pose.velo_x, 2) + pow(odom_Infoarray->objects[i].pose.velo_y, 2));

                if(odom_Infoarray->objects[i].maintain_count ==10)
                {
                    odom_Infoarray->objects[i].pose.theta=accumulate(odom_Infoarray->objects[i].theta.begin(), odom_Infoarray->objects[i].theta.end(), 0.0)/ odom_Infoarray->objects[i].theta.size();
                    // cout << "odom_ten_theta : " <<odom_Infoarray->objects[i].pose.theta << endl;
                    if(fabs(odom_Infoarray->objects[i].pose.theta-atan2(odom_Infoarray->objects[i].pose.velo_y,odom_Infoarray->objects[i].pose.velo_x))<0.5 &&
                       fabs(odom_Infoarray->objects[i].pose.theta-atan2(odom_Infoarray->objects[i].pose.velo_y,odom_Infoarray->objects[i].pose.velo_x))>0.001)
                    {
                        // cout<<"차이값 : "<<odom_Infoarray->objects[i].pose.theta-atan2(odom_Infoarray->objects[i].pose.velo_y,odom_Infoarray->objects[i].pose.velo_x)<<endl;
                        odom_Infoarray->objects[i].pose.theta=atan2(odom_Infoarray->objects[i].pose.velo_y,odom_Infoarray->objects[i].pose.velo_x);
                        odom_Infoarray->objects[i].collision.min_y=atan2(odom_Infoarray->objects[i].pose.velo_y,odom_Infoarray->objects[i].pose.velo_x);
                        odom_Infoarray->objects[i].theta_count ++;
                    }
                    else
                    {
                        odom_Infoarray->objects[i].collision.min_y=atan2(odom_Infoarray->objects[i].pose.velo_y,odom_Infoarray->objects[i].pose.velo_x);
                        odom_Infoarray->objects[i].pose.theta=0;

                    }
                }

                if (odom_Infoarray->objects[i].theta.size() >10)
                {odom_Infoarray->objects[i].theta.erase(odom_Infoarray->objects[i].theta.begin());}

                //theta 변화율이 작을 경우 theta count를 올려준다.
                if(odom_Infoarray->objects[i].maintain_count >10) {
                    if (fabs(odom_Infoarray->objects[i].collision.min_y //여기서 콜리전은 쎄타값에 대한 신뢰도 (maintain_theta 와 비슷) 판단의 값으로 잠깐 메시지값을 빌려씀
                    -atan2(odom_Infoarray->objects[i].pose.velo_y, odom_Infoarray->objects[i].pose.velo_x)) < 0.15 &&
                        fabs(odom_Infoarray->objects[i].collision.min_y -
                             atan2(odom_Infoarray->objects[i].pose.velo_y, odom_Infoarray->objects[i].pose.velo_x)) >0.001)
                    { odom_Infoarray->objects[i].theta_count++; }
                    //theta변화율이 클 경우 theta값에 0을 입력시킨다. heading은 collision에 넣어놓는다.
                    else
                    {
                        odom_Infoarray->objects[i].collision.min_y=atan2(odom_Infoarray->objects[i].pose.velo_y,odom_Infoarray->objects[i].pose.velo_x);
                        odom_Infoarray->objects[i].pose.theta=0;
                        odom_Infoarray->objects[i].theta_count = 0;
                    }
                }

                //id유지횟수 10이상, heading 유지횟수 4이상만, 일정 변화율을 가질 때 theta값에 변화를 준다.(이동한다고 판단)
                if(odom_Infoarray->objects[i].maintain_count >10 && odom_Infoarray->objects[i].theta_count >3 )
                {
                    if(fabs(odom_Infoarray->objects[i].collision.min_y-atan2(odom_Infoarray->objects[i].pose.velo_y,odom_Infoarray->objects[i].pose.velo_x))<0.5 &&
                            fabs(odom_Infoarray->objects[i].collision.min_y-atan2(odom_Infoarray->objects[i].pose.velo_y,odom_Infoarray->objects[i].pose.velo_x))>0.001)
                    {
//                        cout<<"theta_count : "<<odom_Infoarray.objects[i].theta_count<<endl;
//                        cout<<"차이값 : "<<odom_Infoarray.objects[i].collision.min_y-atan2(odom_Infoarray.objects[i].pose.velo_y,odom_Infoarray.objects[i].pose.velo_x)<<endl;
                        odom_Infoarray->objects[i].pose.theta=atan2(odom_Infoarray->objects[i].pose.velo_y,odom_Infoarray->objects[i].pose.velo_x);
                        odom_Infoarray->objects[i].collision.min_y=atan2(odom_Infoarray->objects[i].pose.velo_y,odom_Infoarray->objects[i].pose.velo_x);
                    }
                }
                if(odom_Infoarray->objects[i].maintain_count <10){odom_Infoarray->objects[i].pose.theta=0;}
            }
        }

        odom_Infoarray->header.frame_id = "odom";
        odom_Infoarray->header.stamp = msg->header.stamp;
//        oldtransformed_pointcloud = newtransformed_pointcloud;
        past_t_ = curr_t_;

        publish_Infoarray->header.frame_id = "odom";
        publish_Infoarray->header.stamp = msg->header.stamp;
        msg_pub.publish(publish_Infoarray);

        sensor_msgs::PointCloud2 center_pointcloud;
        pcl::toROSMsg(*newtransformed_pointcloud,center_pointcloud);

        center_pointcloud.header.frame_id = "/odom";
        center_pointcloud.header.stamp = curr_t_;

        center_pub.publish(center_pointcloud);


        old_Infoarray=odom_Infoarray;
        // cout << "added_size : " << added_size << endl;

        added_size_vec.push_back(added_size);
        matching_type1_size_vec.push_back(matching_type1_size);
        matching_type2_size_vec.push_back(matching_type2_size);
        matching_type3_size_vec.push_back(matching_type3_size);
        matching_type4_size_vec.push_back(matching_type4_size);
        matching_type5_size_vec.push_back(matching_type5_size);


        //checking added_size_vec
//        for(int i=0; i< added_size_vec.size(); i++) {
//            cout << "added_[i] : " << added_size_vec[i] << endl;
//        }

        //checking added_infoarray
        for (int k = 0; k <last_added_Infoarray->objects.size(); k++) {

//                cout << "last_added_infoarray_id : " << last_added_Infoarray.objects[k].id << endl;
//            cout << "last_added_infoarray_x : " << last_added_Infoarray.objects[k].pose.x << endl;
//            cout << "last_added_infoarray_y : " << last_added_Infoarray.objects[k].pose.y << endl;
//            cout << "last_added_infoarray_maintain : " << last_added_Infoarray.objects[k].maintain_count << endl;

        }

        // cout<<"last_added_infoarray size : "<<last_added_Infoarray->objects.size()<<endl;
        // cout << "added_size_[0] : " <<added_size_vec[0]<<endl;
        //10번째 step부터 added_infoarray에 데이터를 지워나간다.
        if (step > 10 && last_added_Infoarray->objects.size() > added_size_vec[0]) {
            last_added_Infoarray->objects.erase(last_added_Infoarray->objects.begin(),
                                               last_added_Infoarray->objects.begin() + added_size_vec[0]);
            added_size_vec.erase(added_size_vec.begin());


            if(matching_type1_size_vec[0]!=0 && matching_type_1.size() >20)
            {
                matching_type_1.erase(matching_type_1.begin(),matching_type_1.begin()+matching_type1_size_vec[0]);
            }
            if(matching_type2_size_vec[0]!=0 && matching_type_2.size() >20)
            {
                matching_type_2.erase(matching_type_2.begin(),matching_type_2.begin()+matching_type2_size_vec[0]);
            }
            if(matching_type3_size_vec[0]!=0 && matching_type_3.size() >20)
            {
                matching_type_3.erase(matching_type_3.begin(),matching_type_3.begin()+matching_type3_size_vec[0]);
            }
            if(matching_type4_size_vec[0]!=0 && matching_type_4.size() >20)
            {
                matching_type_4.erase(matching_type_4.begin(), matching_type_4.begin() + matching_type4_size_vec[0]);
            }

            if(matching_type5_size_vec[0]!=0 && matching_type_5.size() >20)
            {
                matching_type_5.erase(matching_type_5.begin(),matching_type_5.begin()+matching_type5_size_vec[0]);
            }
            matching_type1_size_vec.erase(matching_type1_size_vec.begin());
            matching_type2_size_vec.erase(matching_type2_size_vec.begin());
            matching_type3_size_vec.erase(matching_type3_size_vec.begin());
            matching_type4_size_vec.erase(matching_type4_size_vec.begin());
            matching_type5_size_vec.erase(matching_type5_size_vec.begin());
        }


        // cout << "step: " << step << endl;



        //delay방지를 위해 저장시킨 infoarray의 갯수를 제한한다.

        // cout << "last_Infoarray_size : " << last_added_Infoarray->objects.size() << endl;
        step ++;
    }






protected:
    ros::NodeHandlePtr nh;
    ros::NodeHandlePtr pnh;

    ros::Publisher center_pub;
    ros::Publisher msg_pub;
    ros::Publisher vis_pub;
    ros::Publisher vis2_pub;
    ros::Publisher vis3_pub;

    ros::Subscriber posearray_sub;
    ros::Subscriber msg_sub;
    ros::Subscriber Odom_sub;

};






int main(int argc, char **argv)          //노드 메인함수 char **이다 계속 int하는데 주의할것
{
    ros::init(argc, argv, "tracking");       //노드명 초기화

    ROS_INFO("started filter node");
    tracking_ tracking;
    ros::spin();

    return 0;
}

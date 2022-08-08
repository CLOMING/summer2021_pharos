#include "utility.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>


#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std;


class obstacle_env {
private:
    ros::NodeHandle nh;
    ros::NodeHandlePtr pnh;

public:

    ros::Subscriber subObject;
    ros::Publisher pubObstacle;
    ros::Publisher pubMarker;
    std::string subObstacle,pubCluster;
    visualization_msgs::Marker line_list;
    float toler, z_diff_toler, height_toler;
    int min_size, max_size;


    obstacle_env(){


        // [ROS] Publish object B-Box
        line_list.header.frame_id = "vehicle_frame";
        line_list.ns = "bounding_box_Object";
        // line_list.pose.orientation.w = 1.0;
        line_list.id = 0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.08;
        line_list.color.r = 1.0;
        line_list.color.g = 1.0;
        line_list.color.b = 1.0;
        line_list.color.a = 0.7;
        line_list.action = visualization_msgs::Marker::ADD;

        pnh = ros::NodeHandlePtr(new ros::NodeHandle("~"));
        pnh->param<std::string>("subObstacle", subObstacle, "subObstacle");
        pnh->param<int>("min_size", min_size, 40);
        pnh->param<int>("max_size", max_size, 10000);
        pnh->param<float>("toler", toler, 0.4);
        pnh->param<float>("height_toler", height_toler, 0.5);
        pnh->param<float>("z_diff_toler", z_diff_toler, 0.4);
        pnh->param<std::string>("pubcluster", pubCluster, "pubcluster");

        subObject = nh.subscribe<sensor_msgs::PointCloud2>(subObstacle, 5, &obstacle_env::clustering_points, this);

        pubObstacle = nh.advertise<sensor_msgs::PointCloud2>(pubCluster,10);
        pubMarker = nh.advertise<visualization_msgs::Marker>("/lidar_pcl/obstacle/detectedBOX", 10);  // [PHAROS PathPlanner] FIX name for Preliminaries
    }

    void clustering_points(const sensor_msgs::PointCloud2ConstPtr& total_msg) {
        //std::cout << "Clustering Function" << std::endl;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_object_in (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_object (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_object_2 (new pcl::PointCloud<pcl::PointXYZ> ());

        sensor_msgs::PointCloud2 pubclustermsg;

        pcl::fromROSMsg(*total_msg, *pcl_object_in);

        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud (pcl_object_in);
        sor.setLeafSize (0.1, 0.1, 0.1);
        sor.filter (*pcl_object_in);

        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud (pcl_object_in);                //입력
        pass.setFilterFieldName ("z");             //적용할 좌표 축 (eg. Z축)
        pass.setFilterLimits(0.2, 3.0);          //적용할 값 (최소, 최대 값)
        pass.filter(*pcl_object);

        copyPointCloud(*pcl_object, *pcl_object_2);

        visualization_msgs::Marker line_list_reset;
        line_list.points = line_list_reset.points;
        line_list.action = visualization_msgs::Marker::ADD;
        pcl::PointCloud<pcl::PointXYZI>::Ptr object_clustered (new pcl::PointCloud<pcl::PointXYZI> ());

        if(pcl_object_2->empty()){
            std::cout << "empty" << std::endl;
        }else{
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_map_frame (new pcl::search::KdTree<pcl::PointXYZ>);
            tree_map_frame->setInputCloud (pcl_object_2);
            // cout << "PCL IN" << endl;
            std::vector<pcl::PointIndices> cluster_indices;
            // cout << "PCL IN1" << endl;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            // cout << "PCL IN2" << endl;
            ec.setClusterTolerance (toler); // 20cm
            // cout << "PCL IN3" << endl;
            ec.setMinClusterSize (min_size);
            ec.setMaxClusterSize (max_size);
            ec.setSearchMethod (tree_map_frame);
            ec.setInputCloud (pcl_object_2);
            // cout << "PCL IN4" << endl;
            ec.extract (cluster_indices);
            // cout << "PCL IN5" << endl;
            int j = 0;
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
                cout << "iter" << endl;
                pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_num (new pcl::PointCloud<pcl::PointXYZI> ());
                pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_num_vehicle (new pcl::PointCloud<pcl::PointXYZI> ());
                for(std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
                    pcl::PointXYZ pt = pcl_object_2->points[*pit];
                    pcl::PointXYZI pt2;
                    // pcl::PointXYZI minPt, maxPt;
                    
                    pt2.x=pt.x, pt2.y=pt.y, pt2.z=pt.z;
                    pt2.intensity=(float)(j);
                    // std::cout<< "intensity:" << j <<std::endl;
                    cluster_num->points.push_back(pt2);
                }

                pcl::PointXYZI minPt, maxPt;
                
                pcl::getMinMax3D (*cluster_num, minPt, maxPt);
                Eigen::Vector3f diff = minPt.getVector3fMap() - maxPt.getVector3fMap();
                float z_diff=maxPt.z-minPt.z;

                if (diff.squaredNorm () < 30 && z_diff>z_diff_toler && maxPt.z>height_toler){
                    *object_clustered += *cluster_num;
                    cout << "Cluster [" << j << "] : " << endl;
                    cout << "  Squared Norm Distance : " << diff.squaredNorm() << endl;
                    cout << "  z difference : " << z_diff <<endl;
                    j=j+10;
                    bounding_Box(cluster_num);
                    cout << "  Dist to obstacle : " << minPt.x <<endl;
                }
                else{ }
                
            }
        }
        cout << "PCL OUT" << endl;
        pcl::toROSMsg(*object_clustered, pubclustermsg);
        pubclustermsg.header.frame_id = "vehicle_frame";
        pubObstacle.publish(pubclustermsg);
        pubMarker.publish(line_list);
    }

    void bounding_Box(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_PCL){
        pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_extractor;
        feature_extractor.setInputCloud (input_PCL);
        feature_extractor.compute ();

        pcl::PointXYZI min_point_OBB_F;
        pcl::PointXYZI max_point_OBB_F;
        pcl::PointXYZI position_OBB_F;
        Eigen::Matrix3f rotational_matrix_OBB_F;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;
        Eigen::Vector3f mass_center;

        feature_extractor.getAABB (min_point_OBB_F, max_point_OBB_F);
        Eigen::Matrix<float, 8, 3> bounding_Box_Pose_TF;
        bounding_Box_Pose_TF(0,0) = min_point_OBB_F.x; bounding_Box_Pose_TF(0,1) = min_point_OBB_F.y; bounding_Box_Pose_TF(0,2) = min_point_OBB_F.z; 
        bounding_Box_Pose_TF(1,0) = min_point_OBB_F.x; bounding_Box_Pose_TF(1,1) = min_point_OBB_F.y; bounding_Box_Pose_TF(1,2) = max_point_OBB_F.z; 
        bounding_Box_Pose_TF(2,0) = max_point_OBB_F.x; bounding_Box_Pose_TF(2,1) = min_point_OBB_F.y; bounding_Box_Pose_TF(2,2) = max_point_OBB_F.z; 
        bounding_Box_Pose_TF(3,0) = max_point_OBB_F.x; bounding_Box_Pose_TF(3,1) = min_point_OBB_F.y; bounding_Box_Pose_TF(3,2) = min_point_OBB_F.z; 

        bounding_Box_Pose_TF(4,0) = min_point_OBB_F.x; bounding_Box_Pose_TF(4,1) = max_point_OBB_F.y; bounding_Box_Pose_TF(4,2) = min_point_OBB_F.z; 
        bounding_Box_Pose_TF(5,0) = min_point_OBB_F.x; bounding_Box_Pose_TF(5,1) = max_point_OBB_F.y; bounding_Box_Pose_TF(5,2) = max_point_OBB_F.z;
        bounding_Box_Pose_TF(6,0) = max_point_OBB_F.x; bounding_Box_Pose_TF(6,1) = max_point_OBB_F.y; bounding_Box_Pose_TF(6,2) = max_point_OBB_F.z; 
        bounding_Box_Pose_TF(7,0) = max_point_OBB_F.x; bounding_Box_Pose_TF(7,1) = max_point_OBB_F.y; bounding_Box_Pose_TF(7,2) = min_point_OBB_F.z; 


        // feature_extractor.getOBB (min_point_OBB_F, max_point_OBB_F, position_OBB_F, rotational_matrix_OBB_F);
        // // cout << rotational_matrix_OBB_F << endl;
        // tf::Matrix3x3 R;
        // double roll, pitch, yaw;
        // R.setValue(rotational_matrix_OBB_F(0, 0), rotational_matrix_OBB_F(0, 1), rotational_matrix_OBB_F(0, 2),
        //             rotational_matrix_OBB_F(1, 0), rotational_matrix_OBB_F(1, 1), rotational_matrix_OBB_F(1, 2),
        //             rotational_matrix_OBB_F(2, 0), rotational_matrix_OBB_F(2, 1), rotational_matrix_OBB_F(2, 2));
        // R.getRPY(roll, pitch, yaw);
        // // cout << roll << " " << pitch << " " << yaw << endl;
        // Eigen::Affine3f OBB_transform;
        // OBB_transform = pcl::getTransformation(position_OBB_F.x, position_OBB_F.y, position_OBB_F.z, roll, pitch, yaw);
        // Eigen::Matrix4f OBB_TF = OBB_transform.matrix();

        // pcl::PointXYZI min_point_OBB;
        // pcl::PointXYZI max_point_OBB;

        // Eigen::Matrix<float, 8, 3> bounding_Box_Pose;
        
        // bounding_Box_Pose(0,0) = min_point_OBB_F.x; bounding_Box_Pose(0,1) = min_point_OBB_F.y; bounding_Box_Pose(0,2) = min_point_OBB_F.z; 
        // bounding_Box_Pose(1,0) = min_point_OBB_F.x; bounding_Box_Pose(1,1) = min_point_OBB_F.y; bounding_Box_Pose(1,2) = max_point_OBB_F.z; 
        // bounding_Box_Pose(2,0) = max_point_OBB_F.x; bounding_Box_Pose(2,1) = min_point_OBB_F.y; bounding_Box_Pose(2,2) = max_point_OBB_F.z; 
        // bounding_Box_Pose(3,0) = max_point_OBB_F.x; bounding_Box_Pose(3,1) = min_point_OBB_F.y; bounding_Box_Pose(3,2) = min_point_OBB_F.z; 

        // bounding_Box_Pose(4,0) = min_point_OBB_F.x; bounding_Box_Pose(4,1) = max_point_OBB_F.y; bounding_Box_Pose(4,2) = min_point_OBB_F.z; 
        // bounding_Box_Pose(5,0) = min_point_OBB_F.x; bounding_Box_Pose(5,1) = max_point_OBB_F.y; bounding_Box_Pose(5,2) = max_point_OBB_F.z;
        // bounding_Box_Pose(6,0) = max_point_OBB_F.x; bounding_Box_Pose(6,1) = max_point_OBB_F.y; bounding_Box_Pose(6,2) = max_point_OBB_F.z; 
        // bounding_Box_Pose(7,0) = max_point_OBB_F.x; bounding_Box_Pose(7,1) = max_point_OBB_F.y; bounding_Box_Pose(7,2) = min_point_OBB_F.z; 

        // Eigen::Matrix<float, 8, 3> bounding_Box_Pose_TF;

        // for(int i = 0; i<8; i++){
        //     bounding_Box_Pose_TF(i,0) = OBB_TF(0,0) * bounding_Box_Pose(i,0) + OBB_TF(0,1) * bounding_Box_Pose(i,1) + OBB_TF(0,2) * bounding_Box_Pose(i,2) + OBB_TF(0,3);
        //     bounding_Box_Pose_TF(i,1) = OBB_TF(1,0) * bounding_Box_Pose(i,0) + OBB_TF(1,1) * bounding_Box_Pose(i,1) + OBB_TF(1,2) * bounding_Box_Pose(i,2) + OBB_TF(1,3);
        //     bounding_Box_Pose_TF(i,2) = OBB_TF(2,0) * bounding_Box_Pose(i,0) + OBB_TF(2,1) * bounding_Box_Pose(i,1) + OBB_TF(2,2) * bounding_Box_Pose(i,2) + OBB_TF(2,3);
        // }

        // cout << bounding_Box_Pose << endl;
        geometry_msgs::Point pf;
        geometry_msgs::Point pl;

        for(int i = 0; i<3; i++){
            pf.x = bounding_Box_Pose_TF(i,0);
            pf.y = bounding_Box_Pose_TF(i,1);
            pf.z = bounding_Box_Pose_TF(i,2);

            pl.x = bounding_Box_Pose_TF(i+1,0);
            pl.y = bounding_Box_Pose_TF(i+1,1);
            pl.z = bounding_Box_Pose_TF(i+1,2);

            line_list.points.push_back(pf);
            line_list.points.push_back(pl);  
        }

        pf.x = bounding_Box_Pose_TF(3,0);
        pf.y = bounding_Box_Pose_TF(3,1);
        pf.z = bounding_Box_Pose_TF(3,2);

        pl.x = bounding_Box_Pose_TF(0,0);
        pl.y = bounding_Box_Pose_TF(0,1);
        pl.z = bounding_Box_Pose_TF(0,2);

        line_list.points.push_back(pf);
        line_list.points.push_back(pl);  
        
        for(int i = 4; i<7; i++){
            pf.x = bounding_Box_Pose_TF(i,0);
            pf.y = bounding_Box_Pose_TF(i,1);
            pf.z = bounding_Box_Pose_TF(i,2);

            pl.x = bounding_Box_Pose_TF(i+1,0);
            pl.y = bounding_Box_Pose_TF(i+1,1);
            pl.z = bounding_Box_Pose_TF(i+1,2);

            line_list.points.push_back(pf);
            line_list.points.push_back(pl);  
        }

        pf.x = bounding_Box_Pose_TF(7,0);
        pf.y = bounding_Box_Pose_TF(7,1);
        pf.z = bounding_Box_Pose_TF(7,2);

        pl.x = bounding_Box_Pose_TF(4,0);
        pl.y = bounding_Box_Pose_TF(4,1);
        pl.z = bounding_Box_Pose_TF(4,2);

        line_list.points.push_back(pf);
        line_list.points.push_back(pl);  
        // Rviz Line List

        for(int i = 0; i<4; i++){
            pf.x = bounding_Box_Pose_TF(i,0);
            pf.y = bounding_Box_Pose_TF(i,1);
            pf.z = bounding_Box_Pose_TF(i,2);

            pl.x = bounding_Box_Pose_TF(i+4,0);
            pl.y = bounding_Box_Pose_TF(i+4,1);
            pl.z = bounding_Box_Pose_TF(i+4,2);

            line_list.points.push_back(pf);
            line_list.points.push_back(pl);  
        }
    
    }
};




int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacleseg_node");

    obstacle_env obstacle_filter;

    ROS_INFO("\033[1;32m----> [PHAROS Perception] LiDAR Obstacle Clustering : Initialized\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
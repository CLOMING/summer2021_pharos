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

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <pcl/common/centroid.h>




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

    //TEST
    pcl::PointXYZI query = (0.0, 0.0, 0.0, 0.0);
    ros::Publisher pubNewCluster;


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
        pnh->param<std::string>("pubcluster", pubCluster, "pubcluster");

        subObject = nh.subscribe<sensor_msgs::PointCloud2>(subObstacle, 5, &obstacle_env::clustering_points, this);

        pubObstacle = nh.advertise<sensor_msgs::PointCloud2>(pubCluster,10);
        pubMarker = nh.advertise<visualization_msgs::Marker>("/PHAHROS/perceptopn/LiDAR/objectbox", 10);  // [PHAROS PathPlanner] FIX name for Preliminaries
        
        //TEST
        pubNewCluster = nh.advertise<sensor_msgs::PointCloud2>("/NewMethodCluster", 10); 

    }


    void clustering_points(const sensor_msgs::PointCloud2ConstPtr& total_msg) {

        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_object_1 (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_object (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_object_3 (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_object_4 (new pcl::PointCloud<pcl::PointXYZI> ());
        sensor_msgs::PointCloud2 pubclustermsg;
        pcl::fromROSMsg(*total_msg, *pcl_object_1);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_object_2 (new pcl::PointCloud<pcl::PointXYZI> ());

        // Consider only obstacles nearby
        pcl::PassThrough<pcl::PointXYZI> boxfilter; 
        boxfilter.setInputCloud (pcl_object_1);
        boxfilter.setFilterFieldName ("x");
        boxfilter.setFilterLimits (0, 100.0);
        boxfilter.filter (*pcl_object_3);


        // PCL above ground only
        pcl::PassThrough<pcl::PointXYZI> pass; 
        pass.setInputCloud (pcl_object_3);   
        pass.setFilterFieldName ("z");    
        pass.setFilterLimits(0.2, 3.0); 
        pass.filter(*pcl_object);

        copyPointCloud(*pcl_object, *pcl_object_2);


        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud (pcl_object_2);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance (0.5); // 20cm
        ec.setMinClusterSize (200);
        ec.setMaxClusterSize (3000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (pcl_object_2);
        ec.extract (cluster_indices);

        //std::cout << cluster_indices.size() << std::endl;

        // line_list.action = visualization_msgs::Marker::DELETEALL;
        visualization_msgs::Marker line_list_reset;
        line_list.points = line_list_reset.points;
        line_list.action = visualization_msgs::Marker::ADD;


        static tf::TransformListener listener;

        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_num (new pcl::PointCloud<pcl::PointXYZI> ());

        pcl::PointCloud<pcl::PointXYZI>::Ptr object_clustered (new pcl::PointCloud<pcl::PointXYZI> ());
        int j = 1;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            //pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_num (new pcl::PointCloud<pcl::PointXYZI> ());
            for(std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
                //pcl::PointXYZ pt = pcl_object_2->points[*pit];
                pcl::PointXYZI pt = pcl_object_2->points[*pit];

                pcl::PointXYZI pt2;
                // pcl::PointXYZI minPt, maxPt;
                
                pt2.x=pt.x, pt2.y=pt.y, pt2.z=pt.z;
                pt2.intensity=(float)(j);
                // std::cout<< "intensity:" << j <<std::endl;
                cluster_num->points.push_back(pt2);                
            }

            /*
            // frame transform : veh -> odom
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI>);

            // Get tf matrix
            tf::StampedTransform transform;
            std::string target_frame = "odom";

            try {
                listener.lookupTransform(target_frame, "vehicle_frame", ros::Time(0), transform);
            }
            catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(0.01).sleep();
            }

            pcl_ros::transformPointCloud(*cluster_num, *transformed_cloud, transform);
            (*transformed_cloud).header.frame_id = target_frame;
            (*transformed_cloud).header.stamp = cluster_num->header.stamp;

            bounding_Box(transformed_cloud);
            */

            pcl::PointXYZI minPt, maxPt;
            pcl::getMinMax3D (*cluster_num, minPt, maxPt);
            Eigen::Vector3f diff = minPt.getVector3fMap() - maxPt.getVector3fMap();

            float z_diff = maxPt.z - minPt.z;
            //cout << "Cluster [" << j << "] : " << diff.squaredNorm () << endl;

            if (diff.squaredNorm () < 30 && z_diff>0.4 && maxPt.z>0.4){
                *object_clustered += *cluster_num;
                cout << "Cluster [" << j << "] : " << endl;
                cout << "  Squared Norm Distance : " << diff.squaredNorm() << endl;
                cout << "  z difference : " << z_diff <<endl;
                j=j+10;

                bounding_Box(cluster_num);

                cout << "  Dist to obstacle : " << minPt.x <<endl;

            }
            else{

            }

        }

        pcl::toROSMsg(*object_clustered, pubclustermsg);
        pubclustermsg.header.frame_id = "vehicle_frame";
        pubObstacle.publish(pubclustermsg);
        pubMarker.publish(line_list);



        // TEST
        pcl::CentroidPoint<pcl::PointXYZI> centroid;

        pcl::PointCloud<pcl::PointXYZI>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        int N = 1000; // 차랑 주변 크기
        std::vector<int> idxes;
        std::vector<float> sqr_dists;

        // std::cout << pcl_ob

        kdtree.setInputCloud(pcl_object_2);
        kdtree.nearestKSearch(query, N, idxes, sqr_dists);
        std::cout << "# of cluster found by nearestKSearch : " << idxes.size() << std::endl;
        std::cout << "Query Point" << query.x << query.y << query.z << std::endl;

        if (idxes.size() == 0) {
            std::cout << " No cluster found at (" << query.x << ", " << query.y << ", " << query.z << ")" << std::endl;

            if (cluster_num->size() == 0) {
                std::cout << "Empty KdTree Clustering!!" << std::endl;
            }
            else {
                // Set queary again
                for (int i = 0; i < cluster_num->size(); i++)
                    centroid.add(cluster_num->points[i]);
                centroid.get(query);

                std::cout << "New query point : (" << query.x << ", " << query.y << ", " << query.z << ")" << std::endl;

                // One more trial 
                kdtree.nearestKSearch(query, N, idxes, sqr_dists);

                std::cout << "# of point found at new point :" << idxes.size() << std::endl;
            }
        }

        for (const auto& idx: idxes){
            if (pcl_object_2 -> points[idx].x > 5) {
                boundary->points.push_back(pcl_object_2->points[idx]);
                centroid.add(pcl_object_2->points[idx]);
            }
        }
        

        //pcl::toROSMsg(*object_clustered, pubclustermsg);
        (*boundary).header.frame_id = "vehicle_frame";
        pubNewCluster.publish(boundary);

        centroid.get(query);

        query.z += 0.2;


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


//sources ~/workspace/pharos_ws/devel/setup.bash

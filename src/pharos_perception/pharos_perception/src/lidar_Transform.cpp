#include "utility.h"

using namespace std;

int VLP_L_FrameNum, Ouster_FrameNum, VLP_R_FrameNum, Roof_FrameNum;
std::string sub_Velodyne_Left, sub_Ouster, sub_Velodyne_Right, pub_Velodyne_Left, pub_Ouster,pub_Velodyne_Right;

class LidarTransform {
private:
  ros::NodeHandle nh;
  ros::NodeHandlePtr pnh;

public: 
  ros::Subscriber ros_Sub_Ouster_T;
  ros::Subscriber ros_Sub_Velodyne_L;
  ros::Subscriber ros_Sub_Velodyne_R;

  ros::Publisher ros_pub_Ouster_T;
  ros::Publisher ros_pub_Velodyne_L;
  ros::Publisher ros_pub_Velodyne_R;

  sensor_msgs::PointCloud2 msg_Ouster;
  sensor_msgs::PointCloud2 msg_Velodyne_L;
  sensor_msgs::PointCloud2 msg_Velodyne_R;

  Eigen::Affine3f transLeftToVeh;
  Eigen::Affine3f transRightToVeh;
  Eigen::Affine3f transTopToVeh;

    LidarTransform() {

      pnh = ros::NodeHandlePtr(new ros::NodeHandle("~"));
      
      pnh->param<int>("VLP_L_FrameNum", VLP_L_FrameNum, 0);
      pnh->param<int>("Ouster_FrameNum", Ouster_FrameNum, 0);
      pnh->param<int>("VLP_R_FrameNum", VLP_R_FrameNum, 0);
      pnh->param<int>("Roof_FrameNum", Roof_FrameNum, 0);
      
      pnh->param<std::string>("sub_Ouster", sub_Ouster, "sub_Ouster");
      pnh->param<std::string>("sub_Velodyne_Left", sub_Velodyne_Left, "sub_Velodyne_Left");
      pnh->param<std::string>("sub_Velodyne_Right", sub_Velodyne_Right, "sub_Velodyne_Right");

      pnh->param<std::string>("pub_Ouster", pub_Ouster, "pub_Ouster");
      pnh->param<std::string>("pub_Velodyne_Left", pub_Velodyne_Left, "pub_Velodyne_Left");
      pnh->param<std::string>("pub_Velodyne_Right", pub_Velodyne_Right, "pub_Velodyne_Right");      
      
      Eigen::Affine3f Roof_Vehicle;
      Eigen::Affine3f VLP_L_Roof;
      Eigen::Affine3f VLP_R_Roof;
      Eigen::Affine3f OS_Roof;

      getTransformationMatrix(Roof_FrameNum, &Roof_Vehicle);
      getTransformationMatrix(VLP_L_FrameNum, &VLP_L_Roof);
      getTransformationMatrix(VLP_R_FrameNum, &VLP_R_Roof);
      getTransformationMatrix(Ouster_FrameNum, &OS_Roof);

      transLeftToVeh = Roof_Vehicle * VLP_L_Roof;
      transTopToVeh = Roof_Vehicle * OS_Roof;
      transRightToVeh = Roof_Vehicle * VLP_R_Roof;
      
      ros_Sub_Velodyne_L = nh.subscribe<sensor_msgs::PointCloud2>(sub_Velodyne_Left, 5, &LidarTransform::leftTransformCB, this, ros::TransportHints().tcpNoDelay());
      ros_Sub_Velodyne_R = nh.subscribe<sensor_msgs::PointCloud2>(sub_Velodyne_Right, 5, &LidarTransform::rightTransformCB, this, ros::TransportHints().tcpNoDelay());
      ros_Sub_Ouster_T = nh.subscribe<sensor_msgs::PointCloud2>(sub_Ouster, 5, &LidarTransform::topTransformCB, this, ros::TransportHints().tcpNoDelay());

      ros_pub_Ouster_T = nh.advertise<sensor_msgs::PointCloud2> (pub_Ouster, 10);
      ros_pub_Velodyne_L = nh.advertise<sensor_msgs::PointCloud2> (pub_Velodyne_Left, 10);
      ros_pub_Velodyne_R = nh.advertise<sensor_msgs::PointCloud2> (pub_Velodyne_Right, 10);

    }

    void getTransformationMatrix(int tfNum, Eigen::Affine3f* matPtr) {

      // [PHAROS Peception] : TF data Parsing
      ros::NodeHandle tfNode("~");
      std::string tf_name = "tf_" + to_string(tfNum);
      float x, y, z, roll, pitch, yaw;

      tfNode.getParam(tf_name+"/x", x);
      tfNode.getParam(tf_name+"/y", y);
      tfNode.getParam(tf_name+"/z", z);
      tfNode.getParam(tf_name+"/roll", roll);
      tfNode.getParam(tf_name+"/pitch", pitch);
      tfNode.getParam(tf_name+"/yaw", yaw);

      *matPtr = pcl::getTransformation(x, y, z, roll*M_PI/180, pitch*M_PI/180, yaw*M_PI/180);
    }

    void leftTransformCB(const sensor_msgs::PointCloud2ConstPtr& leftLaserMsg)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr pclLeft (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::fromROSMsg(*leftLaserMsg, *pclLeft);

      // Transformation
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::transformPointCloud (*pclLeft, *transformed_cloud, transLeftToVeh.matrix());

      pcl::toROSMsg(*transformed_cloud, msg_Velodyne_L);
      msg_Velodyne_L.header.frame_id = "vehicle_frame";
      msg_Velodyne_L.header.stamp = leftLaserMsg->header.stamp;
      ros_pub_Velodyne_L.publish(msg_Velodyne_L);
    }

    void topTransformCB(const sensor_msgs::PointCloud2ConstPtr& topLaserMsg)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr pclTop (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::PointCloud<pcl::PointXYZI>::Ptr pclTop_filtered (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::fromROSMsg(*topLaserMsg, *pclTop_filtered);

      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::transformPointCloud (*pclTop_filtered, *transformed_cloud, transTopToVeh.matrix());
      pcl::toROSMsg(*transformed_cloud, msg_Ouster);

      msg_Ouster.header.frame_id = "vehicle_frame";
      msg_Ouster.header.stamp = topLaserMsg->header.stamp;
      ros_pub_Ouster_T.publish(msg_Ouster);
    }

    void rightTransformCB(const sensor_msgs::PointCloud2ConstPtr& rightLaserMsg)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr pclRight (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::fromROSMsg(*rightLaserMsg, *pclRight);

      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::transformPointCloud (*pclRight, *transformed_cloud, transRightToVeh.matrix());
      pcl::toROSMsg(*transformed_cloud, msg_Velodyne_R);

      msg_Velodyne_R.header.frame_id = "vehicle_frame";
      msg_Velodyne_R.header.stamp = rightLaserMsg->header.stamp;
      ros_pub_Velodyne_R.publish(msg_Velodyne_R);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_node");

    LidarTransform LT;

    ROS_INFO("\033[1;32m----> [PHAROS Perception] PointCloud Transformer : Initialized\033[0m");

    ros::MultiThreadedSpinner spinner(10);
    spinner.spin();

    return 0;
}

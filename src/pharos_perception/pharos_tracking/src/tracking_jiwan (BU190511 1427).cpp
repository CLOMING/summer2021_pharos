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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>

#define R 1
#define raduisThreshold 1


using namespace std;
nav_msgs::Odometry odom;

struct Point
{
    double x, y;
};

struct Velocity
{
    double x, y;
};

struct MarkerPub
{
    ros::Publisher Text;
    ros::Publisher Velocity;
    ros::Publisher TextArray;
    ros::Publisher VelocityArray;
};

struct AdditinoalInfo
{
    double distanceMin;
    unsigned int matchingIndex;
    unsigned int distancePriority;
    pharos_msgs::ObjectPose pose;
    pharos_msgs::ObjectSize size;
    pharos_msgs::ObjectCollision collision;
    unsigned int id;
    unsigned int maintain_count;
    unsigned int type;
    Point velocity;
    double speed;
    double theta;
    unsigned int theta_count;
    unsigned int vehicle_state;
};

//struct ObjInfoArray
//{
//    AdditinoalInfoArray newO;
//    AdditinoalInfoArray oldO;
//    AdditinoalInfoArray expectO;
//};



class Tracking
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Subscriber classifiedLidarMsgSub;
    ros::Subscriber odomSub;

    ros::Publisher pointPub;

    MarkerPub markerPub;

    visualization_msgs::MarkerArray textMarkerArray;
    visualization_msgs::MarkerArray velocityMarkerArray;

    std::list<int> idCenter;
    std::list<int>::iterator iter;

    std::string odomTopic;
    std::string classifiedLidarMsgTopic;

    std::vector<AdditinoalInfo> newInfoArray;
    std::vector<AdditinoalInfo> oldInfoArray;

    double curr_t;
    double past_t;
    double dt;

    bool isInit;

public:
    Tracking() : pnh("~"), isInit(true)
    {
        this->pnh.param<std::string>("odomTopic", this->odomTopic, "/odom/vehicle");
        this->pnh.param<std::string>("classifiedLidarMsgTopic", this->classifiedLidarMsgTopic, "/pharos_transformed_msg");

        this->odomSub = nh.subscribe(odomTopic, 10, &Tracking::odomCB, this);
        this->classifiedLidarMsgSub = nh.subscribe(classifiedLidarMsgTopic, 10, &Tracking::trackingCB, this);

        markerPub.Text = nh.advertise<visualization_msgs::Marker>("/trackingTextMark", 10);
        markerPub.TextArray = nh.advertise<visualization_msgs::MarkerArray>("/trackingTextMarkArray", 10);
        markerPub.VelocityArray = nh.advertise<visualization_msgs::MarkerArray>("/trackingVelocityMarkArray", 10);

        pointPub = nh.advertise<sensor_msgs::PointCloud2>("fan_shape_points", 10);

    }

    void odomCB(const nav_msgs::OdometryPtr &msg)
    {
        odom = *msg;
    }

    void trackingCB(const pharos_msgs::ObjectInfoArrayPtr &msg)
    {
        textMarkerArray.markers.clear();
        velocityMarkerArray.markers.clear();
        newInfoArray.clear();

        if(msg->objects[0].pose.x == 10000) // lidar 값이 없을 때,
        {
            WriteInfoRviz();
            return;
        }


        for (int k = 0; k < msg->objects.size(); ++k)
        {
            AdditinoalInfo newInfo = {0};
            newInfo.size = msg->objects[k].size;
            newInfo.pose = msg->objects[k].pose;
            newInfo.type = msg->objects[k].type;
            newInfo.collision = msg->objects[k].collision;
            newInfoArray.push_back(newInfo);
        }


        curr_t = ros::Time::now().toSec();

        if(isInit)
        {
            for (int i = 0; i < newInfoArray.size(); ++i)
            {
                newInfoArray[i].id = visitIdCenterToIssue();
            }
            oldInfoArray = newInfoArray;
            past_t = curr_t;
            isInit = false;
            return;
        }

        dt = curr_t - past_t;
        for (int j = 0; j < oldInfoArray.size(); ++j)
        {
            oldInfoArray[j].pose.expected_x = oldInfoArray[j].pose.x + oldInfoArray[j].velocity.x * dt;
            oldInfoArray[j].pose.expected_y = oldInfoArray[j].pose.y + oldInfoArray[j].velocity.y * dt;
        }

        if(oldInfoArray.size() <= newInfoArray.size())
        {
            Point oldExpect;
            Point newPose;
            double distanceMin(1000000);
            unsigned matchingIndex;
            for (int i = 0; i < oldInfoArray.size(); ++i)
            {
                oldExpect.x = oldInfoArray[i].pose.expected_x;
                oldExpect.y = oldInfoArray[i].pose.expected_y;

                for (int j = 0; j < newInfoArray.size(); ++j)
                {
                    newPose.x = newInfoArray[j].pose.x;
                    newPose.y = newInfoArray[j].pose.y;
                    double distance = Distance(oldExpect, newPose);
                    if(distance < distanceMin)
                    {
                        distanceMin = distance;
                        matchingIndex = j;
                    }
                }

                oldInfoArray[i].distanceMin = distanceMin;
                oldInfoArray[i].matchingIndex = matchingIndex;
                newInfoArray[matchingIndex].id = oldInfoArray[i].id;
            }

//            for (int k = 0; k < oldInfoArray.size(); ++k)
//            {
//                for (int i = k; i < oldInfoArray.size() - 1; ++i)
//                {
//                    if(oldInfoArray[i].distanceMin > oldInfoArray[i+1].distanceMin)
//                    {
//                        oldInfoArray[i+1].distancePriority = k + 1; // 등수 매기기 1부터 시작
//                        oldInfoArray[]
//                    }
//                }
//            }
//
//            for (int l = 0; l < oldInfoArray.size(); ++l)
//            {
//                cout << "hello" << endl;
//                cout << oldInfoArray[l].distanceMin << endl;
//            }



            for (int j = 0; j < newInfoArray.size(); ++j)
            {
                if(newInfoArray[j].id == 0)
                {
                    newInfoArray[j].id = visitIdCenterToIssue();
                }
            }

        }

//        for (iter = idCenter.begin(); iter != idCenter.end(); ++iter)
//        {
//            cout << *iter << endl;
//        }
//        cout << endl;

        double distance;

        pcl::PointCloud<pcl::PointXYZI>::Ptr drawingPoints(new pcl::PointCloud<pcl::PointXYZI>);

        DrawPoint(newInfoArray, drawingPoints, 30);


        MarkObjectRviz(&newInfoArray, 0);
        MarkObjectRviz(&oldInfoArray, 1);
        WriteInfoRviz();

        sensor_msgs::PointCloud2Ptr sensor_pointcloud_test(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*drawingPoints, *sensor_pointcloud_test);
        sensor_pointcloud_test->header.frame_id = odom.header.frame_id;
        sensor_pointcloud_test->header.stamp = ros::Time::now();

        pointPub.publish(*sensor_pointcloud_test);
        markerPub.TextArray.publish(textMarkerArray);
        markerPub.VelocityArray.publish(velocityMarkerArray);

        past_t = curr_t;
        oldInfoArray = newInfoArray;

    }

    double Distance(const Point &p1, const Point &p2)
    {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

    void DrawPoint(const std::vector<AdditinoalInfo>& objectInfoArray, pcl::PointCloud<pcl::PointXYZI>::Ptr& drawingPoints,
                        double yawAngleChange)
    {
        yawAngleChange *= M_PI/180;
        Eigen::Matrix2f Rotating;
        Rotating << cos(yawAngleChange), -sin(yawAngleChange),
                    sin(yawAngleChange), cos(yawAngleChange);
        for (int k = 0; k < objectInfoArray.size(); ++k)
        {
            pcl::PointXYZI pt;
            for (int j = 0; j < 11; ++j)
            {
                pt.x = objectInfoArray[k].pose.x - 0.5 + 0.2*j;

                for (int i = 0; i < 11; ++i)
                {
                    auto temp = pt.x;
                    pt.y = objectInfoArray[k].pose.y - 0.5 + 0.1*i;
                    pt.z = odom.pose.pose.position.z;

                    pt.x -= objectInfoArray[k].pose.x;
                    pt.y -= objectInfoArray[k].pose.y;
                    Eigen::Vector2f location(pt.x, pt.y);
                    location = Rotating * location;
                    pt.x = location.x();
                    pt.y = location.y();


                    pt.x += objectInfoArray[k].pose.x;
                    pt.y += objectInfoArray[k].pose.y;

                    drawingPoints->push_back(pt);
                    pt.x = temp;
                }

            }

        }
    }


    void MarkObjectRviz(const std::vector<AdditinoalInfo>* objectInfoArray, bool isOld)
    {
        float duration = dt;
        for (int i = 0; i < objectInfoArray->size(); ++i)
        {
            if(isOld)
            {
                visualization_msgs::Marker marker;
                std::stringstream text;
                marker.header.frame_id = "odom";
                marker.header.stamp = ros::Time::now();
//            marker.ns = ""std::to_string(i);
                marker.ns = "hello";
                marker.id = objectInfoArray->at(i).id;
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text << "class type : " << objectInfoArray->at(i).type << endl << "id : " << marker.id << endl;
                marker.text = text.str();
                marker.action = visualization_msgs::Marker::MODIFY;
                marker.pose.position.x = objectInfoArray->at(i).pose.x;
                marker.pose.position.y = objectInfoArray->at(i).pose.y;
                marker.pose.position.z = objectInfoArray->at(i).pose.z;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 5;
                marker.scale.y = 1;
                marker.scale.z = 1;
                marker.color.a = 1.0;
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 0.0;
                marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
                marker.lifetime = ros::Duration(duration);

                textMarkerArray.markers.push_back(marker);
            }
            else
            {
                visualization_msgs::Marker marker;
                std::stringstream text;
                marker.header.frame_id = "odom";
                marker.header.stamp = ros::Time::now();
//            marker.ns = ""std::to_string(i);
                marker.ns = "hello";
                marker.id = objectInfoArray->at(i).id;
                marker.type = visualization_msgs::Marker::CUBE;
//                text << "class type : " << objectInfoArray[i].type << endl << "id : " << marker.id << endl;
                marker.text = text.str();
                marker.action = visualization_msgs::Marker::MODIFY;
                marker.pose.position.x = objectInfoArray->at(i).pose.x;
                marker.pose.position.y = objectInfoArray->at(i).pose.y;
                marker.pose.position.z = (objectInfoArray->at(i).pose.z)/2;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.3;
                marker.scale.y = 0.3;
                marker.scale.z = 0.5;
                marker.color.a = 1.0;
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 0.0;
                marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
                marker.lifetime = ros::Duration(duration);

                textMarkerArray.markers.push_back(marker);
            }

        }

        if(isOld)
        {
            for (int i = 0; i < objectInfoArray->size(); ++i)
            {

                visualization_msgs::Marker ArrowMarker;
                std::stringstream text;
                ArrowMarker.header.frame_id = "odom";
                ArrowMarker.header.stamp = ros::Time::now();
//            ArrowMarker.ns = ""std::to_string(i);
                ArrowMarker.ns = "hello";
                ArrowMarker.id = objectInfoArray->at(i).id;
                ArrowMarker.type = visualization_msgs::Marker::ARROW;
                text << "class type : " << objectInfoArray->at(i).type << endl << "id : " << ArrowMarker.id << endl;
                ArrowMarker.text = text.str();
                ArrowMarker.action = visualization_msgs::Marker::MODIFY;
                ArrowMarker.pose.position.x = objectInfoArray->at(i).pose.x;
                ArrowMarker.pose.position.y = objectInfoArray->at(i).pose.y;
                ArrowMarker.pose.position.z = objectInfoArray->at(i).pose.z;
                tf2::Quaternion myQ;
                myQ.setEuler(0,0,(30+ArrowMarker.id) * M_PI/180);
                ArrowMarker.pose.orientation.x = myQ.x();
                ArrowMarker.pose.orientation.y = myQ.y();
                ArrowMarker.pose.orientation.z = myQ.z();
                ArrowMarker.pose.orientation.w = myQ.w();
                ArrowMarker.scale.x = 5;
                ArrowMarker.scale.y = 0.25;
                ArrowMarker.scale.z = 0.25;
                ArrowMarker.color.a = 1.0;
                ArrowMarker.color.r = 1;
                ArrowMarker.color.g = 1;
                ArrowMarker.color.b = 1.0;
                ArrowMarker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
                ArrowMarker.lifetime = ros::Duration(duration);

                velocityMarkerArray.markers.push_back(ArrowMarker);

            }
        }

    }

    void WriteInfoRviz()
    {
        float duration = 0.2;
        visualization_msgs::Marker marker;
        std::stringstream text;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
//            marker.ns = ""std::to_string(i);
        marker.ns = "hello";
        marker.id = 100000;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text << "oldInfoArray.size() : " << oldInfoArray.size();
        marker.text = text.str();
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.pose.position.x = odom.pose.pose.position.x + 50;
        marker.pose.position.y = odom.pose.pose.position.y + 50;
        marker.pose.position.z = odom.pose.pose.position.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 15;
        marker.scale.y = 3;
        marker.scale.z = 3;
        marker.color.a = 1.0;
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 1.0;
        marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        marker.lifetime = ros::Duration(duration);
        markerPub.Text.publish(marker);
    }


    unsigned int visitIdCenterToIssue()
    {
        bool isFull;
        int id(1);
        if(idCenter.empty())
        {
            idCenter.push_back(id);
            return id;
        }
        else
        {
            for (iter = idCenter.begin(); iter != idCenter.end(); ++iter) // 양방향 접근 반복자임. std::list 공부해야 함.
            {
                if(*iter != id) // 예를 들어, 3번에서 5번으로 되었을 때, 4번을 할당해주면 되겠지~
                {
                    idCenter.push_back(id);
                    isFull = false;
                    break;
                }
                isFull = true;
                ++id;
            }
            idCenter.sort(); // 순서 섞였으니 sort 해줘야겠지
            if(isFull)
            {
                idCenter.push_back(id); // 가득 찼을 경우에는 뒤에다 넣으면 되겠지~
            }
            return id;
        }
    }

    void visitIdCenterToRemove(const unsigned int& id)
    {
        idCenter.remove(id);
    }

};

int main(int argc, char **argv)          //노드 메인함수 char **이다 계속 int하는데 주의할것
{
    ros::init(argc, argv, "tracking_jiwan");       //노드명 초기화

    ROS_INFO("started filter node");
    Tracking tracking;
    ros::spin();
    return 0;
}

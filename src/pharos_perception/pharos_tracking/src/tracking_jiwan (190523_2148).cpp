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
#define personThreshold 1
#define elseThreshold 2
#define carThreshold 3
#define rectangular 1
#define circular 2
#define filterNum 10



using namespace std;
nav_msgs::Odometry odom;

struct Point
{
    double x, y;
};

struct Velocity
{
    double x, y, r, theta;
};

struct MarkerPub
{
    ros::Publisher Text;
    ros::Publisher Velocity;
    ros::Publisher TextArray;
    ros::Publisher VelocityArray;
};


struct forDistancePriority
{
    double distanceMin;
    unsigned int shortestDistanceIndex_ToPassID;
};

struct AdditinoalInfo
{
    double distanceMin;
    double distanceThreshold;
    unsigned int matchingIndex;
    unsigned int distancePriority;
    bool isSimilarity;
    bool isIdPassComplete;
    pharos_msgs::ObjectPose pose;
    pharos_msgs::ObjectSize size;
    pharos_msgs::ObjectSize initialSize;
    double shadowArea;
    pharos_msgs::ObjectCollision collision;
    unsigned int id;
    unsigned int maintain_count;
    unsigned int type;
//    unsigned int fixedType;
    std::deque<Velocity> velocityDeque;
    Velocity velocity;
    double speed;
    double theta;
    unsigned int theta_count;
    unsigned int vehicle_state;
};

//struct ObjInfoArray
//{isRight
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
    ros::Publisher trackedInfoPub;

    MarkerPub markerPub;

    visualization_msgs::MarkerArray textMarkerArray;
    visualization_msgs::MarkerArray velocityMarkerArray;

    std::list<int> idCenter;
    std::list<int>::iterator iter;

    std::string odomTopic;
    std::string classifiedLidarMsgTopic;

    std::vector<AdditinoalInfo> newInfoArray;
    std::vector<AdditinoalInfo> oldInfoArray;

    std::vector<AdditinoalInfo>::iterator vec_iter_AI;
    std::vector<AdditinoalInfo>::reverse_iterator vec_riter_AI;

    std::vector<forDistancePriority> initalDataArray;
    std::vector<unsigned int> checkOverlapIndex;

    pharos_msgs::ObjectInfoArray trackedInfoArray;

    double distanceThreshold;

    double curr_t;
    double past_t;
    double dt;

    bool isInit;
    bool isRight;

public:
    Tracking() : pnh("~"), isInit(true)
    {
        this->pnh.param<std::string>("odomTopic", this->odomTopic, "/odom/vehicle");
        this->pnh.param<std::string>("classifiedLidarMsgTopic", this->classifiedLidarMsgTopic, "/pharos_transformed_msg");
        this->pnh.param<bool>("isRight", isRight, true);

        this->odomSub = nh.subscribe(odomTopic, 10, &Tracking::odomCB, this);
        this->classifiedLidarMsgSub = nh.subscribe(classifiedLidarMsgTopic, 10, &Tracking::trackingCB, this);

        markerPub.Text = nh.advertise<visualization_msgs::Marker>("/trackingTextMark", 10);
        markerPub.TextArray = nh.advertise<visualization_msgs::MarkerArray>("/trackingTextMarkArray", 10);
        markerPub.VelocityArray = nh.advertise<visualization_msgs::MarkerArray>("/trackingVelocityMarkArray", 10);

        pointPub = nh.advertise<sensor_msgs::PointCloud2>("fan_shape_points", 10);

        trackedInfoPub = nh.advertise<pharos_msgs::ObjectInfoArray>("/pharos_tracked2_msg",10);

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
//        for (int l = 0; l < oldInfoArray.size(); ++l)
//        {
//            oldInfoArray[l].isIdPassComplete = false;
//        }

//      값이 없을 때 idCenter 고장남.
        if(msg->objects[0].pose.x == 10000) // lidar 값이 없을 때,
        {
            WriteInfoRviz();
            oldInfoArray.clear();
            idCenter.clear();
            return;
        }

        for (int k = 0; k < msg->objects.size(); ++k)
        {
            AdditinoalInfo newInfo = {0};
            newInfo.size = msg->objects[k].size;
            newInfo.pose = msg->objects[k].pose;
            newInfo.type = msg->objects[k].type;
//            if(newInfo.type != 0 && newInfo.fixedType == 0)
//            {
//                newInfo.fixedType = newInfo.type;
//            }
            newInfo.collision = msg->objects[k].collision;
            newInfo.shadowArea = (newInfo.size.max_x - newInfo.size.min_x) * (newInfo.size.max_y - newInfo.size.min_y);
            newInfoArray.push_back(newInfo);
        }


        unsigned int initOldInfoSize(oldInfoArray.size());
        unsigned int initNewInfoSize(newInfoArray.size()); // for debugging print



        if(isInit)
        {
            for (int i = 0; i < newInfoArray.size(); ++i)
            {
                newInfoArray[i].id = visitIdCenterToIssue();
            }
            oldInfoArray = newInfoArray;
            curr_t = ros::Time::now().toSec();
            past_t = curr_t;
            isInit = false;
            return;
        }


        distanceThreshold = 0;
        for (int m = 0; m < newInfoArray.size(); ++m)
        {
            double xGain;
            double yGain;
            if(0 <= newInfoArray[m].shadowArea && newInfoArray[m].shadowArea <= 1)
            {
                xGain = 0.25*(newInfoArray[m].size.max_x - newInfoArray[m].size.min_x);
                yGain = 0.25*(newInfoArray[m].size.max_y - newInfoArray[m].size.min_y);
                distanceThreshold = personThreshold;
                newInfoArray[m].distanceThreshold = personThreshold;
            }
            else if(newInfoArray[m].shadowArea >= 8)
            {
                xGain = -0.25*(newInfoArray[m].size.max_x - newInfoArray[m].size.min_x);
                yGain = -0.25*(newInfoArray[m].size.max_y - newInfoArray[m].size.min_y);
                distanceThreshold = carThreshold;
                newInfoArray[m].distanceThreshold = carThreshold;
            }

            else
            {
                xGain = 0;
                yGain = 0;
                distanceThreshold = elseThreshold;
                newInfoArray[m].distanceThreshold = elseThreshold;
            }

            for (int i = 0; i < newInfoArray.size(); ++i)
            {
                if(m != i && !newInfoArray[m].isSimilarity)
                {
                    bool isX_inBox = (newInfoArray[m].size.min_x - xGain) <= newInfoArray[i].pose.x && newInfoArray[i].pose.x <= (newInfoArray[m].size.max_x + xGain);
                    bool isY_inBox = (newInfoArray[m].size.min_y - yGain) <= newInfoArray[i].pose.y && newInfoArray[i].pose.y <= (newInfoArray[m].size.max_y + yGain);
//                    bool isZ_inBox = (newInfoArray[m].size.min_z - gain) <= newInfoArray[i].pose.z && newInfoArray[i].pose.z <= (newInfoArray[m].size.max_z + gain);

                    if(isX_inBox && isY_inBox)
                    {
                        newInfoArray[i].isSimilarity = true; // 일단 아무거나 하나 지운건데 다음에는 거리가 먼 애를 지워보자(X) 작은 애를 지우자
                    }
                }
            }
        }


        vec_iter_AI = newInfoArray.end();
        --vec_iter_AI; // end()는 마지막요소가 아닌 벡터의 끝을 가리킴
        for (int n = newInfoArray.size() - 1; n >= 0; --n)
        {
            if(newInfoArray[n].isSimilarity)
            {
                newInfoArray.erase(vec_iter_AI);
            }
            --vec_iter_AI;
        }

        curr_t = ros::Time::now().toSec();
        dt = curr_t - past_t;
        for (int j = 0; j < oldInfoArray.size(); ++j)
        {
            oldInfoArray[j].pose.expected_x = oldInfoArray[j].pose.x + oldInfoArray[j].velocity.x * dt;
            oldInfoArray[j].pose.expected_y = oldInfoArray[j].pose.y + oldInfoArray[j].velocity.y * dt;
        }
        past_t = curr_t;
        /*
         * 여기에 없지만
         * 속도는 방법 :
         * id 매칭 후 바로 속도 대입
         */


        initalDataArray.clear();
        checkOverlapIndex.clear();

        for (int i = oldInfoArray.size() - 1; i >= 0; --i)
        {
            Point oldExpect;
            Point newPose;
            bool isOverlab(false);
            unsigned int shortestDistanceIndex_ToPassID;
            oldExpect.x = oldInfoArray[i].pose.expected_x;
            oldExpect.y = oldInfoArray[i].pose.expected_y;
            double distanceMin(1000000);
            for (int j = 0; j < newInfoArray.size(); ++j)
            {
                newPose.x = newInfoArray[j].pose.x;
                newPose.y = newInfoArray[j].pose.y;
                double distance = Distance(oldExpect, newPose);
                if(distance < distanceMin)
                {
                    distanceMin = distance; // 둘다 가까울 때 겹침.
                    shortestDistanceIndex_ToPassID = j;
                }
            }
            oldInfoArray[i].distanceMin = distanceMin;
            oldInfoArray[i].matchingIndex = shortestDistanceIndex_ToPassID;
            for (int k = 0; k < checkOverlapIndex.size(); ++k)
            {
                if(checkOverlapIndex[k] == shortestDistanceIndex_ToPassID)
                {
                    isOverlab = true;
                    break;
                }
            }
            if(distanceMin <= newInfoArray[shortestDistanceIndex_ToPassID].distanceThreshold && !isOverlab) // 일단 거리순으로 먼저 배정하는 것이 아니라 만족하면 배정하는 중. 나중에 거리순으로 바꿔야 함.
            {
                newInfoArray[shortestDistanceIndex_ToPassID].id = oldInfoArray[i].id;
                if(oldInfoArray[i].type != 0 && newInfoArray[shortestDistanceIndex_ToPassID].type == 0)
                    newInfoArray[shortestDistanceIndex_ToPassID].type = oldInfoArray[i].type;

                newInfoArray[shortestDistanceIndex_ToPassID].velocity.x
                        = (newInfoArray[shortestDistanceIndex_ToPassID].pose.x - oldInfoArray[i].pose.x);
                newInfoArray[shortestDistanceIndex_ToPassID].velocity.y
                        = (newInfoArray[shortestDistanceIndex_ToPassID].pose.y - oldInfoArray[i].pose.y);
                newInfoArray[shortestDistanceIndex_ToPassID].velocity.r
                        = sqrt(pow(newInfoArray[shortestDistanceIndex_ToPassID].velocity.x, 2) + pow(newInfoArray[shortestDistanceIndex_ToPassID].velocity.y, 2));
                newInfoArray[shortestDistanceIndex_ToPassID].velocity.theta
                        = atan(newInfoArray[shortestDistanceIndex_ToPassID].velocity.y / newInfoArray[shortestDistanceIndex_ToPassID].velocity.x);

                if(newInfoArray[shortestDistanceIndex_ToPassID].velocityDeque.empty())
                {

                }
                oldInfoArray[i].isIdPassComplete = true;
                checkOverlapIndex.push_back(shortestDistanceIndex_ToPassID);
            }
        }



//        for (int l = 0; l < oldInfoArray.size(); ++l)
//        {
//            vec_iter_AI = oldInfoArray.end();
//            --vec_iter_AI;
//            for (int i = oldInfoArray.size() - 1; i >= 0; --i)
//            {
//                if(initalDataArray[l].shortestDistanceIndex_ToPassID == initalDataArray[i].shortestDistanceIndex_ToPassID)
//                {
//                    if(initalDataArray[i].distanceMin > initalDataArray[l].distanceMin)
//                    {
//                        oldInfoArray.erase(vec_iter_AI);
//                    }
//                    --vec_iter_AI;
//                }
//            }
//        }

        for (int j = 0; j < newInfoArray.size(); ++j)
        {
            if(newInfoArray[j].id == 0)
            {
                newInfoArray[j].id = visitIdCenterToIssue();
            }
        }

        for (int i = oldInfoArray.size() - 1; i >= 0; --i)
        {
            if(oldInfoArray[i].isIdPassComplete == false)
            {
                visitIdCenterToRemove(oldInfoArray, oldInfoArray[i].id);
                // 여기다가 벡터 안의 id에 해당 하는 값도 지워야지
            }
        }

        DebuggingPrint(initOldInfoSize, initNewInfoSize);


        pcl::PointCloud<pcl::PointXYZI>::Ptr drawingPoints(new pcl::PointCloud<pcl::PointXYZI>);

//        DrawPoint(newInfoArray, drawingPoints, 30, rectangular);
        DrawPoint(oldInfoArray, drawingPoints, 0, circular);

        MarkObjectRviz(&newInfoArray, 0);
        MarkObjectRviz(&oldInfoArray, 1);
        WriteInfoRviz();

        sensor_msgs::PointCloud2Ptr sensor_pointcloud_test(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*drawingPoints, *sensor_pointcloud_test);
        sensor_pointcloud_test->header.frame_id = odom.header.frame_id;
        sensor_pointcloud_test->header.stamp = ros::Time::now();

        trackedInfoArray.objects.clear();
        trackedInfoArray.header.stamp = msg->header.stamp;
        for (int l = 0; l < newInfoArray.size(); ++l)
        {
            pharos_msgs::ObjectInfo trackedInfo;
            trackedInfo.pose = newInfoArray[l].pose;
            trackedInfo.size = newInfoArray[l].size;
            trackedInfo.collision = newInfoArray[l].collision;
            trackedInfo.id = newInfoArray[l].id;
            trackedInfo.type = newInfoArray[l].type;

            trackedInfoArray.objects.push_back(trackedInfo);
        }

        pointPub.publish(*sensor_pointcloud_test);
        markerPub.TextArray.publish(textMarkerArray);
        markerPub.VelocityArray.publish(velocityMarkerArray);
        trackedInfoPub.publish(trackedInfoArray);


        oldInfoArray.clear();
        oldInfoArray = newInfoArray;

    }

    double Distance(const Point &p1, const Point &p2)
    {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

    void DrawPoint(const std::vector<AdditinoalInfo>& objectInfoArray, pcl::PointCloud<pcl::PointXYZI>::Ptr& drawingPoints,
                        double yawAngleChange, unsigned int drawMode)
    {
        if(drawMode == rectangular)
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
                        pt.x = temp; // x 값이 바뀌어서 다시 원래대로 해줌
                    }

                }

            }
        }
        else if(drawMode == circular)
        {
            Eigen::Matrix2f Rotating;
            int pointNum(200);
            for (int k = 0; k < objectInfoArray.size(); ++k)
            {
                yawAngleChange = 0;
                yawAngleChange *= M_PI/180;
                for (int i = 0; i < pointNum; ++i)
                {
                    pcl::PointXYZI pt = {0};
                    pt.x = objectInfoArray[k].distanceThreshold;
                    Rotating << cos(yawAngleChange), -sin(yawAngleChange),
                               sin(yawAngleChange), cos(yawAngleChange);
                    Eigen::Vector2f location(pt.x, pt.y);
                    location = Rotating * location;
                    pt.x = location.x();
                    pt.y = location.y();

                    pt.x += objectInfoArray[k].pose.x;
                    pt.y += objectInfoArray[k].pose.y;

                    pt.z = odom.pose.pose.position.z;
//                    pt.intensity =
                    drawingPoints->push_back(pt);

                    yawAngleChange += 2*M_PI / pointNum;
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
                text << "i : " << i << endl;
                marker.text = text.str();
                marker.action = visualization_msgs::Marker::MODIFY;
                marker.pose.position.x = objectInfoArray->at(i).pose.x;
                marker.pose.position.y = objectInfoArray->at(i).pose.y;
                marker.pose.position.z = objectInfoArray->at(i).pose.z;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 2.5;
                marker.scale.y = 0.5;
                marker.scale.z = 0.5;
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
                marker.id = objectInfoArray->at(i).id+100;
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
                myQ.setEuler(0,0,objectInfoArray->at(i).velocity.theta);
                ArrowMarker.pose.orientation.x = myQ.x();
                ArrowMarker.pose.orientation.y = myQ.y();
                ArrowMarker.pose.orientation.z = myQ.z();
                ArrowMarker.pose.orientation.w = myQ.w();
                ArrowMarker.scale.x = objectInfoArray->at(i).velocity.r;
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
        text << "oldInfoArray.size() : " << oldInfoArray.size() << endl;
        text << "newInfoArray.size() : " << newInfoArray.size();
        marker.text = text.str();
        marker.action = visualization_msgs::Marker::MODIFY;

        Eigen::Vector2f point;
        point.x() = 30;
        point.y() = 0;

        Eigen::Quaternionf q;
        q.x() = odom.pose.pose.orientation.x;
        q.y() = odom.pose.pose.orientation.y;
        q.z() = odom.pose.pose.orientation.z;
        q.w() = odom.pose.pose.orientation.w;
        auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

        Eigen::Matrix2f Rotating;
        Rotating << cos(euler[2]), -sin(euler[2]),
                    sin(euler[2]), cos(euler[2]);

        point = Rotating * point;

        point.x() += odom.pose.pose.position.x;
        point.y() += odom.pose.pose.position.y;

        marker.pose.position.x = point.x();
        marker.pose.position.y = point.y();
        marker.pose.position.z = odom.pose.pose.position.z + 10;
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

    void visitIdCenterToRemove(std::vector<AdditinoalInfo>& vector, const unsigned int& id)
    {
        idCenter.remove(id);
        std::vector<AdditinoalInfo>::iterator iter;
        for (iter = vector.begin(); iter != vector.end(); ++iter)
        {
            if(iter->id == id)
            {
                vector.erase(iter);
                break;
            }
        }

    }

    std::vector<unsigned int> priorityIssue(std::vector<forDistancePriority>& distancePriorityArray)
    {
        std::vector<unsigned int> priorityTable;
        for (int k = 0; k < distancePriorityArray.size(); ++k)
        {
            for (int i = k; i < distancePriorityArray.size() - 1; ++i)
            {
                if(distancePriorityArray[i].distanceMin > distancePriorityArray[i+1].distanceMin)
                {
                    forDistancePriority temp;
                    temp = distancePriorityArray[i];
                    distancePriorityArray[i] = distancePriorityArray[i+1];
                    distancePriorityArray[i+1] = temp;
                }
            }
            priorityTable.push_back(distancePriorityArray[k].shortestDistanceIndex_ToPassID);
        }
        return priorityTable;
    }

    void DebuggingPrint(const unsigned int& initOldInfoSize, const unsigned int& initNewInfoSize)
    {
        cout << endl << endl;
        cout << "OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO" << endl;
        for (int i = 0; i < oldInfoArray.size(); ++i)
        {
            printf("i : %d\n", i);
            cout << "O distanceMin : " << oldInfoArray[i].distanceMin << endl;
            cout << "O matchingIndex : " << oldInfoArray[i].matchingIndex << endl;
            cout << "O distancePriority : " << oldInfoArray[i].distancePriority << endl;
            cout << "O isSimilarity : " << oldInfoArray[i].isSimilarity << endl;
            cout << "O isIdPassComplete : " << oldInfoArray[i].isIdPassComplete << endl;
            cout << "O id : " << oldInfoArray[i].id << endl;
            cout << "O maintain_count : " << oldInfoArray[i].maintain_count << endl;
            cout << "O type : " << oldInfoArray[i].type << endl;
            cout << "O speed : " << oldInfoArray[i].speed << endl;
            cout << "O theta : " << oldInfoArray[i].theta << endl;
            cout << "O theta_count : " << oldInfoArray[i].theta_count << endl;
            cout << "O vehicle_state : " << oldInfoArray[i].vehicle_state << endl;
            cout << endl;

            cout << "O pose" << endl;
            cout << "\tx : " << oldInfoArray[i].pose.x << endl;
            cout << "\ty : " << oldInfoArray[i].pose.y << endl;
            cout << "\tz : " << oldInfoArray[i].pose.z << endl;
            cout << "O size" << endl;
            cout << "\tmin_x : " << oldInfoArray[i].size.min_x << endl;
            cout << "\tmax_x : " << oldInfoArray[i].size.max_x << endl;
            cout << "\tmin_y : " << oldInfoArray[i].size.min_y << endl;
            cout << "\tmax_y : " << oldInfoArray[i].size.max_y << endl;
            cout << "\tmin_z : " << oldInfoArray[i].size.min_z << endl;
            cout << "\tmax_z : " << oldInfoArray[i].size.max_z << endl;
            cout << "O velocity" << endl;
            cout << "\tx : " << oldInfoArray[i].velocity.x << endl;
            cout << "\ty : " << oldInfoArray[i].velocity.y << endl;
            cout << "\tr : " << oldInfoArray[i].velocity.r << endl;
            cout << "\ttheta : " << oldInfoArray[i].velocity.theta << endl;
            cout << endl;

            cout << "idCenter : ";
            for (iter = idCenter.begin(); iter != idCenter.end(); ++iter)
            {
                if(iter != idCenter.end())
                {
                    cout << *iter << " ";
                }
            }
            cout << endl;
            cout << "- - - - - - - - - - - - - - - - -" << endl;

        }
        cout << "OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO" << endl;
        cout << endl;
        cout << "NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN" << endl;
        for (int i = 0; i < newInfoArray.size(); ++i)
        {
            printf("i : %d\n", i);
            cout << "N distanceMin : " << newInfoArray[i].distanceMin << endl;
            cout << "N matchingIndex : " << newInfoArray[i].matchingIndex << endl;
            cout << "N distancePriority : " << newInfoArray[i].distancePriority << endl;
            cout << "N isSimilarity : " << newInfoArray[i].isSimilarity << endl;
            cout << "N isIdPassComplete : " << newInfoArray[i].isIdPassComplete << endl;
            cout << "N id : " << newInfoArray[i].id << endl;
            cout << "N maintain_count : " << newInfoArray[i].maintain_count << endl;
            cout << "N type : " << newInfoArray[i].type << endl;
            cout << "N speed : " << newInfoArray[i].speed << endl;
            cout << "N theta : " << newInfoArray[i].theta << endl;
            cout << "N theta_count : " << newInfoArray[i].theta_count << endl;
            cout << "N vehicle_state : " << newInfoArray[i].vehicle_state << endl;
            cout << endl;

            cout << "N pose" << endl;
            cout << "\tx : " << newInfoArray[i].pose.x << endl;
            cout << "\ty : " << newInfoArray[i].pose.y << endl;
            cout << "\tz : " << newInfoArray[i].pose.z << endl;
            cout << "N size" << endl;
            cout << "\tmin_x : " << newInfoArray[i].size.min_x << endl;
            cout << "\tmax_x : " << newInfoArray[i].size.max_x << endl;
            cout << "\tmin_y : " << newInfoArray[i].size.min_y << endl;
            cout << "\tmax_y : " << newInfoArray[i].size.max_y << endl;
            cout << "\tmin_z : " << newInfoArray[i].size.min_z << endl;
            cout << "\tmax_z : " << newInfoArray[i].size.max_z << endl;
//            double shadowArea = (newInfoArray[i].size.max_x - newInfoArray[i].size.min_x) * (newInfoArray[i].size.max_y - newInfoArray[i].size.min_y);
//            cout << "\tshadowArea : " << newInfoArray[i].Volume << endl;
//            cout << "\tshadowArea : " << shadowArea << endl;
            cout << "N velocity" << endl;
            cout << "\tx : " << newInfoArray[i].velocity.x << endl;
            cout << "\ty : " << newInfoArray[i].velocity.y << endl;
            cout << "\tr : " << newInfoArray[i].velocity.r << endl;
            cout << "\ttheta : " << newInfoArray[i].velocity.theta << endl;
            cout << endl;

            cout << "idCenter : ";
            for (iter = idCenter.begin(); iter != idCenter.end(); ++iter)
            {
                if(iter != idCenter.end())
                {
                    cout << *iter << " ";
                }
            }
            cout << endl;
            cout << "- - - - - - - - - - - - - - - - -" << endl;

        }
        cout << "initOldInfoSize : " << initOldInfoSize << endl;
        cout << "initNewInfoSize : " << initNewInfoSize << endl;
        cout << "NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN" << endl;
    }

    void velocityMovementAverageFilter()
    {

    }

};

int main(int argc, char **argv)          //노드 메인함수 char **이다 계속 int하는데 주의할것
{
    ros::init(argc, argv, "jiwan_tracking");       //노드명 초기화

    ROS_INFO("started filter node");
    Tracking tracking;
    ros::spin();
    return 0;
}

////          출력 및 우선순위 선발
//            for (int k = 0; k < initalDataArray.size(); ++k)
//            {
//                cout << endl;
//                cout << "k : " << k << endl;
//                cout << "distanceMin : " << initalDataArray[k].distanceMin << endl;
//                cout << "shortestDistanceIndex_ToPassID : " << initalDataArray[k].shortestDistanceIndex_ToPassID << endl;
//            }
//
//            std::vector<unsigned int> priorityIndexTable = priorityIssue(initalDataArray);
//
////            if()
//            for (int k = 0; k < priorityIndexTable.size(); ++k)
//            {
//                cout << priorityIndexTable[k] << " ";
//                newInfoArray[priorityIndexTable[k]].id = oldInfoArray[k].id;
//            }
//            cout << endl;
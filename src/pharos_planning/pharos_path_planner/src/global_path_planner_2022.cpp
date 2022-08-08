#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pharos_road_information/RoadNetworks.h>
#include <pharos_road_information/Road.h>
#include <pharos_road_information/Lane.h>
#include <pharos_road_information/Lanes.h>
#include <pharos_road_information/Waypoint.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <pharos_path_planner/RoadInfo.h>
#include <pharos_path_planner/ReferencePath.h>

using namespace pharos_road_information;

RoadNetworksPtr Road_Networks_(new RoadNetworks);
nav_msgs::PathPtr final_path_(new nav_msgs::Path);

class GlobalPathPlanning
{
public:
    std::vector<std::vector<double> > road_graph_;

    int32_t publish_rate_;

    ros::NodeHandlePtr node_;
    ros::NodeHandlePtr pnode_;

    ros::Publisher global_path_pub_;
    ros::Subscriber road_net_graph_sub_;
    ros::Publisher current_pose_with_waypoint_pub_;

    std::vector<std::vector <Lane> > lane_graph_;

    unsigned int source_node_ ;
    unsigned int destination_node_;

    struct Node
    {
        double cost; // Can be distance or traveling time(dist/vel).
        bool visited; // 'True' means already be visited(checked) and 'False' means not check yet.
        int parent_node; // Parent node of this node. It is updated when minimum cost for this is updated.
    };


    void VectorToQuat(const tf::Vector3 vector3, geometry_msgs::Quaternion &pose_quaternion){

        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;

        yaw = atan2(vector3.y(),vector3.x());

        tf::Quaternion q;
        q.setRPY(roll,pitch,yaw);
        tf::quaternionTFToMsg(q,pose_quaternion);
    }

    inline tf::Vector3 VectorBetweenTwoPoses(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2){

        tf::Vector3 vector3;

        double dx = pose2.pose.position.x - pose1.pose.position.x;
        double dy = pose2.pose.position.y - pose1.pose.position.y;
        double dz = pose2.pose.position.z - pose1.pose.position.z;

        vector3.setValue(dx,dy,dz);

        return vector3;
    }

    int init()
    {
        node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        pnode_->param("publish_rate", publish_rate_, 5);

        // Publish
        global_path_pub_ = node_ -> advertise<nav_msgs::Path>(std::string("/path/reference_path"), 10);

        // Subscribe
        road_net_graph_sub_ = node_ -> subscribe("/road_network_graph", 5, &GlobalPathPlanning::RoadNetworkGraphCallback, this);
        // current_pose_with_waypoint_pub_ = node_->advertise<geometry_msgs::PoseStamped>("/pose_on_the_road", 10);

        // Init Parameters
        unsigned int source_node_ = 1;
        unsigned int destination_node_ = 29;

        return 0;
    }

    void RoadNetworkGraphCallback(const pharos_road_information::RoadNetworksConstPtr& msg){
        lane_graph_.clear(); //Shinwoo

        *Road_Networks_ = *msg;

        unsigned int number_of_roads_ = msg -> roads.size();
        for (unsigned int i = 0; i < number_of_roads_; i++){
            int number_of_lanes = msg -> roads.at(i).lanes.size();
            for (unsigned int j = 0; j < number_of_lanes; j++){
                int begin_node = msg -> roads[i].lanes[j].begin_node;
                int end_node = msg -> roads[i].lanes[j].end_node;
                road_graph_[begin_node][end_node] = msg -> roads[i].distance;
            }

            // Case: multiple lanes -> lane change
            if (number_of_lanes > 1){
                for (unsigned int j = 0; j < number_of_lanes - 1; j++){
                    int current_end_node = msg -> roads[i].lanes[j].end_node;
                    int next_begin_node = msg -> roads[i].lanes[j+1].begin_node;
                    road_graph_[current_end_node][next_begin_node] = 0.01;

                    current_end_node = msg -> roads[i].lanes[j+1].end_node;
                    next_begin_node = msg -> roads[i].lanes[j].begin_node;
                    road_graph_[current_end_node][next_begin_node] = 0.01;
                }
            }
        }

        // Find path using Dijkstra algorithm
        std::vector<int> solution_nodes; // Shinwoo

        DijkstraGraphSearch(source_node_,destination_node_, &road_graph_, &solution_nodes);

        // Publish global path (reference path)
        final_path_->poses.clear();

        std::cout << std::endl;

        std::vector<int>::iterator i;
        for (i = solution_nodes.begin(); i != solution_nodes.end(); i++){
            int begin_node;
            int end_node;

            begin_node = *i;
            end_node = *(i+1);

            std::cout << begin_node << "->";
            std::vector<Waypoint> waypoints = lane_graph_[begin_node][end_node].waypoints;
            if (waypoints.empty()) continue;

            geometry_msgs::PoseStamped wp;
            geometry_msgs::PoseStamped wp_next;

            unsigned int road_number = lane_graph_[begin_node][end_node].road_number;
            unsigned int lane_number = lane_graph_[begin_node][end_node].lane_number;

            std::vector<Waypoint>::iterator j; // Shinwoo
            for (j = waypoints.begin(); j != waypoints.end(); j++){
                wp.pose.position.x = (*j).x;
                wp.pose.position.y = (*j).y;
                wp.pose.position.z = 0.0;

                if(j+1 == waypoints.end()){
                    wp.pose.orientation = (final_path_->poses.back()).pose.orientation;
                }
                else{
                    wp_next.pose.position.x = (*(j+1)).x;
                    wp_next.pose.position.y = (*(j+1)).y;
                    wp_next.pose.position.z = (*(j+1)).z;

                    geometry_msgs::Quaternion quat;
                    tf::Vector3 vector3;
                    vector3 = VectorBetweenTwoPoses(wp,wp_next);
                    VectorToQuat(vector3, quat);
                    wp.pose.orientation = quat;
                }

                final_path_ -> poses.push_back(wp);
            }
        }
        final_path_->header.stamp = ros::Time::now();
        final_path_->header.frame_id = "odom";

        global_path_pub_.publish(final_path_);
    }

    void DijkstraGraphSearch(const unsigned int start_node, const unsigned int finish_node, std::vector<std::vector <double> > *Graph, std::vector<int> *solution_nodes)
    {
        Node node_init_value;
        node_init_value.cost = std::numeric_limits<double>::infinity();
        node_init_value.visited = false;
        node_init_value.parent_node = -1;

        double min_cost = std::numeric_limits<double>::infinity();

        std::vector<Node> nodes(Graph->size(), node_init_value);

        unsigned int current_node = start_node;
        nodes[current_node].cost = 0;
        nodes[current_node].visited = true;

        std::vector<int> neighbor_nodes;
        bool done = false;
        bool finish = false;

        while(!done){
            // Find neighbor nodes
            neighbor_nodes.clear();

            std::vector<double>::iterator i;
            int node_num = 0;

            for (i = Graph -> at(current_node).begin(); i != Graph -> at(current_node).end(); i++){
                if(*i != 0 && nodes[node_num].visited == false) neighbor_nodes.push_back(node_num);
                node_num++;
            }

            // Cost update
            std::vector<int>::iterator it;
            for (it = neighbor_nodes.begin(); it != neighbor_nodes.end(); it++){
                double dist = Graph -> at(current_node).at(*it);

                nodes[*it].cost = std::min(nodes[current_node].cost + dist, nodes[*it].cost);
                if (nodes[current_node].cost + dist < nodes[*it].cost) nodes[*it].parent_node = current_node;
            }

            // Node propagationint32_t publish_rate_;

            node_num = 0;
            std::vector<Node>::iterator it2;
            for (it2 = nodes.begin(); it2 != nodes.end(); it2++){
                if ((*it2).visited == false && (*it2).cost < min_cost){
                    min_cost = (*it2).cost;
                    current_node = node_num;
                }
                node_num++;
            }
            nodes[current_node].visited = true;

            // Done
            if (current_node == finish_node) finish = true;
        }

        // Resulting path
        std::vector<int> solution;

        if(finish) solution.push_back(finish_node);

        while (current_node != start_node){
            solution.push_back(nodes[current_node].parent_node);
            current_node = nodes[current_node].parent_node;

            std::cout << current_node << "->";
        }
        std::cout << std::endl;

        std::reverse(solution.begin(), solution.end());
        *solution_nodes = solution;
    }

    void publish()
    {
        ros::Rate loop_rate(publish_rate_);

        while (node_->ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GlobalPathPlanning_node");
    GlobalPathPlanning gpp_node;

    if (gpp_node.init())
    {
        ROS_FATAL("GlobalPathPlanning_node initialization failed");
        return -1;
    }

    gpp_node.publish();
    return 0;
}

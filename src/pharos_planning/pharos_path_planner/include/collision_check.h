#ifndef _COLLISION_CHECK_H_
#define _COLLISION_CHECK_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include "path_planner.h"

#define OCC_FULL 200
#define OCC_BASE 120
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))


class VehicleCollisionCheck{
public:

    double vehicle_width_;
    double vehicle_length_;
    double vehicle_wheel_base_;

    VehicleCollisionCheck(){
        vehicle_width_ = 2.0;
        vehicle_length_ = 4.4;
        // vehicle_length_ = 5.82;
        vehicle_wheel_base_ = 2.7;
    }
    void VehiclePoseToPolygon(const geometry_msgs::PoseStampedConstPtr &vehicle_pose, const double scale, geometry_msgs::PolygonPtr &polygon){

        geometry_msgs::Point32 p1,p2,p3,p4;

        double vehicle_length = vehicle_length_*scale;
        double vehicle_width = vehicle_width_*scale;
        double vehicle_wheel_base = vehicle_wheel_base_*scale;
//        ROS_ERROR("scale :: %lf \n" , scale);
//        ROS_ERROR("vehicle width  :: %lf \n\n" , vehicle_width );
//        ROS_ERROR("vehicle length :: %lf \n\n" , vehicle_length );

        double VehBox_Min_x = -(vehicle_length-vehicle_wheel_base)/2.0;
        double VehBox_Max_x = vehicle_wheel_base + (vehicle_length-vehicle_wheel_base)/2.0+0.5;
        double VehBox_Min_y = -vehicle_width/2.0;
        double VehBox_Max_y = vehicle_width/2.0;

        p1.x = VehBox_Min_x;
        p1.y = VehBox_Min_y;
        p1.z = 0.0;

        p2.x = VehBox_Max_x;
        p2.y = VehBox_Min_y;
        p2.z = 0.0;

        p3.x = VehBox_Max_x;
        p3.y = VehBox_Max_y;
        p3.z = 0.0;

        p4.x = VehBox_Min_x;
        p4.y = VehBox_Max_y;
        p4.z = 0.0;


        polygon->points.push_back(p1);
        polygon->points.push_back(p2);
        polygon->points.push_back(p3);
        polygon->points.push_back(p4);



        // Polygon Rotation
        tf::Pose pose;
        tf::poseMsgToTF(vehicle_pose->pose, pose);

        double Roll,Pitch, Yaw;
        static float vehicle_heading;
        pose.getBasis().getRPY(Roll, Pitch, Yaw);
        vehicle_heading = (float)Yaw;

        float x0,y0;
        float x1,y1;
        for(unsigned int i=0;i<polygon->points.size();i++){
            x0 = polygon->points.at(i).x;
            y0 = polygon->points.at(i).y;

            x1 = x0*(float)cos(vehicle_heading) - y0*(float)sin(vehicle_heading);
            y1 = x0*(float)sin(vehicle_heading) + y0*(float)cos(vehicle_heading);

            polygon->points.at(i).x = x1;
            polygon->points.at(i).y = y1;
        }

        // Polygon Translation
        for(unsigned int i=0;i<polygon->points.size();i++){
            polygon->points.at(i).x += vehicle_pose->pose.position.x;
            polygon->points.at(i).y += vehicle_pose->pose.position.y;
        }
    };

    void VehiclePoseToSqure(const geometry_msgs::PoseStampedConstPtr &vehicle_pose, const double scale, geometry_msgs::PolygonPtr &polygon){

        geometry_msgs::Point32 p1,p2,p3,p4;

        double length_of_squre = 5.5;
        double vehicle_length = length_of_squre*scale;
        double vehicle_width = length_of_squre*scale;
        double vehicle_wheel_base = length_of_squre*scale;

        double VehBox_Min_x = -vehicle_width/2.0;
        double VehBox_Max_x = vehicle_width/2.0;
        double VehBox_Min_y = -vehicle_width/2.0;
        double VehBox_Max_y = vehicle_width/2.0;

        p1.x = VehBox_Min_x;
        p1.y = VehBox_Min_y;
        p1.z = 0.0;

        p2.x = VehBox_Max_x;
        p2.y = VehBox_Min_y;
        p2.z = 0.0;

        p3.x = VehBox_Max_x;
        p3.y = VehBox_Max_y;
        p3.z = 0.0;

        p4.x = VehBox_Min_x;
        p4.y = VehBox_Max_y;
        p4.z = 0.0;

        polygon->points.push_back(p1);
        polygon->points.push_back(p2);
        polygon->points.push_back(p3);
        polygon->points.push_back(p4);

        // Polygon Rotation
        tf::Pose pose;
        tf::poseMsgToTF(vehicle_pose->pose, pose);

        double Roll,Pitch, Yaw;
        static float vehicle_heading;
        pose.getBasis().getRPY(Roll, Pitch, Yaw);
        vehicle_heading = (float)Yaw;

        float x0,y0;
        float x1,y1;
        for(unsigned int i=0;i<polygon->points.size();i++){
            x0 = polygon->points.at(i).x;
            y0 = polygon->points.at(i).y;

            x1 = x0*(float)cos(vehicle_heading) - y0*(float)sin(vehicle_heading);
            y1 = x0*(float)sin(vehicle_heading) + y0*(float)cos(vehicle_heading);

            polygon->points.at(i).x = x1;
            polygon->points.at(i).y = y1;
        }

        // Polygon Translation
        for(unsigned int i=0;i<polygon->points.size();i++){
            polygon->points.at(i).x += vehicle_pose->pose.position.x;
            polygon->points.at(i).y += vehicle_pose->pose.position.y;
        }
    };


    bool CollisionCheck(const geometry_msgs::PoseStampedConstPtr vehicle_pose,
                        const tf::Point EgoVehiclePose,
                        double collision_ragne_scale,
                        const nav_msgs::OccupancyGridConstPtr &map,nav_msgs::OccupancyGridPtr &history_map){


        bool collision = false;

        geometry_msgs::PolygonPtr vehicle_polygon (new geometry_msgs::Polygon);

        //vehicle_pose to polygon

        double min_scale = 1.0;
        double max_scale = 1.1;

        // double vehicle_scale = ((max_scale-min_scale)/max_travel_dist)*travel_dist + min_scale;
        double vehicle_scale = collision_ragne_scale;

//        std::cout <<"collisioncheck vehicle pose " << "vehicle pose :: " << *vehicle_pose << std::endl;
//        printf("\n");
//        ROS_ERROR("In collision check collision range scale :: %lf \n" , collision_ragne_scale);
        VehiclePoseToPolygon(vehicle_pose,vehicle_scale,vehicle_polygon);


//        std::cout <<"before minus ego vehicle pose" << "vehicle polygon :: " << *vehicle_polygon << std::endl;
//        printf("\n");

        for(unsigned int i=0; i<vehicle_polygon->points.size();i++){
            vehicle_polygon->points.at(i).x -= EgoVehiclePose.x();
            vehicle_polygon->points.at(i).y -= EgoVehiclePose.y();
        }

//        std::cout <<"after minus ego vehicle pose" << "vehicle polygon :: " << *vehicle_polygon << std::endl;
//        printf("\n");

        //To Grid Index
        int grid_size = map->info.height;
        int gridXOrigin = grid_size/2;
        int gridYOrigin = grid_size/2;
        double resolution = map->info.resolution;
        double resolutionInverse = 1/resolution;

        //---------Density collision check
        std::vector<tf::Point> New_p1;
        std::vector<tf::Point> New_p2;
        unsigned int deviding = 15;

        tf::Point new_p1_begin,new_p1_end;
        tf::Point new_p2_begin,new_p2_end;
        new_p1_begin.setValue(vehicle_polygon->points.at(0).x,vehicle_polygon->points.at(0).y,0.0);
        new_p1_end.setValue(vehicle_polygon->points.at(1).x,vehicle_polygon->points.at(1).y,0.0);
        new_p2_begin.setValue(vehicle_polygon->points.at(3).x,vehicle_polygon->points.at(3).y,0.0);
        new_p2_end.setValue(vehicle_polygon->points.at(2).x,vehicle_polygon->points.at(2).y,0.0);

        double dist = new_p1_begin.distance(new_p1_end);



        double dev_dist = dist/(deviding);
        for(unsigned int i=1; i<deviding;i++){
            tf::Point p1;
            double p1_x = new_p1_begin.x()*(1.0-(double)i/deviding) + new_p1_end.x()*((double)i/deviding);
            double p1_y = new_p1_begin.y()*(1.0-(double)i/deviding) + new_p1_end.y()*((double)i/deviding);
            p1.setValue(p1_x,p1_y,0.0);
            New_p1.push_back(p1);

            tf::Point p2;
            double p2_x = new_p2_begin.x()*(1.0-(double)i/deviding) + new_p2_end.x()*((double)i/deviding);
            double p2_y = new_p2_begin.y()*(1.0-(double)i/deviding) + new_p2_end.y()*((double)i/deviding);
            p2.setValue(p2_x,p2_y,0.0);
            New_p2.push_back(p2);
        }
        tf::Point p3,p4;
        double p3_x = (vehicle_polygon->points.at(0).x + vehicle_polygon->points.at(3).x)/2.0;
        double p3_y = (vehicle_polygon->points.at(0).y + vehicle_polygon->points.at(3).y)/2.0;
        double p4_x = (vehicle_polygon->points.at(1).x + vehicle_polygon->points.at(2).x)/2.0;
        double p4_y = (vehicle_polygon->points.at(1).y + vehicle_polygon->points.at(2).y)/2.0;

        p3.setValue(p3_x,p3_y,0.0);
        p4.setValue(p4_x,p4_y,0.0);

        New_p1.push_back(p3);
        New_p2.push_back(p4);



        for(unsigned int i=0; i<New_p1.size();i++){
            double x1 = New_p1.at(i).x();
            double y1 = New_p1.at(i).y();
            double x2 = New_p2.at(i).x();
            double y2 = New_p2.at(i).y();
            double a = (y2-y1)/(x2-x1);
            double b = y1-a*x1;

            double inclination = atan2(y2-y1,x2-x1);

            //printf("collision check!\n");
            if(fabs(inclination) < M_PI/4 || fabs(inclination) > (3.0/4.0)*M_PI) {
                double x = x1;
                if (x1 < x2) {
                    while (x < x2) {
                        double y = a * x + b;
                        int int_x = (unsigned int) (x * resolutionInverse) + gridXOrigin;
                        int int_y = (unsigned int) (y * resolutionInverse) + gridYOrigin;
                        if (int_x >= 0 && int_x < grid_size && int_y >= 0 && int_y < grid_size) {
                            unsigned int IDX = MAP_IDX(grid_size, int_x, int_y);
//                            printf("1 idx: %d\n",IDX);
                            history_map->data[IDX] = OCC_BASE;
                            if ((uint8_t) map->data.at(IDX) == OCC_FULL) {
                                collision = true;
                                break;
                            }
                        }
                        double x_res = fabs(resolution * cos(atan2(y2 - y1, x2 - x1)));
                        x += x_res;

                    }
                } else if (x1 > x2) {

                    while (x > x2) {
                        double y = a * x + b;
                        int int_x = (unsigned int) (x * resolutionInverse) + gridXOrigin;
                        int int_y = (unsigned int) (y * resolutionInverse) + gridYOrigin;
                        if (int_x >= 0 && int_x < grid_size && int_y >= 0 && int_y < grid_size) {
                            unsigned int IDX = MAP_IDX(grid_size, int_x, int_y);
                            //printf("2 idx: %d\n",IDX);
                            history_map->data[IDX] = OCC_BASE;
                            if ((uint8_t) map->data.at(IDX) == OCC_FULL) {
                                collision = true;

                                break;
                            }
                        }
                        double x_res = fabs(resolution * cos(atan2(y2 - y1, x2 - x1)));
                        x -= x_res;
                    }
                }
            }
            else{ // x1 == x2

                double y = y1;
                if(y1 < y2){
                    while(y < y2){
                        double x = (y-b)/a;
                        int int_x = (unsigned int)(x  * resolutionInverse) + gridXOrigin;
                        int int_y = (unsigned int)(y  * resolutionInverse) + gridYOrigin;
                        if(int_x >= 0 && int_x < grid_size && int_y >= 0 && int_y < grid_size){
                            unsigned int IDX = MAP_IDX(grid_size,int_x,int_y);
//                            printf("2 idx: %d\n",IDX);
                            history_map->data[IDX] = OCC_BASE;
                            if((uint8_t)map->data.at(IDX) == OCC_FULL){
                                collision = true;
                                break;
                            }
                        }
                        double y_res = resolution;
                        y += y_res;
                    }
                }
                else{

                    while(y > y2){
                        double x = (y-b)/a;
                        int int_x = (unsigned int)(x  * resolutionInverse) + gridXOrigin;
                        int int_y = (unsigned int)(y  * resolutionInverse) + gridYOrigin;
                        if(int_x >= 0 && int_x < grid_size && int_y >= 0 && int_y < grid_size){
                            unsigned int IDX = MAP_IDX(grid_size,int_x,int_y);
                            //printf("2 idx: %d\n",IDX);
                            history_map->data[IDX] = OCC_BASE;
                            if((uint8_t)map->data.at(IDX) == OCC_FULL){
                                collision = true;

                                break;
                            }
                        }
                        double y_res = resolution;
                        y -= y_res;
                    }
                }
            }
            if(collision) break;
        }

        for(unsigned int i=0; i<vehicle_polygon->points.size();i++){
            float x1 = vehicle_polygon->points.at(i).x;
            float y1 = vehicle_polygon->points.at(i).y;
            float x2 = vehicle_polygon->points.at((i+1)%vehicle_polygon->points.size()).x;
            float y2 = vehicle_polygon->points.at((i+1)%vehicle_polygon->points.size()).y;

            double a = (y2-y1)/(x2-x1);
            double b = y1-a*x1;

            double inclination = atan2(y2-y1,x2-x1);

            //printf("collision check!\n");
            if(fabs(inclination) < M_PI/4 || fabs(inclination) > (3.0/4.0)*M_PI) {
                double x = x1;
                if (x1 < x2) {
                    while (x < x2) {
                        double y = a * x + b;
                        int int_x = (unsigned int) (x * resolutionInverse) + gridXOrigin;
                        int int_y = (unsigned int) (y * resolutionInverse) + gridXOrigin;
                        if (int_x >= 0 && int_x < grid_size && int_y >= 0 && int_y < grid_size) {
                            unsigned int IDX = MAP_IDX(grid_size, int_x, int_y);
                            //printf("1 idx: %d\n",IDX);
                            history_map->data[IDX] = OCC_BASE;
                            if ((uint8_t) map->data.at(IDX) == OCC_FULL) {
                                collision = true;

                                break;
                            }
                        }
                        double x_res = fabs(resolution * cos(atan2(y2 - y1, x2 - x1)));
                        x += x_res;

                    }
                } else if (x1 > x2) {
                    while (x > x2) {
                        double y = a * x + b;
                        int int_x = (unsigned int) (x * resolutionInverse) + gridXOrigin;
                        int int_y = (unsigned int) (y * resolutionInverse) + gridXOrigin;
                        if (int_x >= 0 && int_x < grid_size && int_y >= 0 && int_y < grid_size) {
                            unsigned int IDX = MAP_IDX(grid_size, int_x, int_y);
                            //printf("2 idx: %d\n",IDX);
                            history_map->data[IDX] = OCC_BASE;
                            if ((uint8_t) map->data.at(IDX) == OCC_FULL) {
                                collision = true;

                                break;
                            }
                        }
                        double x_res = fabs(resolution * cos(atan2(y2 - y1, x2 - x1)));
                        x -= x_res;

                    }
                }
            }
            else{ // x1 == x2
                double y = y1;
                if(y1 < y2){
                    while(y < y2){
                        double x = (y-b)/a;
                        int int_x = (unsigned int)(x  * resolutionInverse) + gridXOrigin;
                        int int_y = (unsigned int)(y  * resolutionInverse) + gridXOrigin;
                        if(int_x >= 0 && int_x < grid_size && int_y >= 0 && int_y < grid_size){
                            unsigned int IDX = MAP_IDX(grid_size,int_x,int_y);
                            //printf("2 idx: %d\n",IDX);
                            history_map->data[IDX] = OCC_BASE;
                            if((uint8_t)map->data.at(IDX) == OCC_FULL){
                                collision = true;

                                break;
                            }
                        }
                        double y_res = resolution;
                        y += y_res;
                    }
                }
                else{
                    while(y > y2){
                        double x = (y-b)/a;
                        int int_x = (unsigned int)(x  * resolutionInverse) + gridXOrigin;
                        int int_y = (unsigned int)(y  * resolutionInverse) + gridXOrigin;
                        if(int_x >= 0 && int_x < grid_size && int_y >= 0 && int_y < grid_size){
                            unsigned int IDX = MAP_IDX(grid_size,int_x,int_y);
                            //printf("2 idx: %d\n",IDX);
                            history_map->data[IDX] = OCC_BASE;
                            if((uint8_t)map->data.at(IDX) == OCC_FULL){
                                collision = true;

                                break;
                            }
                        }
                        double y_res = resolution;
                        y -= y_res;
                    }
                }
            }

            if(collision) break;
        }
        //---------------
        return collision;
    };
};

#endif


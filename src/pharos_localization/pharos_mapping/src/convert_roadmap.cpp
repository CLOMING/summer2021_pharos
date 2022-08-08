#include <ros/ros.h>
#include <cstdlib>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetMapResponse.h>
#include <tf/tf.h>

void mapsave(nav_msgs::OccupancyGrid map, std::string mapname)
{
    printf("Received a %d X %d map @ %.3f m/pix\n",
           map.info.width,
           map.info.height,
           map.info.resolution);


    std::string mapdatafile = mapname + ".pgm";
    printf("Writing map occupancy data to %s\n", mapdatafile.c_str());
    FILE* out = fopen(mapdatafile.c_str(), "w");
    if (!out)
    {
        printf("Couldn't save map file to %s", mapdatafile.c_str());
        return;
    }

    fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
            map.info.resolution, map.info.width, map.info.height);
    for(unsigned int y = 0; y < map.info.height; y++) {
        for(unsigned int x = 0; x < map.info.width; x++) {
            unsigned int i = x + (map.info.height - y - 1) * map.info.width;
//          if (map->data[i] == 0) { //occ [0,0.1)
//            fputc(254, out);
//          } else if (map->data[i] == +100) { //occ (0.65,1]
//            fputc(000, out);
//          } else { //occ [0.1,0.65]
//            fputc(205, out);
//          }
            fputc(map.data[i],out);
        }
    }

    fclose(out);

    std::string mapmetadatafile = mapname + ".yaml";
    printf("Writing map occupancy data to %s\n", mapmetadatafile.c_str());
    FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


    /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

     */

    geometry_msgs::Quaternion orientation = map.info.origin.orientation;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
            mapdatafile.c_str(), map.info.resolution, map.info.origin.position.x, map.info.origin.position.y, yaw);

    fclose(yaml);

    printf("%s Save Converted Map Done\n\n",mapname.c_str());
}

void MapCB(const nav_msgs::OccupancyGridConstPtr &Map){
    ROS_INFO("Convert Roadinfo Map Start!!!");

    nav_msgs::OccupancyGrid convert_map;
    unsigned long RoadinfoMapArraySize = Map->data.size();

    convert_map.header = Map->header;
    convert_map.data.resize(RoadinfoMapArraySize,0);
    convert_map.info = Map->info;
    std::cout << RoadinfoMapArraySize << std::endl;

    for(int i=0;i<RoadinfoMapArraySize;i++){
//        std::cout << i << std::endl;
        if(Map->data[i]>20){
            convert_map.data[i]=255;
        }
    }

    ROS_INFO("Convert Roadinfo Map Done!!!");

    mapsave(convert_map, "ConvertedMap");
}

int main(int argc, char** argv) {
    ros::init(argc,argv,"convert_roadmap_node");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    ros::Subscriber roadmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/roadinfo_map", 1, MapCB);

    ros::spin();

}
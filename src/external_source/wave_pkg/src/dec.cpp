//ROS
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <wave_pkg/GeographicalPath.h>
#include <wave_pkg/AccelerationSet4Way_t.h>
#include <wave_pkg/AntennaOffsetSet_t.h>
#include <wave_pkg/BrakeSystemStatus_t.h>
#include <wave_pkg/BSM.h>
#include <wave_pkg/BSMcoreData.h>
#include <wave_pkg/BSMpartII.h>
#include <wave_pkg/ConnectsToList.h>
#include <wave_pkg/FullPositionVector.h>
#include <wave_pkg/IntersectionGeometryList.h>
#include <wave_pkg/IntersectionReferenceID.h>
#include <wave_pkg/IntersectionStateList_t.h>
#include <wave_pkg/IntersectionReferenceID_t.h>
#include <wave_pkg/LaneAttributes_t.h>
#include <wave_pkg/LaneList_t.h>
#include <wave_pkg/MAP.h>
#include <wave_pkg/MovementEventList_t.h>
#include <wave_pkg/MovementList_t.h>
#include <wave_pkg/NodeAttributeSetXY.h>
#include <wave_pkg/NodeListLL_t.h>
#include <wave_pkg/NodeListXY_t.h>
#include <wave_pkg/NodeOffsetPointXY_t.h>
#include <wave_pkg/OffsetSystem_t.h>
#include <wave_pkg/PathHistory_t.h>
#include <wave_pkg/Position3D.h>
#include <wave_pkg/Position3D_t.h>
#include <wave_pkg/PositionalAccuracy_t.h>
#include <wave_pkg/RTCM.h>
#include <wave_pkg/RTCMheader.h>
#include <wave_pkg/RTCMmessageList_t.h>
#include <wave_pkg/SPAT.h>
#include <wave_pkg/SpecialVehicleExtensions_t.h>
#include <wave_pkg/SupplementalVehicleExtensions_t.h>
#include <wave_pkg/TIM.h>
#include <wave_pkg/TimeChangeDetails.h>
#include <wave_pkg/TravelerDataFrame__content_PR.h>
#include <wave_pkg/TravelerDataFrameList_t.h>
#include <wave_pkg/VehicleSafetyExtensions_t.h>
#include <wave_pkg/VehicleSize_t.h>

#include <IA5String.h>
#include <iostream>
#include <string>
#include <bitset>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <IntersectionGeometryList.h>
#include <TravelerDataFrameList.h>
#include <TravelerDataFrame.h>
#include <NodeAttributeXYList.h>
#include <NodeSetXY.h>
#include <IntersectionStateList.h>
#include <MovementList.h>
#include <MovementEventList.h>
#include <BasicSafetyMessage.h>
#include <LaneDataAttributeList.h>
#include <EventDescription.h>
#include <LaneList.h>
#include <ConnectsToList.h>
#include <RTCMmessageList.h>
#include <WorkZone.h>

#include "MessageFrame.h"

#pragma pack(push,1);

#define J2735_UPER_MAP  18
#define J2735_UPER_SPAT 19
#define J2735_UPER_BSM  20
#define J2735_UPER_RTCM 28
#define J2735_UPER_TIM  31

ros::NodeHandlePtr pnode_;
ros::NodeHandlePtr node_;

ros::Publisher tim_msg_pub_;
ros::Publisher spat_msg_pub_;
ros::Publisher map_msg_pub_;
ros::Publisher bsm_msg_pub_;
ros::Publisher rtcm_msg_pub_;



typedef struct AVC_HDR {
    uint32_t sec;
    uint32_t usec;
    uint32_t type;
    uint32_t len;
} __attribute__((packed)) AVC_HDR_t;

typedef struct AVC_TAIL {
    uint16_t crc16;
} __attribute__((packed)) AVC_TAIL_t;

static int RUNNING = 0;


static void decode_MapData(MessageFrame_t *frame){

    wave_pkg::MAP _msg;
    wave_pkg::IntersectionGeometryList _intersectionGeometryList;
    wave_pkg::IntersectionReferenceID _id;
    wave_pkg::Position3D_t _refPoint;
    wave_pkg::LaneList_t _laneSet;
    wave_pkg::LaneAttributes_t _laneAttributes;
    wave_pkg::NodeListXY_t _nodeList;
    wave_pkg::ConnectsToList _connectsTo;
    wave_pkg::NodeOffsetPointXY_t _delta;
    wave_pkg::NodeAttributeSetXY _attributes;
    

    _msg.msgCnt = frame->value.MapData.msgIssueRevision;
    _msg.layertype = *frame->value.MapData.layerType;

    char geometryname_buf_[63];
    for (int i = 0; i < sizeof(geometryname_buf_); i++) {
        geometryname_buf_[i] = NULL;
    }
    for (int i = 0; i < frame->value.MapData.intersections->list.array[0]->name->size; i++) {
        geometryname_buf_[i] = frame->value.MapData.intersections->list.array[0]->name->buf[i];
    }
    std::string _std(geometryname_buf_);
//    std::cout <<"map msg name "<< _std << std::endl;
    _intersectionGeometryList.name = _std;

    _intersectionGeometryList.revision = frame->value.MapData.intersections->list.array[0]->revision;
    _intersectionGeometryList.lanewidth = *frame->value.MapData.intersections->list.array[0]->laneWidth;

    _id.id = frame->value.MapData.intersections->list.array[0]->id.id;
    _id.region = *frame->value.MapData.intersections->list.array[0]->id.region;

    _refPoint.latitude = frame->value.MapData.intersections->list.array[0]->refPoint.lat;
    _refPoint.longitude = frame->value.MapData.intersections->list.array[0]->refPoint.Long;
    _refPoint.elevation = *frame->value.MapData.intersections->list.array[0]->refPoint.elevation;

    frame->value.MapData.intersections->list.array[0]->laneSet.list.array[0]->nodeList.nodes.list.array[0]->delta.node_LatLon.lat;

//    for (int i = 0; i < frame->value.MapData.intersections->list.array[0]->laneSet.list.size-2; i++)
//    {
//
//        _laneSet.laneAttributes.clear();
//        _laneSet.nodeList.clear();
//        _nodeList.delta.clear();
//        _nodeList.attributes.clear();
//        _laneSet.connectsTo.clear();
//
//
//        _laneSet.laneID = frame->value.MapData.intersections->list.array[0]->laneSet.list.array[i]->laneID;
////        _laneAttributes.directionalUse = frame->value.MapData.intersections->list.array[0]->laneSet.list.array[i]->laneAttributes.directionalUse.buf[]
//        _laneAttributes.laneType = *frame->value.MapData.intersections->list.array[0]->laneSet.list.array[i]->laneAttributes.laneType.vehicle.buf;
//        _laneSet.maneuvers = *frame->value.MapData.intersections->list.array[0]->laneSet.list.array[i]->maneuvers->buf;
//
//
//        if(frame->value.MapData.intersections->list.array[0]->laneSet.list.array[i]->connectsTo)
//        {
//            for(int k = 0; k< frame->value.MapData.intersections->list.array[0]->laneSet.list.array[i]->connectsTo->list.size/2 ; k++)
//            {
//                _connectsTo.landid = frame->value.MapData.intersections->list.array[0]->laneSet.list.array[i]->connectsTo->list.array[k]->connectingLane.lane;
//                _connectsTo.maneuvers = *frame->value.MapData.intersections->list.array[0]->laneSet.list.array[i]->connectsTo->list.array[k]->connectingLane.maneuver->buf;
//                if(frame->value.MapData.intersections->list.array[0]->laneSet.list.array[i]->connectsTo->list.array[k]->signalGroup)
//                {
//                    _connectsTo.signalGroup  = *frame->value.MapData.intersections->list.array[0]->laneSet.list.array[i]->connectsTo->list.array[k]->signalGroup;
//                }
//
//                _laneSet.connectsTo.push_back(_connectsTo);
//            }
//        }
//
//        for(int j = 0; j< frame->value.MapData.intersections->list.array[0]->laneSet.list.array[i]->nodeList.nodes.list.size-1; j++)
//        {
////            _nodeList.attributes = frame->value.MapData.intersections->list.array[0]->laneSet.list.array[i]->nodeList.nodes.list.array[j]->attributes->localNode->list.array;
//            _delta.x = frame->value.MapData.intersections->list.array[0]->laneSet.list.array[i]->nodeList.nodes.list.array[j]->delta.node_XY6.x;
//            _delta.y = frame->value.MapData.intersections->list.array[0]->laneSet.list.array[i]->nodeList.nodes.list.array[j]->delta.node_XY6.y;
//            _nodeList.delta.push_back(_delta);
////            _nodeList.attributes.push_back(_attributes);
//        }
//        _laneSet.nodeList.push_back(_nodeList);
//        _laneSet.laneAttributes.push_back(_laneAttributes);
//        _intersectionGeometryList.laneSet.push_back(_laneSet);
//
//    }

//    _laneAttributes.directionalUse = u_long(frame->value.MapData.intersections->list.array[0]->laneSet.list.array[0]->laneAttributes.directionalUse);
    //    ^^^^^^^^^^^^^^^^^^^^^^^^^^^ string
//    _laneAttributes.laneType = frame->value.MapData.intersections->list.array[0]->laneSet.list.array[0]->laneAttributes.laneType; // need to make structure
//    _laneSet.maneuvers = u_int64_t(frame->value.MapData.intersections->list.array[0]->laneSet.list.array[0]->maneuvers);
//    _delta.x = u_int16_t(frame->value.MapData.intersections->list.array[0]->laneSet.list.array[0]->nodeList.nodes.list.array[0]->delta.node_XY1.x);
//    _delta.y = u_int16_t(frame->value.MapData.intersections->list.array[0]->laneSet.list.array[0]->nodeList.nodes.list.array[0]->delta.node_XY1.y);

    //core dumped code
    //std::cout << frame->value.MapData.intersections->list.array[0]->laneSet.list.array[0]->nodeList.nodes.list.array[0]->delta.node_XY1.y << std::endl;
    //std::cout << frame->value.MapData.intersections->list.array[0]->laneSet.list.array[0]->nodeList.nodes.list.array[0]->attributes-> << std::endl;
    //std::cout << frame->value.MapData.intersections->list.array[0]->laneSet.list.array[0]->nodeList.nodes.list.count << std::endl;
//    _attributes.localNodes = frame->value.MapData.intersections->list.array[0]->laneSet.list.array[0]->nodeList.nodes.list.array[0]->attributes->localNode;
    //std::cout << frame->value.MapData.intersections->list.array[0]->laneSet.list.array[0]->nodeList.nodes.list.array[0]->attributes << std::endl;
    //ROS_ERROR("sdfweg123123123");
    //if(frame->value.MapData.intersections->list.array[0]->laneSet.list.array[0]->nodeList.nodes.list.size != 0)
    //{
    //   ROS_ERROR("gsdfsdfawega");
    //    std::cout << frame->value.MapData.intersections->list.array[0]->laneSet.list.array[0]->nodeList.nodes.list.array[0]->attributes<< std::endl;
    //}

    //_connectsTo.landid = u_int64_t(frame->value.MapData.intersections->list.array[0]->laneSet.list.array[0]->connectsTo->list.array[0]->connectionID);
    //_connectsTo.maneuvers = u_int64_t(frame->value.MapData.intersections->list.array[0]->laneSet.list.array[0]->connectsTo->list.array[0]->connectingLane.maneuver);
    //_connectsTo.signalGroup = u_int64_t(frame->value.MapData.intersections->list.array[0]->laneSet.list.array[0]->connectsTo->list.array[0]->signalGroup);

    _intersectionGeometryList.id.push_back(_id);
    _intersectionGeometryList.refPoint.push_back(_refPoint);
    _msg.intersectionGeometryList.push_back(_intersectionGeometryList);


    map_msg_pub_.publish(_msg);
}

static void decode_SignalPhaseAndTimingMessage(MessageFrame_t *frame){

    wave_pkg::SPAT _msg;
    wave_pkg::IntersectionStateList_t _intersections;
    wave_pkg::IntersectionReferenceID_t _id;
    wave_pkg::MovementList_t _states;
    wave_pkg::MovementEventList_t _state_time_speed;
    wave_pkg::TimeChangeDetails _timing;


    //containing wave messages in msg structures//



    char msgbuf_[63];
    for(int i=0;i<sizeof(msgbuf_);i++)
    {
        msgbuf_[i] = NULL;
    }
    for(int i=0;i<frame->value.SPAT.name->size;i++)
    {
        msgbuf_[i] = frame->value.SPAT.name->buf[i];
    }

    std::string _std(msgbuf_);
//    std::cout << _std << std::endl;

    _intersections.name = _std;
    _id.id = frame->value.SPAT.intersections.list.array[0]->id.id;
    _id.region = *frame->value.SPAT.intersections.list.array[0]->id.region;
    _intersections.msgCnt = frame->value.SPAT.intersections.list.array[0]->revision;

    for(int i = 0; i< frame->value.SPAT.intersections.list.array[0]->status.size; i++)
    {
        _intersections.statusSPAT = frame->value.SPAT.intersections.list.array[0]->status.buf[i];
    }


//    for(int i=0; i < frame->value.SPAT.intersections.list.size; i++)
//    {
//        printf("intersection size : %d\n",frame->value.SPAT.intersections.list.size);
//        if(frame->value.SPAT.intersections.list.size != 0)
//        {
//            printf("intersection list size is not 0 !!\n");
//            printf("i is %d \n" , i);
//            printf("array size is %d\n",frame->value.SPAT.intersections.list.size);
//
//            if(frame->value.SPAT.intersections.list.array[1]->status.size != 0)
//            {
//                printf("status size is %d\n", frame->value.SPAT.intersections.list.array[1]->status.size);
//            }
//
////            for(int j =0; j < frame->value.SPAT.intersections.list.array[i]->status.size; j++ )
////            {
////                printf("buf yet!!\n");
////                printf("%d\n",frame->value.SPAT.intersections.list.array[i]->status.buf[j]);
////                printf("done!!\n");
////            }
//
//        }
//
//    }


//    _intersections.maneuverAssist = frame->value.SPAT.intersections.list.array[0]->maneuverAssistList->list.array[0]->pedBicycleDetect;

//    _id.region = frame->value.SPAT.intersections.list.array[0]->id.region;

    for(int k = 0; k<frame->value.SPAT.intersections.list.array[0]->states.list.size; k++)
    {
        _state_time_speed.timing.clear();
        _states.state_time_speed.clear();
        _states.signalGroupId = frame->value.SPAT.intersections.list.array[0]->states.list.array[k]->signalGroup;

        _states.name = *frame->value.SPAT.intersections.list.array[0]->states.list.array[k]->movementName->buf;  //string make char to string



        _state_time_speed.eventState = frame->value.SPAT.intersections.list.array[0]->states.list.array[k]->state_time_speed.list.array[0]->eventState;
        _timing.minEndTime = frame->value.SPAT.intersections.list.array[0]->states.list.array[k]->state_time_speed.list.array[0]->timing->minEndTime;
//        _timing.maxEndTime = frame->value.SPAT.intersections.list.array[0]->states.list.array[k]->state_time_speed.list.array[0]->timing->maxEndTime;

        _state_time_speed.timing.push_back(_timing);
        _states.state_time_speed.push_back(_state_time_speed);
        _intersections.states.push_back(_states);

    }




    //std::cout << frame->value.SPAT.intersections.list.array[0]->states.list.array[0]->state_time_speed.list.array[0]->timing->minEndTime << std::endl;

//    _state_time_speed.timing.push_back(_timing);
//    _states.state_time_speed.push_back(_state_time_speed);
//    _intersections.states.push_back(_states);
    _intersections.id.push_back(_id);
    _msg.intersections.push_back(_intersections);

//    std::cout << frame->value.SPAT << std::endl;

    spat_msg_pub_.publish(_msg);
}

static void decode_BasicSafetyMessage(MessageFrame_t *frame){

    wave_pkg::BSM _msg;
    wave_pkg::BSMcoreData _coredata;
    wave_pkg::BSMpartII _partII;
    wave_pkg::PositionalAccuracy_t _accuracy;
    wave_pkg::AccelerationSet4Way_t _accelSet;
    wave_pkg::BrakeSystemStatus_t _brakes;
    wave_pkg::VehicleSize_t _size;
    wave_pkg::VehicleSafetyExtensions_t _VehicleSafetyExtensions;
    wave_pkg::SpecialVehicleExtensions_t _SpecialVehicleExtensions;
    wave_pkg::SupplementalVehicleExtensions_t _SupplementalVehicleExtensions;
    wave_pkg::PathHistory_t _pathHistorySets;

//    //containing wave messages in msg structures//
//
    _coredata.msgCnt = uint8_t(frame->value.BasicSafetyMessage.coreData.msgCnt);

//    _coredata.id = std::string(frame->value.BasicSafetyMessage.coreData.id);

    _coredata.secMark = u_int16_t(frame->value.BasicSafetyMessage.coreData.secMark);
    _coredata.lat = int32_t(frame->value.BasicSafetyMessage.coreData.lat);
    _coredata.longitude = int32_t(frame->value.BasicSafetyMessage.coreData.Long);
    _coredata.elev = int16_t(frame->value.BasicSafetyMessage.coreData.elev);

//    _coredata.accuracy = (frame->value.BasicSafetyMessage.coreData.accuracy);
    _coredata.transmission = frame->value.BasicSafetyMessage.coreData.transmission;
    _coredata.speed = frame->value.BasicSafetyMessage.coreData.speed;
    _coredata.heading = frame->value.BasicSafetyMessage.coreData.heading;
    _coredata.angle = frame->value.BasicSafetyMessage.coreData.angle;

    _accuracy.semiMinor = frame->value.BasicSafetyMessage.coreData.accuracy.semiMinor;
    _accuracy.semiMajor = frame->value.BasicSafetyMessage.coreData.accuracy.semiMajor;
    _accuracy.orientation = frame->value.BasicSafetyMessage.coreData.accuracy.orientation;



    _accelSet.longitude = frame->value.BasicSafetyMessage.coreData.accelSet.Long;
    _accelSet.lat    = frame->value.BasicSafetyMessage.coreData.accelSet.lat;
    _accelSet.vert = frame->value.BasicSafetyMessage.coreData.accelSet.vert;
    _accelSet.yaw = frame->value.BasicSafetyMessage.coreData.accelSet.yaw;

//    _brakes.wheelBrakes = u_long(frame->value.BasicSafetyMessage.coreData.brakes.wheelBrakes); //need to make 4 types of brake system
//    printf("%d\n",frame->value.BasicSafetyMessage.coreData.brakes.wheelBrakes.buf[0]);
    _brakes.traction = frame->value.BasicSafetyMessage.coreData.brakes.traction;
    _brakes.albs = frame->value.BasicSafetyMessage.coreData.brakes.albs;
    _brakes.scs = frame->value.BasicSafetyMessage.coreData.brakes.scs;
    _brakes.brakeBoost = frame->value.BasicSafetyMessage.coreData.brakes.brakeBoost;
    _brakes.auxBrakes = frame->value.BasicSafetyMessage.coreData.brakes.auxBrakes;

    _size.width = frame->value.BasicSafetyMessage.coreData.size.width;
    _size.length = frame->value.BasicSafetyMessage.coreData.size.length;

    _VehicleSafetyExtensions.events = u_int64_t(frame->value.BasicSafetyMessage.partII->list.array[0]->partII_Value.VehicleSafetyExtensions.events);
    _VehicleSafetyExtensions.exteriorlights = u_int64_t(frame->value.BasicSafetyMessage.partII->list.array[0]->partII_Value.VehicleSafetyExtensions.lights);
    _pathHistorySets.latOffset = u_int64_t(frame->value.BasicSafetyMessage.partII->list.array[0]->partII_Value.VehicleSafetyExtensions.pathHistory->crumbData.list.array[0]->latOffset);
    _pathHistorySets.longOffset = u_int64_t(frame->value.BasicSafetyMessage.partII->list.array[0]->partII_Value.VehicleSafetyExtensions.pathHistory->crumbData.list.array[0]->lonOffset);
    _pathHistorySets.lelevOffset = u_int64_t(frame->value.BasicSafetyMessage.partII->list.array[0]->partII_Value.VehicleSafetyExtensions.pathHistory->crumbData.list.array[0]->elevationOffset);

//     core dumped code
//    std::cout << frame->value.BasicSafetyMessage.partII->list.count << std::endl;
//    _SpecialVehicleExtensions.lightbarInUse = u_int16_t(frame->value.BasicSafetyMessage.partII->list.array[0]->partII_Value.SpecialVehicleExtensions.vehicleAlerts->lightsUse);
//    _SpecialVehicleExtensions.typeEvent = u_int64_t(frame->value.BasicSafetyMessage.partII->list.array[0]->partII_Value.SpecialVehicleExtensions.vehicleAlerts->events);
//    _SpecialVehicleExtensions.description = u_int64_t(frame->value.BasicSafetyMessage.partII->list.array[0]->partII_Value.SpecialVehicleExtensions.description->description->list.array[0]);

//    ROS_INFO("========Test for BSM========");
//    std::cout << frame->value.BasicSafetyMessage.partII->list.array[0]->partII_Value.SpecialVehicleExtensions.vehicleAlerts->lightsUse << std::endl;
//    ROS_INFO("============================");
       if(frame->value.BasicSafetyMessage.partII->list.size == 0)
       {
           printf("there is no partII data!!!");
        
       }

       
//     _SupplementalVehicleExtensions.vehicleType = frame->value.BasicSafetyMessage.partII->list.array[1]->partII_Value.SupplementalVehicleExtensions.classDetails->vehicleType;
    _SupplementalVehicleExtensions.vehicleClass = *frame->value.BasicSafetyMessage.partII->list.array[1]->partII_Value.SupplementalVehicleExtensions.classification;
    



    _VehicleSafetyExtensions.pathHistorySets.push_back(_pathHistorySets);
    _partII.VehicleSafetyExtensions.push_back(_VehicleSafetyExtensions);
    _partII.SupplementalVehicleExtensions.push_back(_SupplementalVehicleExtensions);
    _partII.SpecialVehicleExtensions.push_back(_SpecialVehicleExtensions);
    _coredata.size.push_back(_size);
    _coredata.brakes.push_back(_brakes);
    _coredata.accelSet.push_back(_accelSet);
    _coredata.accuracy.push_back(_accuracy);
    _msg.partII.push_back(_partII);
    _msg.coreData.push_back(_coredata);

//    std::cout << _msg << std::endl;

    bsm_msg_pub_.publish(_msg);
}

static void decode_RTCMcorrections(MessageFrame_t *frame){

    wave_pkg::RTCM _msg;
    wave_pkg::FullPositionVector _anchorPoint;
    wave_pkg::RTCMheader _rtcmHeader;
    wave_pkg::RTCMmessageList_t _msgs;
    wave_pkg::AntennaOffsetSet_t _offsetSet;

    _msg.ui8_RtcmMsgs.clear();
    std::vector<uint8_t>().swap(_msg.ui8_RtcmMsgs);
    //containing wave messages in msg structures//

    _msg.msgCnt = frame->value.RTCMcorrections.msgCnt;
    _msg.rev    = frame->value.RTCMcorrections.rev;

    uint8_t rtcmbuf_[1024];
    std::cout << "sizeof(rtcmbuf_)" << std::endl << sizeof(rtcmbuf_) << std::endl;
    for(int i=0;i < sizeof(rtcmbuf_);i++)
    {
        rtcmbuf_[i] = NULL;
    }
    for(int i=0; i < frame->value.RTCMcorrections.msgs.list.array[0]->size;i++)
    {
        _msg.ui8_RtcmMsgs.push_back(frame->value.RTCMcorrections.msgs.list.array[0]->buf[i]);
        // printf("value_buf :: %02X \n" , frame->value.RTCMcorrections.msgs.list.array[0]->buf[i]);
    }

    //// need to convert hexcode to c code ////
    _rtcmHeader.offsetSet.push_back(_offsetSet);
    _msg.rtcmHeader.push_back(_rtcmHeader);
    _msg.msgs.push_back(_msgs);
    _msg.anchorPoint.push_back(_anchorPoint);

    rtcm_msg_pub_.publish(_msg);
}

static void decode_TravelerInformation(MessageFrame_t *frame){

    ROS_ERROR("start ! \n");
    wave_pkg::TIM _msg;
    wave_pkg::TravelerDataFrameList_t _dataFrames;
    wave_pkg::GeographicalPath _list;
    wave_pkg::TravelerDataFrame__content_PR _presentlist;
    wave_pkg::Position3D _anchor;
    wave_pkg::OffsetSystem_t _path;
    wave_pkg::NodeListLL_t _ll;


    printf("frame->value.dataframelist size :: %d \n" ,frame->value.TravelerInformation.dataFrames.list.count );


        for(int i =0; i< frame->value.TravelerInformation.dataFrames.list.count; i++)
        {
            if(frame->value.TravelerInformation.dataFrames.list.array[i]->priority)
            {
                _dataFrames.priority = u_int16_t(frame->value.TravelerInformation.dataFrames.list.array[i]->priority);
                _dataFrames.durationTime = u_int16_t(frame->value.TravelerInformation.dataFrames.list.array[i]->duratonTime);
                _dataFrames.startTime = u_int16_t(frame->value.TravelerInformation.dataFrames.list.array[i]->startTime);
                _dataFrames.startYear = u_int64_t(frame->value.TravelerInformation.dataFrames.list.array[i]->startYear);
                _dataFrames.frameType = u_int16_t(frame->value.TravelerInformation.dataFrames.list.array[i]->frameType);

                _anchor.lat = frame->value.TravelerInformation.dataFrames.list.array[i]->regions.list.array[0]->anchor->lat;
                _anchor.longitude = frame->value.TravelerInformation.dataFrames.list.array[i]->regions.list.array[0]->anchor->Long;
                _anchor.elevation = *frame->value.TravelerInformation.dataFrames.list.array[i]->regions.list.array[0]->anchor->elevation;
                _list.laneWidth = *frame->value.TravelerInformation.dataFrames.list.array[i]->regions.list.array[0]->laneWidth;

                _list.anchor.clear();
                _dataFrames.list.clear();
                _dataFrames.presentlist.clear();
                _path.ll.clear();


                for(int j= 0; j< frame->value.TravelerInformation.dataFrames.list.array[i]->regions.list.array[0]->description->path.offset.xy.nodes.list.count; j++)
                {
                    _list.path.clear();

                    _ll.longitude = frame->value.TravelerInformation.dataFrames.list.array[i]->regions.list.array[0]->description->path.offset.xy.nodes.list.array[j]->delta.node_LatLon.lon;
                    _ll.lat = frame->value.TravelerInformation.dataFrames.list.array[i]->regions.list.array[0]->description->path.offset.xy.nodes.list.array[j]->delta.node_LatLon.lat;

                    _path.ll.push_back(_ll);
                    _list.path.push_back(_path);

                }


                char zone[4];


                if(frame->value.TravelerInformation.dataFrames.list.array[i]->content.speedLimit.list.array[0]->item.text.buf[0] =='s')
                {
                    char school[2];

                    _presentlist.type.clear();
                    _presentlist.value.clear();
                    school[0] = frame->value.TravelerInformation.dataFrames.list.array[i]->content.speedLimit.list.array[0]->item.text.buf[0];
                    school[1] = frame->value.TravelerInformation.dataFrames.list.array[i]->content.speedLimit.list.array[0]->item.text.buf[1];

                    zone[0] = frame->value.TravelerInformation.dataFrames.list.array[i]->content.speedLimit.list.array[0]->item.text.buf[3];
                    zone[1] = frame->value.TravelerInformation.dataFrames.list.array[i]->content.speedLimit.list.array[0]->item.text.buf[4];

                    std::string _std(school);
                    std::string _str(zone);

                    _presentlist.type = _std;
                    _presentlist.value = _str;

                }

                if(frame->value.TravelerInformation.dataFrames.list.array[i]->content.workZone.list.array[0]->item.text.buf[0] == 'w')
                {
                    char workzone[2];

                    _presentlist.type.clear();
                    _presentlist.value.clear();

//            std::cout << "workzone buf :: " << frame->value.TravelerInformation.dataFrames.list.array[i]->content.workZone.list.array[0]->item.text.buf[0] << std::endl;
//            std::cout << "workzone buf :: " << frame->value.TravelerInformation.dataFrames.list.array[i]->content.workZone.list.array[0]->item.text.buf[1] << std::endl;
//            std::cout << "workzone buf :: " << frame->value.TravelerInformation.dataFrames.list.array[i]->content.workZone.list.array[0]->item.text.buf[2] << std::endl;
//            std::cout << "workzone buf :: " << frame->value.TravelerInformation.dataFrames.list.array[i]->content.workZone.list.array[0]->item.text.buf[3] << std::endl;

                    workzone[0] = frame->value.TravelerInformation.dataFrames.list.array[i]->content.workZone.list.array[0]->item.text.buf[0];
                    workzone[1] = frame->value.TravelerInformation.dataFrames.list.array[i]->content.workZone.list.array[0]->item.text.buf[1];

//            for(int k = 0 ; k< frame->value.TravelerInformation.dataFrames.list.array[i]->content.workZone.list.array[0]->item.text.size; k++)
//            {
                    zone[0] = frame->value.TravelerInformation.dataFrames.list.array[i]->content.workZone.list.array[0]->item.text.buf[3];
                    zone[1] = frame->value.TravelerInformation.dataFrames.list.array[i]->content.workZone.list.array[0]->item.text.buf[5];

//            }
                    std::string _std(workzone);
                    std::string _str(zone);

                    _presentlist.type = _std;
                    _presentlist.value = _str;
                }



                _list.anchor.push_back(_anchor);
                _dataFrames.presentlist.push_back(_presentlist);
                _dataFrames.list.push_back(_list);

                _msg.dataFrames.push_back(_dataFrames);


            }
            else
            {
                ROS_ERROR("no message!\n");
                break;
            }

        }

    _msg.msgCnt = u_int16_t(frame->value.TravelerInformation.msgCnt);

    tim_msg_pub_.publish(_msg);

    ROS_ERROR("end ! \n");

}

static int decode_MessageFrame(const uint8_t *buf, int len){
    MessageFrame_t *frame = NULL;
    asn_dec_rval_t rval;

    rval = uper_decode_complete(NULL, &asn_DEF_MessageFrame, (void **)&frame, buf, len);
    if (rval.code != RC_OK) {
        fprintf(stderr, "failed to decode MessageFrame\n");
        fprintf(stdout, "================================================\n");
        ASN_STRUCT_FREE(asn_DEF_MessageFrame, frame);
        return -1;
    }


    //show message decoded in console

//       fprintf(stdout, "------------------------------------------------\n");
//       fprintf(stdout, "decoded %d, code: %d\n", (int)rval.consumed, rval.code);
//       fprintf(stdout, "------------------------------------------------\n");
//       asn_fprint(stdout, &asn_DEF_MessageFrame, frame);
//       fprintf(stdout, "================================================\n");



    //find the code type to publish in ros

    if (frame->messageId == J2735_UPER_MAP &&
            frame->value.present == MessageFrame__value_PR_MapData) {
            decode_MapData(frame);
        // fprintf(stdout, "------------------------------------------------\n");
        // fprintf(stdout, "decoded %d, code: %d\n", (int)rval.consumed, rval.code);
        // fprintf(stdout, "------------------------------------------------\n");
        // asn_fprint(stdout, &asn_DEF_MessageFrame, frame);
        // fprintf(stdout, "================================================\n");

    }
    else if (frame->messageId == J2735_UPER_SPAT &&
            frame->value.present == MessageFrame__value_PR_SPAT) {
            decode_SignalPhaseAndTimingMessage(frame);
        // fprintf(stdout, "------------------------------------------------\n");
        // fprintf(stdout, "decoded %d, code: %d\n", (int)rval.consumed, rval.code);
        // fprintf(stdout, "------------------------------------------------\n");
        // asn_fprint(stdout, &asn_DEF_MessageFrame, frame);
        // fprintf(stdout, "================================================\n");

    }
    else if (frame->messageId == J2735_UPER_BSM &&
            frame->value.present == MessageFrame__value_PR_BasicSafetyMessage) {
            decode_BasicSafetyMessage(frame);
       // fprintf(stdout, "------------------------------------------------\n");
       // fprintf(stdout, "decoded %d, code: %d\n", (int)rval.consumed, rval.code);
       // fprintf(stdout, "------------------------------------------------\n");
       // asn_fprint(stdout, &asn_DEF_MessageFrame, frame);
       // fprintf(stdout, "================================================\n");
    }
    else if (frame->messageId == J2735_UPER_RTCM &&
            frame->value.present == MessageFrame__value_PR_RTCMcorrections) {
            decode_RTCMcorrections(frame);
//        fprintf(stdout, "------------------------------------------------\n");
//        fprintf(stdout, "decoded %d, code: %d\n", (int)rval.consumed, rval.code);
//        fprintf(stdout, "------------------------------------------------\n");
//        asn_fprint(stdout, &asn_DEF_MessageFrame, frame);
//        fprintf(stdout, "================================================\n");
    }
    else if (frame->messageId == J2735_UPER_TIM &&
            frame->value.present == MessageFrame__value_PR_TravelerInformation) {
            decode_TravelerInformation(frame);
          fprintf(stdout, "------------------------------------------------\n");
          fprintf(stdout, "decoded %d, code: %d\n", (int)rval.consumed, rval.code);
          fprintf(stdout, "------------------------------------------------\n");
          asn_fprint(stdout, &asn_DEF_MessageFrame, frame);
          fprintf(stdout, "================================================\n");
    }
    ASN_STRUCT_FREE(asn_DEF_MessageFrame, frame);
    return rval.code;
}

static int decode_avc2019(const uint8_t *buf, int len){
    if (len < sizeof(AVC_HDR_t)) {
        fprintf(stderr, "packet size is less than header\n");
        return -1;
    }
    //printf("%d\n",sizeof(&buf));
    //printf("%d\n",len);
    //the header part of the message
    const AVC_HDR_t *hdr = (AVC_HDR_t *)buf;  

    uint32_t sec  = ntohl(hdr->sec);
    uint32_t usec = ntohl(hdr->usec);
    uint32_t type = ntohl(hdr->type);
    uint32_t dlen = ntohl(hdr->len);

    //show the header part information

     // fprintf(stdout, "================================================\n");
     // fprintf(stdout, "time: %u.%06u, type: %2u, len: %4u\n", sec, usec, type, dlen);

    //entire length is header part length + data length + tail length
    //compare them to find if the part of packet lost

    if (dlen + sizeof(AVC_HDR_t) + sizeof(AVC_TAIL_t) != len) {
        fprintf(stderr, "packet size is less than data: %lu %u\n",
                dlen + sizeof(AVC_HDR_t), len);
        fprintf(stdout, "================================================\n");
        return -1;
    }

    //call decode_MessageFrame to show decoded message
    //printf("AVC_HDR_t size : %d\n",sizeof(AVC_HDR_t));
    return decode_MessageFrame(buf + sizeof(AVC_HDR_t), dlen);
}

static void signal_handler(int signum){
    fprintf(stdout, "\n");
    RUNNING = 0;
}

int init()
{
    //node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
    pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    //tim_msg_pub_ = node_->advertise<wave_pkg::TIM>("/tim_message", 1);
    //spat_msg_pub_ = node_->advertise<SPAT::SPAT__regional>("/spat_message", 1);
    //map_msg_pub_ = node_->advertise<wave_pkg::MAP>("/map_message", 1);
//  bsm_msg_pub_ = node_->advertise<wave_pkg::BSM>("/bsm_message", 1);
//  rtcm_msg_pub_ = node_->advertise<wave_pkg::RTCM>("/rtcm_message",1);
    //rtcm_msg_pub_ = node_->advertise<RTCMcorrections::RTCMcorrections__regional>("/rtcm_message", 1);


}

int main(int argc, char **argv){


    ros::init(argc, argv, "dec_node");

    node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
    pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    tim_msg_pub_ = node_->advertise<wave_pkg::TIM>("/wave/tim_message", 1);
    spat_msg_pub_ = node_->advertise<wave_pkg::SPAT>("/wave/spat_message", 1);
    map_msg_pub_ = node_->advertise<wave_pkg::MAP>("/wave/map_message", 1);
    bsm_msg_pub_ = node_->advertise<wave_pkg::BSM>("/wave/bsm_message", 1);
    rtcm_msg_pub_ = node_->advertise<wave_pkg::RTCM>("/wave/rtcm_message",1);
//  ros::Rate loop_rate(10);

    ROS_ERROR("gggggggggggggggggg");

    //udp receiver code

    struct timeval tv;
    fd_set fds;

    struct sockaddr_in sockaddr;
    int sock = -1;
    int ret = -1;
    int on = 1;

    uint8_t buf[512];
    int nrecv;

    //socket port num is 60000
    //get data from any address

    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(60000);
    sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (sock == -1) {
        fprintf(stderr, "failed to create socket\n");
        return EXIT_FAILURE;
    }

    ret = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    if (ret == -1) {
        fprintf(stderr, "failed to set socket option: %d\n", errno);
        close(sock);
        return EXIT_FAILURE;
    }

    //bind socket to get information from server.

    ret = bind(sock, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (ret == -1) {
        fprintf(stderr, "failed to bind socket: %d\n", errno);
        close(sock);
        return EXIT_FAILURE;
    }
    while (ros::ok()) {

    signal(SIGINT, signal_handler);


        FD_ZERO(&fds);
        FD_SET(sock, &fds);

        tv.tv_sec  = 0;
        tv.tv_usec = 1000; /* 1ms */

        ret = select(sock + 1, &fds, NULL, NULL, &tv);
        if (ret == -1) {
            if (errno == EINTR)
                continue;
            fprintf(stderr, "failed to select socket: %d\n", errno);
            close(sock);
            return EXIT_FAILURE;
        }
        else if (ret == 0)
            continue;
        static int counter = 1;
        if (FD_ISSET(sock, &fds)) {
            nrecv = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);

            if (nrecv == -1) {
                fprintf(stderr, "failed to recvfrom: %d\n", nrecv);
                printf(" =============  FAIL TIME  :  %d\n",counter++);
                //close(sock);
                //return EXIT_FAILURE;
            }

            ret = decode_avc2019(buf, nrecv);

            if (ret < 0) {
                printf(" =============  FAIL TIME  :  %d\n",counter++);
                fprintf(stderr, "failed to decode: %d\n", ret);
                //close(sock);
                //return EXIT_FAILURE;
            }
        }
    }
    ros::spin();

    close(sock);
    return EXIT_SUCCESS;
}


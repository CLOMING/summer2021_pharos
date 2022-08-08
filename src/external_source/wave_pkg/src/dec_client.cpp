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
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <dirent.h>
#include <sys/mman.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <IntersectionGeometryList.h>
#include <TravelerDataFrameList.h>
#include <TravelerDataFrame.h>

#include "MessageFrame.h"

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

    //containing wave messages in msg structures//
    //_id.id = frame->value.MapData.intersections->list.array[0]->id.id;


    _nodeList.delta.push_back(_delta);
    _nodeList.attributes.push_back(_attributes);
    _laneSet.connectsTo.push_back(_connectsTo);
    _laneSet.nodeList.push_back(_nodeList);
    _laneSet.laneAttributes.push_back(_laneAttributes);
    _intersectionGeometryList.id.push_back(_id);
    _intersectionGeometryList.refPoint.push_back(_refPoint);
    _intersectionGeometryList.laneSet.push_back(_laneSet);
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


    _state_time_speed.timing.push_back(_timing);
    _states.state_time_speed.push_back(_state_time_speed);
    _intersections.states.push_back(_states);
    _intersections.id.push_back(_id);
    _msg.intersections.push_back(_intersections);



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

    //containing wave messages in msg structures//


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


    bsm_msg_pub_.publish(_msg);
}

static void decode_RTCMcorrections(MessageFrame_t *frame){

    wave_pkg::RTCM _msg;
    wave_pkg::FullPositionVector _anchorPoint;
    wave_pkg::RTCMheader _rtcmHeader;
    wave_pkg::RTCMmessageList_t _msgs;
    wave_pkg::AntennaOffsetSet_t _offsetSet;


    //containing wave messages in msg structures//


    _rtcmHeader.offsetSet.push_back(_offsetSet);
    _msg.rtcmHeader.push_back(_rtcmHeader);
    _msg.msgs.push_back(_msgs);
    _msg.anchorPoint.push_back(_anchorPoint);


    rtcm_msg_pub_.publish(_msg);
}

static void decode_TravelerInformation(MessageFrame_t *frame){

    wave_pkg::TIM _msg;
    wave_pkg::TravelerDataFrameList_t _dataFrames;
    wave_pkg::GeographicalPath _list;
    wave_pkg::TravelerDataFrame__content_PR _presentlist;
    wave_pkg::Position3D _anchor;
    wave_pkg::OffsetSystem_t _path;
    wave_pkg::NodeListLL_t _ll;


    //containing wave messages in msg structures//
    //for(int i=0; frame->value.TravelerInformation.dataFrames.list.size; i++)
    //{
        _anchor.lat = frame->value.TravelerInformation.dataFrames.list.array[0]->regions.list.array[0]->anchor->lat;
        _anchor.longitude = frame->value.TravelerInformation.dataFrames.list.array[0]->regions.list.array[0]->anchor->Long;
    //}

    _path.ll.push_back(_ll);
    _list.path.push_back(_path);
    _list.anchor.push_back(_anchor);
    _dataFrames.presentlist.push_back(_presentlist);
    _dataFrames.list.push_back(_list);
    _msg.dataFrames.push_back(_dataFrames);

    tim_msg_pub_.publish(_msg);
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

    fprintf(stdout, "------------------------------------------------\n");
    fprintf(stdout, "decoded %d, code: %d\n", (int)rval.consumed, rval.code);
    fprintf(stdout, "------------------------------------------------\n");
    asn_fprint(stdout, &asn_DEF_MessageFrame, frame);
    fprintf(stdout, "================================================\n");



    //find the code type to publish in ros

    if (frame->messageId == J2735_UPER_MAP &&
            frame->value.present == MessageFrame__value_PR_MapData) {
            decode_MapData(frame);

    }
    else if (frame->messageId == J2735_UPER_SPAT &&
            frame->value.present == MessageFrame__value_PR_SPAT) {
            decode_SignalPhaseAndTimingMessage(frame);
    }
    else if (frame->messageId == J2735_UPER_BSM &&
            frame->value.present == MessageFrame__value_PR_BasicSafetyMessage) {
            decode_BasicSafetyMessage(frame);
    }
    else if (frame->messageId == J2735_UPER_RTCM &&
            frame->value.present == MessageFrame__value_PR_RTCMcorrections) {
            decode_RTCMcorrections(frame);
    }
    else if (frame->messageId == J2735_UPER_TIM &&
            frame->value.present == MessageFrame__value_PR_TravelerInformation) {
            decode_TravelerInformation(frame);
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

    fprintf(stdout, "================================================\n");
    fprintf(stdout, "time: %u.%06u, type: %2u, len: %4u\n", sec, usec, type, dlen);

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


    ros::init(argc, argv, "dec_client_node");

    node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
    pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    tim_msg_pub_ = node_->advertise<wave_pkg::TIM>("/tim_message", 1);
    spat_msg_pub_ = node_->advertise<wave_pkg::SPAT>("/spat_message", 1);
    map_msg_pub_ = node_->advertise<wave_pkg::MAP>("/map_message", 1);
    bsm_msg_pub_ = node_->advertise<wave_pkg::BSM>("/bsm_message", 1);
    rtcm_msg_pub_ = node_->advertise<wave_pkg::RTCM>("/rtcm_message",1);
//  ros::Rate loop_rate(10);

    ROS_ERROR("===== UDP Client Wave Communication Program Start =====");

    //udp receiver code

    struct timeval tv;
    fd_set fds;
    struct sockaddr_in fromAddr;
    struct sockaddr_in sockaddr;
    int sock = -1;
    int ret = -1;
    int on = 1;
    int recvSocket;
    uint8_t buf[4096];
    int nrecv;

    int portNumber;
    in_addr_t ipAddress;

    //socket port num is 60000
    //get data from any address
    std::string ipaddress = "192.168.1.100";
    struct hostent *host = gethostbyname(ipaddress.c_str());
    ipAddress=*((in_addr_t *)(host->h_addr));


    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(60000);
    sockaddr.sin_addr.s_addr = ipAddress;

    socklen_t len=sizeof(fromAddr);

    sock = socket(PF_INET, SOCK_DGRAM, 0); // IPPROTO_UDP -> 0
    if (sock == -1) {
        fprintf(stderr, "failed to create socket\n");
        return EXIT_FAILURE;
    }/*
    //bind socket to ge b t information from server.
    ret = bind(sock, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    
    if (ret == -1) {
        fprintf(stderr, "failed to bind socket: %d\n", errno);
        close(sock);
        return EXIT_FAILURE;
    }
    */
    //listen(sock, 5);
    signal(SIGINT, signal_handler);



    while (ros::ok()) {
        
        
            socklen_t fromAddr_size = sizeof(fromAddr);
            //recvSocket = accept(sock, (struct sockaddr *)&fromAddr, &len);
            nrecv = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&fromAddr,&fromAddr_size);

            if (nrecv == -1) {
                fprintf(stderr, "failed to recvfrom: %d\n", nrecv);
                close(sock);
                return EXIT_FAILURE;
            

            ret = decode_avc2019(buf, nrecv);

            if (ret < 0) {
                fprintf(stderr, "failed to decode: %d\n", ret);
                close(sock);
                return EXIT_FAILURE;
            }
        }
    }
    ros::spin();

    close(sock);
    return EXIT_SUCCESS;
}



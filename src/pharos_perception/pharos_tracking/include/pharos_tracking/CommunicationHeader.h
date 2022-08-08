#include <wave_tcp/BasicSafetyMessage_DATA.h>
#include <wave_tcp/BasicSafetyMessage_DATA_List.h>
#include <wave_tcp/CallMatching_CONFIRM.h>
#include <wave_tcp/CallRequestList_DATA.h>
#include <wave_tcp/CallSelect_RES.h>
#include <wave_tcp/GettingOffComplete_CONFIRM.h>
#include <wave_tcp/GettingOffComplete_REQ.h>
#include <wave_tcp/GettingOnComplete_CONFIRM.h>
#include <wave_tcp/GettingOnComplete_REQ.h>
#include <wave_tcp/GPS_DATA.h>
#include <wave_tcp/MAP_DATA.h>
#include <wave_tcp/MAP_DATA_List.h>
#include <wave_tcp/Registration_REQ.h>
#include <wave_tcp/Registration_RES.h>
#include <wave_tcp/RoadSideAlert_DATA.h>
#include <wave_tcp/RoadSideAlert_DATA_List.h>
#include <wave_tcp/SinalViolation_Alert.h>
#include <wave_tcp/SPAT_DATA.h>
#include <wave_tcp/SPAT_DATA_List.h>
#include <wave_tcp/SelecConfirmCallDataList.h>

#include <wave_tcp/Payload_BasicSafetyMessage_DATA.h>
#include <wave_tcp/Payload_CallMatching_CONFIRM.h>
#include <wave_tcp/Payload_CallRequestList_DATA.h>
#include <wave_tcp/Payload_CallSelect_RES.h>
#include <wave_tcp/Payload_GettingOffComplete_CONFIRM.h>
#include <wave_tcp/Payload_GettingOffComplete_REQ.h>
#include <wave_tcp/Payload_GettingOnComplete_CONFIRM.h>
#include <wave_tcp/Payload_GettingOnComplete_REQ.h>
#include <wave_tcp/Payload_GPS_Position_Accuracy.h>
#include <wave_tcp/Payload_MAP_DATA.h>
#include <wave_tcp/Payload_Registration_REQ.h>
#include <wave_tcp/Payload_Registration_RES.h>
#include <wave_tcp/Payload_RoadSideAlert_DATA.h>
#include <wave_tcp/Payload_SinalViolation_Alert.h>
#include <wave_tcp/Payload_SPAT_DATA.h>

#include <wave_tcp/WaveHeader.h>
#include <wave_tcp/Acceleration_Set_4Way.h>
#include <wave_tcp/BSMcoreData.h>
#include <wave_tcp/Call_Data.h>
#include <wave_tcp/GPS_Position.h>
#include <wave_tcp/GPS_Position_Accuracy.h>
#include <wave_tcp/intersectionGeometry_Basic_info.h>
#include <wave_tcp/intersectionGeometry_Lanelist.h>
#include <wave_tcp/Intersection_info.h>
#include <wave_tcp/IntersectionStateList_basic_info.h>
#include <wave_tcp/IntersectionStateList_Movementlist.h>
#include <wave_tcp/LaneList.h>
#include <wave_tcp/MovementEventList.h>
#include <wave_tcp/Passenger_Position.h>
#include <wave_tcp/Position_Accuracy.h>
#include <wave_tcp/RoadSideAlert.h>
#include <wave_tcp/Select_Call_Data_List.h>
#include <wave_tcp/SpeedLimitList.h>
#include <wave_tcp/Time_Stamp.h>
#include <wave_tcp/VehicleEventFlags.h>
#include <wave_tcp/Vehicle_Position.h>
#include <wave_tcp/Vehicle_Position_Confirm.h>
#include <wave_tcp/Vehicle_Position_REQ.h>
#include <wave_tcp/Vehicle_Postion.h>
#include <wave_tcp/VehicleSafetyExtensions.h>
#include <wave_tcp/Vehicle_Size.h>
#include <wave_tcp/Vehicle_Status.h>

ros::Publisher BasicSafetyMessage_DATA_pub;
ros::Publisher CallMatching_Confirm_pub;
ros::Publisher CallRequestList_DATA_pub;
ros::Publisher CallSelect_RES_pub;
ros::Publisher GettingOffComplete_CONFIRM_pub;
ros::Publisher GettingOffComplete_REQ_pub;
ros::Publisher GettingOnComplete_CONFIRM_pub;
ros::Publisher GettingOnComplete_REQ_pub;
ros::Publisher GPS_DATA_pub;
ros::Publisher MAP_DATA_pub;
ros::Publisher Registration_REQ_pub;
ros::Publisher Registration_RES_pub;
ros::Publisher RoadSideAlert_DATA_pub;
ros::Publisher SinalViolation_Alert_pub;
ros::Publisher SPAT_DATA_pub;
ros::Publisher MissionStatus_pub;

ros::Subscriber CallSelect_RES_sub_;
ros::Subscriber GettingOn_REQ_sub_;
ros::Subscriber GettingOff_REQ_sub_;
//Communication Status
#define Registration_Send 0x01
#define Registration_Recv 0x02
#define Free_Data_Send_Recv 0x03

// Packet Type
#define Registration_REQ_0 0xD1
#define Registration_REQ_1 0x20

#define Registration_RES_0 0xA1
#define Registration_RES_1 0x00

#define GPS_DATA_0 0xD2
#define GPS_DATA_1 0x20

#define BasicSafetyMessage_DATA_0 0xA2
#define BasicSafetyMessage_DATA_1 0x00

#define SPAT_DATA_0 0xA3
#define SPAT_DATA_1 0x00

#define MAP_DATA_0 0xA4
#define MAP_DATA_1 0x00

#define RoadSideAlert_DATA_0 0xA5
#define RoadSideAlert_DATA_1 0x00

#define CallRequestList_DATA_0 0xA6
#define CallRequestList_DATA_1 0x00

#define Callselect_RES_0 0xD3
#define Callselect_RES_1 0x20

#define CallMatching_CONFIRM_0 0xA7
#define CallMatching_CONFIRM_1 0x00

#define GettingOnComplete_REQ_0 0xD4
#define GettingOnComplete_REQ_1 0x20

#define GettingOnComplete_CONFIRM_0 0xA8
#define GettingOnComplete_CONFIRM_1 0x00

#define GettingOffComplete_REQ_0 0xD5
#define GettingOffComplete_REQ_1 0x20

#define GettingOffComplete_CONFIRM_0 0xA9
#define GettingOffComplete_CONFIRM_1 0x00

#define SinalViolation_Alert_0 0xAA
#define SinalViolation_Alert_1 0x00

#define MissionDrop_DATA_0 0xAB
#define MissionDrop_DATA_1 0x00

// Device Type
#define TYPE_AutonomySystem 0xBE
#define TYPE_AcquistionTerminal 0xBA
#define TYPE_OBU 0xBD
#define TYPE_RSU 0xCD
#define TYPE_V2XServer 0xAC
#define TYPE_ControlServer 0xAF
#define TYPE_ControllerTerminal 0xAF
#define TYPE_Pedestrian 0x1F
#define TYPE_Bicycle 0x2F
#define TYPE_Bus 0x3F
#define TYPE_Taxi 0x4F

// Device Prefix
#define PREFIX_AutonomySystem 0x2
#define PREFIX_AcquistionTerminal 0x0
#define PREFIX_OBU 0xC
#define PREFIX_RSU 0xD
#define PREFIX_V2XServer 0xA
#define PREFIX_ControlServer 0xF
#define PREFIX_ControllerTerminal 0x1


// Registration Status
#define REGISTRATION_SUCCESS 0x01
#define REGISTRATION_FAIL 0x02

// ERROR CODE
#define NONE 0x00
#define ERROR_PAYLOAD_LEN 0x01
#define ERROR_DEVICE_TYPE 0x02

// Transmission State
#define neutral 0x00
#define park 0x01
#define forwardGears 0x02
#define reverseGears 0x03
#define Ts_reserved1 0x04
#define Ts_reserved2 0x05
#define Ts_reserved3 0x06
#define TS_unavailable 0x07

// Parking Brake
#define PB_unavaiable 0x00
#define PBoff 0x01
#define OBon 0x02
#define PB_reserved 0x03

// Vehicle Safety Extension
#define GetOnDown 0x01
#define Trouble 0x02
#define Harddeceleration 0x04
#define HardStop 0x08
#define HardTurn 0x10
#define UTurn 0x20

// Crossload Status Information
#define manualControllsEnabled 0x01
#define stopTimelsActivated 0x02
#define failureFlash 0x03
#define preemptlsActive 0x04
#define signalPrioritylsActive 0x05
#define fixedTimeOperation 0x06
#define trafficDependentOperation 0x07
#define standbyOperation 0x08
#define failureMode 0x09
#define CSIoff 0x0A
#define recentMAPmessageUpdate 0x0B
#define recentChangeInMAPassignedLanesIDsUsed 0x0C
#define noValidMAPisAvailableAtThisTime 0x0D
#define noValidSPATisAvailableAtThisTime 0x0E 

// MovementPhaseState
#define MPSunavailable 0x00
#define dark 0x01
#define stop_Then_Proceed 0x02
#define stop_AndRemain 0x03
#define pre_Movement 0x04
#define permissive_Movement_Allowed 0x05
#define protected_Movement_Allowed 0x06
#define permissive_clearance 0x07
#define protected_clearance 0x08
#define caution_Conflicting_Traffic 0x09


// MAP DATA
#define MDunknown 0x00
#define maxSpeedInSchoolZone 0x01
#define WhenChildrenArePresent 0x02
#define maxSpeedInConstructionZone 0x03
#define vehicleMinSpeed 0x04
#define vehicleMaxSpeed 0x05
#define vehicleNightMaxSpeed 0x06
#define truckMinSpeed 0x07
#define truckMaxSpeed 0x08
#define truckNightMaxSpeed 0x09
#define vehiclesWithTrailersMinSpeed 0x0A
#define vehiclesWithTrailersMaxSpeed 0x0B
#define vehiclesWithTrailersNightMaxSpeed 0x0C

// MAP DATA - LaneAttributes - LaneDirection
#define ingressPath 0x01
#define egressPath 0x02 

// MAP DATA - LaneAttributes - LaneSharingle
#define otherNonMotorizedTrafficTypes 0x01
#define individualMotorizedVehicleTraffic 0x02
#define busVehicleTraffic 0x04
#define taxiVehicleTraffic 0x08
#define pedestriansTraffic 0x10
#define cyclistVehicleTraffic 0x20
#define trackedVehicleTraffic 0x40

// MAP DATA - LaneTypeAttributes-Vehicle
#define isVehiclerevocableLane 0x01
#define isVehicleflyOverLane 0x02
#define hovLaneUseOnly 0x04
#define restrictedToBusUse 0x08
#define restrictedToTaxiUse 0x10
#define restrictedFromPublicUse 0x20
#define hasIRbeaconCoverage 0x40
#define permissionOnRequest 0x80

// AllowedManeuvers
#define maneuverStraightAllowed 0x01
#define maneuverLeftAllowed 0x02
#define maneuverRightAllowed 0x03
#define maneuverUTurnAllowed 0x04
#define maneuverLeftTurnOnRedAllowed 0x05
#define maneuverRightTurnOnRedAllowed 0x06
#define maneuverLaneChangeAllowed 0x07
#define maneuverNoStoppingAllowed 0x08
#define yieldAllwaysRequired 0x09
#define goWithHalt 0x0A
#define caution 0x0B
#define AMreserved1 0x0C

// RSA (Road Side Alert Message)
#define stop_and_go_traffic 258
#define Serious_accident 514
#define minor_accident 517
#define diable_vehicle 534
#define overturned_vehicle 554
#define Road_maintenance_operations 1036
#define obstruction_on_roadway 1281
#define vehicle_traveling_wrong_way 1793
#define slow_vehicle 2052
#define heavy_snow 4867
#define heavy_rain 4884
#define strong_winds 5127
#define dense_fog 5377
#define surface_water_hazard 5893
#define pedestrians 9486
#define n300 11533

// RSA - HeadingSlice
#define from000_0to022_5degrees 0x01
#define from022_5to045_0degrees 0x02
#define from045_0to067_5degrees 0x03
#define from067_5to090_0degrees 0x04
#define from090_0to112_5degrees 0x05
#define from112_5to135_0degrees 0x06
#define from135_0to157_5degrees 0x07
#define from157_5to180_0degrees 0x08
#define from180_0to202_5degrees 0x09
#define from202_5to225_0degrees 0x0A
#define from225_0to247_5degrees 0x0B
#define from247_5to270_0degrees 0x0C
#define from270_0to292_5degrees 0x0D
#define from292_5to315_0degrees 0x0E
#define from315_0to337_5degrees 0x0F
#define from337_5to360_0degrees 0x10

// Send Msg
// Call Data - Call Status
#define Wait_State 0x00
#define Accept_State 0x01

// Recv Msg
// Call Data - Error Status
#define ERROR_NONE 0x00
#define ERROR_CALL_LIST_RECV_TIMEOUT 0x01
#define ERROR_CALL_MACH_RECV_TIMEOUT 0x02
#define ERROR_PAYLOAD_VALIDATION 0x03
#define STATUS_CALL_MATCH_NON_RECV 0x10

// Confirm Data - Byte Number 0 - Error Status
#define ERROR_CALL_SELECT_SEND_INTV 0x01
#define ERROR_CALL_SELECT_SEND 0x02
#define ERROR_PASSENGER_NUM_EXCESS 0x03
#define ERROR_CALL_NUM_EXCESS 0x04
#define ERROR_PAYLOAD_VALIDATION 0x05
#define ERROR_CALL_SELECT_FAIL 0x06
#define ERROR_CALL_ID 0x07

// Confirm Data - Byte Number 1 - Error Status
#define ERROR_CALL_LIST_ID_FIRST 0x01
#define ERROR_CALL_LIST_ID_SECOND 0x02
#define ERROR_CALL_LIST_ID_THIRD 0x04


// GettingONComplete_REQ - Error Status
#define NONE 0x00
#define ERROR_CONFIRM_RECV_TIMEOUT 0x01
#define ERROR_PAYLOAD_LEN 0x03
#define STATUS_CONFIRM_NON_RECV 0x10

// GettingOnComplete_CONFIRM -  Error Status
#define ERROR_REQ_REV_INTV 0x01
#define ERROR_REQ_REV 0x02
#define ERROR_RANGE_EXCESSS 0x03
#define ERROR_NONE_STOP_EXCESS 0x04
#define ERROR_PAYLOAD_VALIDATION 0x05 

// GettingOffComplete_REQ - Error Status
#define None 0x00
////// Same GettingONComplete_REQ - Error Status

// GettingOffComplete_CONFIRM - Error Status
////// Same GettingOnComplete_CONFIRM - Error Status



#pragma pack(push,1)


//// Child Messages
struct NewDataChecking
{
	bool GPS_DATA;
};

struct Header_
{
    unsigned char Packet_Type[2];
    unsigned char Current_Sequence;
    unsigned short Payload_Size;
    unsigned char Device_Type;
};

struct Time_Stamp_
{
    unsigned int MinuteOfTheYear;                          //0~527040, 527040 is not validable, present UTC year's minute
    unsigned short DSecond;                                //0~65535, when message generated UTX year's mSec(millisecond)
};

struct GPS_Position_
{
	int Latitude;
	int Longitude;
	int Elevation;
};

struct Position_Accuracy_
{
    unsigned char semiMajor;
    unsigned char semiMinor;
    unsigned short orientation;
};

struct Acceleration_Set_4Way_
{
    short Latitude_Acceleration;            //Latitude accelation unit 0.01m/s^2
    short Longitude_Acceleration;           //longitude accelation unit 0.01m/s^2
    short YawRate;                        //yawrate unit 0.01deg/sec
};

struct Vehicle_Size_
{
    unsigned short VehicleWidth;                   //car width
    unsigned short VehicleLength;                   //car length
};

struct PassengerPosition_
{
    int Latitude;
    int Longitude;
};

struct DestinationPosition_
{
    int Latitude;
    int Longitude;
};

struct VehiclePosition_
{
    int Latitude;
    int Longitude;
};

struct VehicleEventFlags_
{
    unsigned char eventGetOnDown;         //hex Value 0x01, case, vehicle's door open when car speed under 4km/h
    unsigned char eventTrouble;           //hex Value 0x02, case, RPM >0, throttle>0, gear rate >1, not using brake, vehicle speed < 4km/h
    unsigned char eventHardDeceleration;  //hex value 0x04, case, without using brake, car speed decelerate 14km/h
    unsigned char eventHardStop;          //hex value 0x08, case, using brake, car speed decelerate 14km/h until car speed 0.
    unsigned char eventHardTurn;           //hex Value 0x10, case, maintain car speed higher than 15km/h, and head change left or right over 45deg
    unsigned char eventUTurn;             //hex Value 0x20, case, maintain car speed higher than 15km/h, and head change left over 60deg
    unsigned char eventRoadWork;          //
    unsigned char reserved;               //
};

struct SpeedLimitList_
{
    unsigned char SpeedLimitType;                          //0~16 2)page 73
    unsigned short Velocity;                               //0~8191(unit of 0.02m/s), reference speed, refer this before provide new speed
};

struct LaneList_
{
    char LaneID[14];                                             //ascii character
    unsigned char LaneDirection;                  //0~1, hex Value 0x01 lane back to front, 0x02 lane front to back
    unsigned char LaneSharing[7];                 //sharing car in lane
    unsigned char LaneTypeAttributes_vehicle[8];  //
    unsigned char AllowedManeuvers[12];
    char FirstLaneID[14];                           //ascii character
    char SecondLaneID[14];                           //ascii character
    unsigned char signalGroupID;                  //0~255
};

struct BSMcoreData_
{
    unsigned char MsgCount;                         //Message sequence
    unsigned char Temporary_ID[4];                  // = Device ID
    unsigned short DSecond;                          // = SecMark
    GPS_Position_ GPS_Position;
    Position_Accuracy_ Position_Accuracy;
    unsigned char TransmissionState;                            //car Transmissionstate 0x00 neutral 0x01 park 0x02 forwardGears 0x03 reverseGears
    unsigned short Speed;                                        //Vehicle speed, CAN data, unit 0.02m/s
    unsigned short Heading;                                      //refer to NMEA RMC, GGAVTG unit 0.0125deg
    signed char SteeringWheelAngle;                             //vehicle handle angle, -126~127 unit 1.5 deg
    Acceleration_Set_4Way_ Acceleration_Set_4Way;
    unsigned char AuxiliaryBrakeStatus;       //parking brake(hand brake) state 0x00 unavail, 0x01 off, 0x02 on, 0x03 reserved(stability control is Engaged
    Vehicle_Size_ Vehicle_Size;

};

struct VehicleSafetyExtensions_
{
    VehicleEventFlags_ VehicleEventFlags;
};

struct MovementEventList_
{
    // MovementEventList
    unsigned char SignalGroupId;                   //
    unsigned char FirstMovementPhaseState;           //
    unsigned char FirstTimeChangeDetails[3];         //
    unsigned char SecondMovementPhaseState;           //
    unsigned char SecondTimeChangeDetails[3];         //
};

struct IntersectionStateList_Basic_Info_
{
    unsigned short IntersectionID;                                    //0~65535(provide random iD, for test 1~500, for contest 10001~50000
    unsigned char MsgCount;                                           //0~255, Message sequence
    unsigned char IntersectionStatusObject[14];                       //0~13 1)page 67
    Time_Stamp_ Time_Stamp;
};

struct IntersectionStateList_MovementList_
{
    unsigned char numOfMovementState;
    MovementEventList_ *MovementEventList;
};

struct IntersectionGeometry_BasicInfo_
{
    unsigned int MinuteOfTheYear;                                       //0~527040, 527040 is not validable, present UTC year's minute
    unsigned char MsgCount;                                             //Message sequence
};

struct IntersectionGeometry_LaneList_
{
    unsigned short IntersectionID;                                        //0~65535(provide random iD, for test 1~500, for contest 10001~50000
    char NodeID[12];                                                      //coordinate for calculate offset, but contest use this for show intersection position
    unsigned short LaneWidth;                                             //reference width, refer to NMEA GGA message
    SpeedLimitList_ SpeedLimitList;
    unsigned char NumOfLaneList;                                          //1~255, amount of lane list
    LaneList_ *LaneList;
};

struct RoadSideAlert_
{
    unsigned char MsgCount;//message sequence
    unsigned short ITIS_ITIScodes;//type of event
    unsigned char HeadingSlice[16];
    GPS_Position_ GPS_Position;
};

struct CallData_
{
    unsigned short Call_ID;
    unsigned char Call_Status;
    unsigned int Taxi_Fee;
    unsigned short Distance;
    unsigned char Number_of_passenger;
    PassengerPosition_ PassengerPosition;
    DestinationPosition_ DestinationPosition;
};

struct SelectCallDataList_
{
    unsigned short Call_ID;
    unsigned int Taxi_Fee;
    unsigned short Distance;
    unsigned char Number_of_Passenger;
    PassengerPosition_ PassengerPosition;
    DestinationPosition_ DestinationPosition;
    VehiclePosition_ VehiclePosition;
    unsigned short Distance_between_Passenger_and_vihecle;
    unsigned char Arrival_Time_to_Passengers;
    unsigned char Estimated_Time_to_Destination;
};
struct SelecCONFIRMCallDataList_
{
    unsigned short Call_ID;
    unsigned int Taxi_Fee;
};

//// Payload Messages
struct Payload_Registration_REQ
{
    unsigned char Device_ID[3];
    GPS_Position_ GPS_Position;
};
struct Payload_Registration_RES
{
    unsigned char Device_ID[3];
    unsigned char registration_status;  //0x01(ok), 0x02(FAIL)
    unsigned char Error_Code;           //0x00(none), 0x01(ERROR_PAYLAD_LEN), 0x02(ERROR_DEVICE_TYPE)
};
struct Payload_GPS_DATA
{
    unsigned char Device_ID[3];
    unsigned short SecMark;			//UTC
    GPS_Position_ GPS_Position;
    Position_Accuracy_ Position_Accuracy;
};
struct Payload_BasicSafetyMessage_DATA
{
    BSMcoreData_ BSMcoreData;
    VehicleSafetyExtensions_ VehicleSafetyExtensions;
};
struct Payload_SPAT_DATA
{
    IntersectionStateList_Basic_Info_ IntersectionStateList_Basic_Info;
    IntersectionStateList_MovementList_ IntersectionStateList_MovementList;
};
struct Payload_MAP_DATA
{
    IntersectionGeometry_BasicInfo_ IntersectionGeometry_BasicInfo;
    IntersectionGeometry_LaneList_ IntersectionGeometry_LaneList;
};
struct Payload_RoadSideAlert_DATA
{
    RoadSideAlert_ RoadSideAlert;
};
struct Payload_CallRequestList_DATA
{
    unsigned char Device_ID[3];
    unsigned short Call_List_ID;
    unsigned char Number_of_Call_Data;
    unsigned char Number_of_availavle_Call_Data;
    CallData_ *CallData;
};
struct Payload_CallSelect_RES
{
    unsigned char Device_ID[3];
    unsigned char Error_Status;
    unsigned char Number_of_Select_Call_Data;
    //Select Call Data List
    SelectCallDataList_ *SelectCallDataList;
};
struct Payload_CallMatching_CONFIRM
{
    unsigned char Device_ID[3];
    unsigned char Error_Status[2];
    unsigned char Number_of_Matching_Call_Data;
    // Select Call Data List
    SelecCONFIRMCallDataList_ *SelecCONFIRMCallDataList;
};
struct Payload_GettingOnComplete_REQ
{
    unsigned char Device_ID[3];
    unsigned char Error_Status;
    unsigned short Call_ID;
    unsigned char Number_of_Passenger;
    PassengerPosition_ PassengerPosition;
    VehiclePosition_ VehiclePosition;
    unsigned char Error_Range;
};
struct Payload_GettingOnComplete_CONFIRM
{
    unsigned char Device_ID[3];
    unsigned char Error_Status;
    unsigned short Call_ID;
    unsigned char Number_of_Passenger;
    PassengerPosition_ PassengerPosition;
    VehiclePosition_ VehiclePosition;
    unsigned char Error_Range;
    unsigned char Current_Speed;
};
struct Payload_GettingOffComplete_REQ
{
    unsigned char Device_ID[3];
    unsigned char Error_Status;
    unsigned short Call_ID;
    unsigned char Number_of_Passenger;
    DestinationPosition_ DestinationPosition;
    VehiclePosition_ VehiclePosition;
    unsigned char Error_Range;
};
struct Payload_GettingOffComplete_CONFIRM//we send
{
    unsigned char Device_ID[3];
    unsigned char Error_Status;
    unsigned short Call_ID;
    unsigned char Number_of_Passenger;
    DestinationPosition_ DestinationPosition;
    VehiclePosition_ VehiclePosition;
    unsigned char Error_Range;
    unsigned char Current_speed;
};
struct Payload_SinalViolation_Alert//we recive
{
    unsigned char Packet_Type[2];
    unsigned char Current_Sequence;
    unsigned short Payload_Size;
    unsigned char Device_Type;
    unsigned char Device_ID[3];
    GPS_Position_ VehiclePosition;
    unsigned short Speed;
    unsigned short Intersection_ID;
    unsigned char SignalGroupID;
    unsigned char MovementPhaseStatus;
};
struct Payload_MissionDrop_DATA
{
    unsigned char Device[3];
    unsigned short Call_ID;
};


//// Meta Messages
struct Registration_REQ // typedef : typedefinition, in this case change Registration_REQ to send
{
    Header_ header;
    Payload_Registration_REQ payload;
};

struct Registration_RES
{
    Header_ header;
    Payload_Registration_RES payload;
};

struct GPS_DATA //GPS
{
    Header_ header;
    Payload_GPS_DATA payload;
};

struct BasicSafetyMessage_DATA//get info from other OBU,V2V
{
    Header_ header;
    Payload_BasicSafetyMessage_DATA payload;
};

struct SPAT_DATA//signal Phase and Timing Message, we recieve from RSU
{
    Header_ header;
    Payload_SPAT_DATA payload;
};

struct MAP_DATA//intersection map info provide protocol,I2V
{
    Header_ header;
    Payload_MAP_DATA payload;
};

struct RoadSideAlert_DATA//accident, sudden event, harzard, construct, they will sent you message, if event clear they will give sign for 10second
{
    Header_ header;
    Payload_RoadSideAlert_DATA payload;
};

struct CallRequestList_DATA //we recieve
{
    Header_ header;
    Payload_CallRequestList_DATA payload;
};

struct CallSelect_RES //have to send
{
    Header_ header;
    Payload_CallSelect_RES payload;
};

struct CallMatching_CONFIRM//we recieve
{
    Header_ header;
    Payload_CallMatching_CONFIRM payload;
};

struct GettingOnComplete_REQ//we send
{
    Header_ header;
    Payload_GettingOnComplete_REQ payload;
};

struct GettingOnComplete_CONFIRM//we recieve
{
    Header_ header;
    Payload_GettingOnComplete_CONFIRM payload;
};

struct GettingOffComplete_REQ//we send
{
    Header_ header;
    Payload_GettingOffComplete_REQ payload;
};

struct GettingOffComplete_CONFIRM//we send
{
    Header_ header;
    Payload_GettingOffComplete_CONFIRM payload;
};

struct SinalViolation_Alert//we recive
{
    Header_ header;
    Payload_SinalViolation_Alert payload;
};
struct MissionDrop_DATA
{
    Header_ header;
    Payload_MissionDrop_DATA payload;
};

GettingOnComplete_REQ GettingOnComplete_REQfromROSmsg(wave_tcp::GettingOnComplete_REQ ros_msg, GettingOnComplete_REQ wave_msg)
{
    *wave_msg.header.Packet_Type = ros_msg.header.Packet_type;
    wave_msg.header.Current_Sequence = ros_msg.header.Current_sequence;
    wave_msg.header.Payload_Size = ros_msg.header.Payload_size;
    wave_msg.header.Device_Type = ros_msg.header.Device_type;

    *wave_msg.payload.Device_ID = ros_msg.payload.Device_ID;
    wave_msg.payload.Error_Status = ros_msg.payload.Error_Status;
    wave_msg.payload.Call_ID = ros_msg.payload.Call_ID;
    wave_msg.payload.Number_of_Passenger = ros_msg.payload.Number_of_Passenger;
    wave_msg.payload.PassengerPosition.Latitude = ros_msg.payload.Passenger_Position.Latitude;
    wave_msg.payload.PassengerPosition.Longitude = ros_msg.payload.Passenger_Position.Longitude;
    wave_msg.payload.VehiclePosition.Latitude = ros_msg.payload.Vehicle_Position_REQ.Latitude;
    wave_msg.payload.VehiclePosition.Longitude = ros_msg.payload.Vehicle_Position_REQ.Longitude;
    wave_msg.payload.Error_Range = ros_msg.payload.Error_Range;

    return wave_msg;
}
GettingOffComplete_REQ GettingOffComplete_REQfromROSmsg(wave_tcp::GettingOffComplete_REQ ros_msg, GettingOffComplete_REQ wave_msg)
{
    *wave_msg.header.Packet_Type = ros_msg.header.Packet_type;
    wave_msg.header.Current_Sequence = ros_msg.header.Current_sequence;
    wave_msg.header.Payload_Size = ros_msg.header.Payload_size;
    wave_msg.header.Device_Type = ros_msg.header.Device_type;

    *wave_msg.payload.Device_ID = ros_msg.payload.Device_ID;
    wave_msg.payload.Error_Status = ros_msg.payload.Error_Status;
    wave_msg.payload.Call_ID = ros_msg.payload.Call_ID;
    wave_msg.payload.Number_of_Passenger = ros_msg.payload.Number_of_Passenger;
    wave_msg.payload.DestinationPosition.Latitude = ros_msg.payload.Destination_Position.Latitude;
    wave_msg.payload.DestinationPosition.Longitude = ros_msg.payload.Destination_Position.Longitude;
    wave_msg.payload.VehiclePosition.Latitude = ros_msg.payload.Vehicle_Position_REQ.Latitude;
    wave_msg.payload.VehiclePosition.Longitude = ros_msg.payload.Vehicle_Position_REQ.Longitude;
    wave_msg.payload.Error_Range = ros_msg.payload.Error_Range;

    return wave_msg;
}
wave_tcp::Registration_REQ Registration_REQtoROSmsg(Registration_REQ wave_msg, wave_tcp::Registration_REQ ros_msg)
{
	ros_msg.header.Current_sequence = wave_msg.header.Current_Sequence;
	ros_msg.header.Device_type = wave_msg.header.Device_Type;
	ros_msg.header.Payload_size = wave_msg.header.Payload_Size;
	ros_msg.header.Packet_type = *wave_msg.header.Packet_Type;

	ros_msg.payload.Device_ID = *wave_msg.payload.Device_ID;
	ros_msg.payload.GpsPosition.Elevation = wave_msg.payload.GPS_Position.Elevation*0.1;
	ros_msg.payload.GpsPosition.Longitude = wave_msg.payload.GPS_Position.Longitude*pow(10,-7);
	ros_msg.payload.GpsPosition.Latitude = wave_msg.payload.GPS_Position.Latitude*pow(10,-7);

	return ros_msg;
}
wave_tcp::GPS_DATA GPS_DATAtoROSmsg(GPS_DATA wave_msg, wave_tcp::GPS_DATA ros_msg)
{
	ros_msg.header.Current_sequence = wave_msg.header.Current_Sequence;
	ros_msg.header.Device_type = wave_msg.header.Device_Type;
	ros_msg.header.Payload_size = wave_msg.header.Payload_Size;
	ros_msg.header.Packet_type = *wave_msg.header.Packet_Type;

	ros_msg.payload.Device_ID = *wave_msg.payload.Device_ID;
	ros_msg.payload.SecMark = wave_msg.payload.SecMark;
	ros_msg.payload.GpsPosition.Elevation = wave_msg.payload.GPS_Position.Elevation*0.1;
	ros_msg.payload.GpsPosition.Longitude = wave_msg.payload.GPS_Position.Longitude*pow(10,-7);
	ros_msg.payload.GpsPosition.Latitude = wave_msg.payload.GPS_Position.Latitude*pow(10,-7);
	ros_msg.payload.GPS_Position_Accuracy.semiMajor - wave_msg.payload.Position_Accuracy.semiMajor*0.05;
	ros_msg.payload.GPS_Position_Accuracy.semiMinor - wave_msg.payload.Position_Accuracy.semiMinor*0.05;
	ros_msg.payload.GPS_Position_Accuracy.orientation - wave_msg.payload.Position_Accuracy.orientation*360/35535;

	return ros_msg;
}
wave_tcp::BasicSafetyMessage_DATA BasicSafetyMessage_DATAtoROSmsg(BasicSafetyMessage_DATA wave_msg, wave_tcp::BasicSafetyMessage_DATA ros_msg)
{
	ros_msg.header.Current_sequence = wave_msg.header.Current_Sequence;
	ros_msg.header.Device_type = wave_msg.header.Device_Type;
	ros_msg.header.Payload_size = wave_msg.header.Payload_Size;
	ros_msg.header.Packet_type = *wave_msg.header.Packet_Type;

	ros_msg.payload.BSMcoreData.MsgCount = wave_msg.payload.BSMcoreData.MsgCount;
	ros_msg.payload.BSMcoreData.Tempoaray_ID = *wave_msg.payload.BSMcoreData.Temporary_ID;
	ros_msg.payload.BSMcoreData.DSecond = wave_msg.payload.BSMcoreData.DSecond;
	ros_msg.payload.BSMcoreData.GPS_Position.Elevation = wave_msg.payload.BSMcoreData.GPS_Position.Elevation*0.1;
	ros_msg.payload.BSMcoreData.GPS_Position.Latitude = wave_msg.payload.BSMcoreData.GPS_Position.Latitude*pow(10,-7);
	ros_msg.payload.BSMcoreData.GPS_Position.Longitude = wave_msg.payload.BSMcoreData.GPS_Position.Longitude*pow(10,-7);
	ros_msg.payload.BSMcoreData.Position_Accuracy.SemiMajorAxisAccuracy = wave_msg.payload.BSMcoreData.Position_Accuracy.semiMajor*0.05;
	ros_msg.payload.BSMcoreData.Position_Accuracy.SemiMinorAxisAccuracy = wave_msg.payload.BSMcoreData.Position_Accuracy.semiMinor*0.05;
	ros_msg.payload.BSMcoreData.Position_Accuracy.SemiMajorAxisOrientation = wave_msg.payload.BSMcoreData.Position_Accuracy.orientation*360/35535;
	ros_msg.payload.BSMcoreData.TransmissionState = wave_msg.payload.BSMcoreData.TransmissionState;
	ros_msg.payload.BSMcoreData.Speed = wave_msg.payload.BSMcoreData.Speed*0.02;
	ros_msg.payload.BSMcoreData.Heading = wave_msg.payload.BSMcoreData.Heading*0.0125;
	ros_msg.payload.BSMcoreData.SteeringWheelAngle = wave_msg.payload.BSMcoreData.SteeringWheelAngle*1.5;
	ros_msg.payload.BSMcoreData.Acceleration_Set_4Way.Latitude_Acceleration = wave_msg.payload.BSMcoreData.Acceleration_Set_4Way.Latitude_Acceleration*0.01;
	ros_msg.payload.BSMcoreData.Acceleration_Set_4Way.Longitude_Acceleration = wave_msg.payload.BSMcoreData.Acceleration_Set_4Way.Longitude_Acceleration*0.01;
	ros_msg.payload.BSMcoreData.Acceleration_Set_4Way.YawRate = wave_msg.payload.BSMcoreData.Acceleration_Set_4Way.YawRate*0.01;
	ros_msg.payload.BSMcoreData.BrakeSystemStatus_AuxiliaryBrake_Status = wave_msg.payload.BSMcoreData.AuxiliaryBrakeStatus;
	ros_msg.payload.BSMcoreData.Vehicle_Size.VehicleLength = wave_msg.payload.BSMcoreData.Vehicle_Size.VehicleLength*0.01;
	ros_msg.payload.BSMcoreData.Vehicle_Size.VehicleWidth = wave_msg.payload.BSMcoreData.Vehicle_Size.VehicleWidth*0.01;

	ros_msg.payload.VehicleSafetyExtensions.VehicleEventFlags.eventGetOnDown = wave_msg.payload.VehicleSafetyExtensions.VehicleEventFlags.eventGetOnDown;
	ros_msg.payload.VehicleSafetyExtensions.VehicleEventFlags.eventTrouble = wave_msg.payload.VehicleSafetyExtensions.VehicleEventFlags.eventTrouble;
	ros_msg.payload.VehicleSafetyExtensions.VehicleEventFlags.eventHardDeceleration = wave_msg.payload.VehicleSafetyExtensions.VehicleEventFlags.eventHardDeceleration;
	ros_msg.payload.VehicleSafetyExtensions.VehicleEventFlags.eventHardStop = wave_msg.payload.VehicleSafetyExtensions.VehicleEventFlags.eventHardStop;
	ros_msg.payload.VehicleSafetyExtensions.VehicleEventFlags.eventHardTurn = wave_msg.payload.VehicleSafetyExtensions.VehicleEventFlags.eventHardTurn;
	ros_msg.payload.VehicleSafetyExtensions.VehicleEventFlags.eventUTurn = wave_msg.payload.VehicleSafetyExtensions.VehicleEventFlags.eventUTurn;
	ros_msg.payload.VehicleSafetyExtensions.VehicleEventFlags.eventRoadWork = wave_msg.payload.VehicleSafetyExtensions.VehicleEventFlags.eventRoadWork;
	ros_msg.payload.VehicleSafetyExtensions.VehicleEventFlags.reserved = wave_msg.payload.VehicleSafetyExtensions.VehicleEventFlags.reserved;

	return ros_msg;
}
wave_tcp::SPAT_DATA SPAT_DATAtoROSmsg(SPAT_DATA wave_msg, wave_tcp::SPAT_DATA ros_msg)
{
	ros_msg.header.Current_sequence = wave_msg.header.Current_Sequence;
	ros_msg.header.Device_type = wave_msg.header.Device_Type;
	ros_msg.header.Payload_size = wave_msg.header.Payload_Size;
	ros_msg.header.Packet_type = *wave_msg.header.Packet_Type;

	ros_msg.payload.IntersectionStateList_basic_info.IntersectionID = wave_msg.payload.IntersectionStateList_Basic_Info.IntersectionID;
	ros_msg.payload.IntersectionStateList_basic_info.MsgCount = wave_msg.payload.IntersectionStateList_Basic_Info.MsgCount;
	ros_msg.payload.IntersectionStateList_basic_info.Time_Stamp.MinuteOfTheYear = wave_msg.payload.IntersectionStateList_Basic_Info.Time_Stamp.MinuteOfTheYear;
	ros_msg.payload.IntersectionStateList_basic_info.Time_Stamp.DSecond = wave_msg.payload.IntersectionStateList_Basic_Info.Time_Stamp.DSecond;

	ros_msg.payload.IntersectionStateList_Movementlist.numOfMovementState = wave_msg.payload.IntersectionStateList_MovementList.numOfMovementState;
	int NumofNovementState = wave_msg.payload.IntersectionStateList_MovementList.numOfMovementState;
	for(int i=0;i<NumofNovementState;i++)
	{
		wave_tcp::MovementEventList list;
		list.SignalGroupID = wave_msg.payload.IntersectionStateList_MovementList.MovementEventList[i].SignalGroupId;
		list.firstMovementPhaseState = wave_msg.payload.IntersectionStateList_MovementList.MovementEventList[i].FirstMovementPhaseState;
        int min, sec, tsec;
        min = wave_msg.payload.IntersectionStateList_MovementList.MovementEventList[i].FirstTimeChangeDetails[0];
        sec = wave_msg.payload.IntersectionStateList_MovementList.MovementEventList[i].FirstTimeChangeDetails[1];
        tsec = wave_msg.payload.IntersectionStateList_MovementList.MovementEventList[i].FirstTimeChangeDetails[2];
		list.firstTimeChnageDetails_minEndTime = min*60+sec+tsec*0.1;
		list.secondMovementPhaseState = wave_msg.payload.IntersectionStateList_MovementList.MovementEventList[i].SecondMovementPhaseState;
        min = wave_msg.payload.IntersectionStateList_MovementList.MovementEventList[i].SecondTimeChangeDetails[0];
        sec = wave_msg.payload.IntersectionStateList_MovementList.MovementEventList[i].SecondTimeChangeDetails[1];
        tsec = wave_msg.payload.IntersectionStateList_MovementList.MovementEventList[i].SecondTimeChangeDetails[2];
		list.secondTimeChangeDetails_minEndTime = min*60+sec+tsec*0.1;

		ros_msg.payload.IntersectionStateList_Movementlist.MovementEventList.push_back(list);
	}

	return ros_msg;
}
wave_tcp::MAP_DATA MAP_DATAtoROSmsg(MAP_DATA wave_msg, wave_tcp::MAP_DATA ros_msg)
{
	ros_msg.header.Current_sequence = wave_msg.header.Current_Sequence;
	ros_msg.header.Device_type = wave_msg.header.Device_Type;
	ros_msg.header.Payload_size = wave_msg.header.Payload_Size;
	ros_msg.header.Packet_type = *wave_msg.header.Packet_Type;

	ros_msg.payload.intersectionGeometry_Basic_info.MinuteOfTheYear = wave_msg.payload.IntersectionGeometry_BasicInfo.MinuteOfTheYear;
	ros_msg.payload.intersectionGeometry_Basic_info.MsgCount = wave_msg.payload.IntersectionGeometry_BasicInfo.MsgCount;
	ros_msg.payload.intersectionGeometry_LaneList.IntersectionID = wave_msg.payload.IntersectionGeometry_LaneList.IntersectionID;
	memcpy(ros_msg.payload.intersectionGeometry_LaneList.NodeID.begin(),wave_msg.payload.IntersectionGeometry_LaneList.NodeID,
		   sizeof(wave_msg.payload.IntersectionGeometry_LaneList.NodeID));
	ros_msg.payload.intersectionGeometry_LaneList.LaneWidth = wave_msg.payload.IntersectionGeometry_LaneList.LaneWidth;
	ros_msg.payload.intersectionGeometry_LaneList.SpeedLimitList.SpeedLimitType = wave_msg.payload.IntersectionGeometry_LaneList.SpeedLimitList.SpeedLimitType;
	ros_msg.payload.intersectionGeometry_LaneList.SpeedLimitList.Velocity = wave_msg.payload.IntersectionGeometry_LaneList.SpeedLimitList.Velocity*0.02;
	ros_msg.payload.intersectionGeometry_LaneList.NumOfLaneList = wave_msg.payload.IntersectionGeometry_LaneList.NumOfLaneList;
	int NumOfLaneList = wave_msg.payload.IntersectionGeometry_LaneList.NumOfLaneList;
	for(int i=0;i<NumOfLaneList;i++)
	{
		wave_tcp::LaneList list;
		memcpy(list.LaneID.begin(), wave_msg.payload.IntersectionGeometry_LaneList.LaneList[i].LaneID, sizeof(wave_msg.payload.IntersectionGeometry_LaneList.LaneList->LaneID));
		list.LaneDirection = wave_msg.payload.IntersectionGeometry_LaneList.LaneList[i].LaneDirection;
		memcpy(list.LaneSharing.begin(),wave_msg.payload.IntersectionGeometry_LaneList.LaneList[i].LaneSharing, sizeof(wave_msg.payload.IntersectionGeometry_LaneList.LaneList->LaneSharing));
		memcpy(list.LaneTypeAttributes_Vehicle.begin(),wave_msg.payload.IntersectionGeometry_LaneList.LaneList[i].LaneTypeAttributes_vehicle,
			   sizeof(wave_msg.payload.IntersectionGeometry_LaneList.LaneList->LaneTypeAttributes_vehicle));
		memcpy(list.AllowedManeuvers.begin(),wave_msg.payload.IntersectionGeometry_LaneList.LaneList[i].AllowedManeuvers,
			   sizeof(wave_msg.payload.IntersectionGeometry_LaneList.LaneList[i].AllowedManeuvers));
		memcpy(list.firstLaneID.begin(),wave_msg.payload.IntersectionGeometry_LaneList.LaneList[i].FirstLaneID, sizeof(wave_msg.payload.IntersectionGeometry_LaneList.LaneList[i].FirstLaneID));
		memcpy(list.secondLaneID.begin(),wave_msg.payload.IntersectionGeometry_LaneList.LaneList[i].SecondLaneID,
			   sizeof(wave_msg.payload.IntersectionGeometry_LaneList.LaneList[i].SecondLaneID));
		list.signalGroupID = wave_msg.payload.IntersectionGeometry_LaneList.LaneList[i].signalGroupID;

        ros_msg.payload.intersectionGeometry_LaneList.LaneList.push_back(list);
	}

	return ros_msg;
}
wave_tcp::RoadSideAlert_DATA RoadSideAlert_DATAtoROSmsg(RoadSideAlert_DATA wave_msg, wave_tcp::RoadSideAlert_DATA ros_msg)
{
	ros_msg.header.Current_sequence = wave_msg.header.Current_Sequence;
	ros_msg.header.Device_type = wave_msg.header.Device_Type;
	ros_msg.header.Payload_size = wave_msg.header.Payload_Size;
	ros_msg.header.Packet_type = *wave_msg.header.Packet_Type;

	ros_msg.payload.roadSideAlert.MsgCount = wave_msg.payload.RoadSideAlert.MsgCount;
	ros_msg.payload.roadSideAlert.ITIS_ITIScodes = wave_msg.payload.RoadSideAlert.ITIS_ITIScodes;
	memcpy(ros_msg.payload.roadSideAlert.HeadingSlice.begin(),wave_msg.payload.RoadSideAlert.HeadingSlice,sizeof(wave_msg.payload.RoadSideAlert.HeadingSlice));
	ros_msg.payload.roadSideAlert.Position.Elevation = wave_msg.payload.RoadSideAlert.GPS_Position.Elevation*0.1;
	ros_msg.payload.roadSideAlert.Position.Longitude = wave_msg.payload.RoadSideAlert.GPS_Position.Longitude*pow(10,-7);
	ros_msg.payload.roadSideAlert.Position.Latitude = wave_msg.payload.RoadSideAlert.GPS_Position.Latitude*pow(10,-7);

	return ros_msg;
}
wave_tcp::CallRequestList_DATA CallRequestList_DATAtoROSmsg(CallRequestList_DATA wave_msg, wave_tcp::CallRequestList_DATA ros_msg)
{
    ros_msg.header.Current_sequence = wave_msg.header.Current_Sequence;
    ros_msg.header.Device_type = wave_msg.header.Device_Type;
    ros_msg.header.Payload_size = wave_msg.header.Payload_Size;
    ros_msg.header.Packet_type = *wave_msg.header.Packet_Type;

    ros_msg.payload.Device_ID = *wave_msg.payload.Device_ID;
    ros_msg.payload.Number_of_available_Call_Data = wave_msg.payload.Number_of_availavle_Call_Data;
    int NumOfCallData = wave_msg.payload.Number_of_Call_Data;
    ros_msg.payload.Number_of_Call_Data = NumOfCallData;
    for(int i=0;i<NumOfCallData;i++)
    {
        wave_tcp::Call_Data list;
        list.Call_ID = wave_msg.payload.CallData[i].Call_ID;
        list.Call_status = wave_msg.payload.CallData[i].Call_Status;
        list.Taxi_Fee = wave_msg.payload.CallData[i].Taxi_Fee;
        list.Distance = wave_msg.payload.CallData[i].Distance;
        list.Number_of_Passenger = wave_msg.payload.CallData[i].Number_of_passenger;
        list.Passenger_Position.Longitude = wave_msg.payload.CallData[i].PassengerPosition.Longitude;
        list.Passenger_Position.Latitude = wave_msg.payload.CallData[i].PassengerPosition.Latitude;
        list.Destination_Position.Latitude = wave_msg.payload.CallData[i].DestinationPosition.Latitude;
        list.Destination_Position.Longitude = wave_msg.payload.CallData[i].DestinationPosition.Longitude;

        ros_msg.payload.Call_Data.push_back(list);
    }

    return ros_msg;
}
wave_tcp::CallMatching_CONFIRM CallMatching_CONFIRMtoROSmsg(CallMatching_CONFIRM wave_msg, wave_tcp::CallMatching_CONFIRM ros_msg)
{
    ros_msg.header.Current_sequence = wave_msg.header.Current_Sequence;
    ros_msg.header.Device_type = wave_msg.header.Device_Type;
    ros_msg.header.Payload_size = wave_msg.header.Payload_Size;
    ros_msg.header.Packet_type = *wave_msg.header.Packet_Type;

    ros_msg.payload.Device_ID = *wave_msg.payload.Device_ID;
    ros_msg.payload.Error_Status = *wave_msg.payload.Error_Status;
    int NumOfMatchingCallData = wave_msg.payload.Number_of_Matching_Call_Data;
    ros_msg.payload.Number_of_Matching_Call_Data = NumOfMatchingCallData;
    for(int i=0;i<NumOfMatchingCallData;i++)
    {
		wave_tcp::SelecConfirmCallDataList list;
        list.Call_ID = wave_msg.payload.SelecCONFIRMCallDataList[i].Call_ID;
		list.Taxi_Fee = wave_msg.payload.SelecCONFIRMCallDataList[i].Taxi_Fee;

        ros_msg.payload.SelectCalldatalist.push_back(list);
    }

	return ros_msg;
}
wave_tcp::GettingOnComplete_CONFIRM GettingOnComplete_CONFIRMtoROSmsg(GettingOnComplete_CONFIRM wave_msg, wave_tcp::GettingOnComplete_CONFIRM ros_msg)
{
    ros_msg.header.Current_sequence = wave_msg.header.Current_Sequence;
    ros_msg.header.Device_type = wave_msg.header.Device_Type;
    ros_msg.header.Payload_size = wave_msg.header.Payload_Size;
    ros_msg.header.Packet_type = *wave_msg.header.Packet_Type;

    ros_msg.payload.Device_ID = *wave_msg.payload.Device_ID;
    ros_msg.payload.Error_Status = wave_msg.payload.Error_Status;
    ros_msg.payload.Call_ID = wave_msg.payload.Call_ID;
    ros_msg.payload.Number_of_Passenger = wave_msg.payload.Number_of_Passenger;
    ros_msg.payload.Passenger_Position.Longitude = wave_msg.payload.PassengerPosition.Longitude;
    ros_msg.payload.Passenger_Position.Latitude = wave_msg.payload.PassengerPosition.Latitude;
    ros_msg.payload.Vehicle_Position_Confirm.Latitude = wave_msg.payload.VehiclePosition.Latitude;
    ros_msg.payload.Vehicle_Position_Confirm.Longitude = wave_msg.payload.VehiclePosition.Longitude;
    ros_msg.payload.Error_Range = wave_msg.payload.Error_Range;
    ros_msg.payload.Current_Speed = wave_msg.payload.Current_Speed;

    return ros_msg;
}
wave_tcp::GettingOffComplete_CONFIRM GettingOffComplete_CONFIRMtoROSmsg(GettingOffComplete_CONFIRM wave_msg, wave_tcp::GettingOffComplete_CONFIRM ros_msg)
{
    ros_msg.header.Current_sequence = wave_msg.header.Current_Sequence;
    ros_msg.header.Device_type = wave_msg.header.Device_Type;
    ros_msg.header.Payload_size = wave_msg.header.Payload_Size;
    ros_msg.header.Packet_type = *wave_msg.header.Packet_Type;

    ros_msg.payload.Device_ID = *wave_msg.payload.Device_ID;
    ros_msg.payload.Error_Status = wave_msg.payload.Error_Status;
    ros_msg.payload.Call_ID = wave_msg.payload.Call_ID;
    ros_msg.payload.Number_of_Passenger = wave_msg.payload.Number_of_Passenger;
    ros_msg.payload.Destination_Position.Longitude = wave_msg.payload.DestinationPosition.Longitude;
    ros_msg.payload.Destination_Position.Latitude = wave_msg.payload.DestinationPosition.Latitude;
    ros_msg.payload.Vehicle_Position_Confirm.Latitude = wave_msg.payload.VehiclePosition.Latitude;
    ros_msg.payload.Vehicle_Position_Confirm.Longitude = wave_msg.payload.VehiclePosition.Longitude;
    ros_msg.payload.Error_Range = wave_msg.payload.Error_Range;
    ros_msg.payload.Current_Speed = wave_msg.payload.Current_speed;

    return ros_msg;
}
wave_tcp::SinalViolation_Alert SinalViolation_AlerttoROSmsg(SinalViolation_Alert wave_msg, wave_tcp::SinalViolation_Alert ros_msg)
{
    ros_msg.header.Current_sequence = wave_msg.header.Current_Sequence;
    ros_msg.header.Device_type = wave_msg.header.Device_Type;
    ros_msg.header.Payload_size = wave_msg.header.Payload_Size;
    ros_msg.header.Packet_type = *wave_msg.header.Packet_Type;

    ros_msg.payload.Device_ID = *wave_msg.payload.Device_ID;
    ros_msg.payload.Vehicle_Position.Longitude = wave_msg.payload.VehiclePosition.Longitude;
    ros_msg.payload.Vehicle_Position.Latitude = wave_msg.payload.VehiclePosition.Latitude;
    ros_msg.payload.Vehicle_Position.Elevation = wave_msg.payload.VehiclePosition.Elevation;
    ros_msg.payload.Vehicle_Status.Speed = wave_msg.payload.Speed;
    ros_msg.payload.Intersection_info.Intersection_ID = wave_msg.payload.Intersection_ID;
    ros_msg.payload.Intersection_info.SignalGroupID = wave_msg.payload.SignalGroupID;
    ros_msg.payload.Intersection_info.MovementPhaseState = wave_msg.payload.MovementPhaseStatus;

    return ros_msg;
}
#pragma pack(pop)
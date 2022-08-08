/*
 * UPER Based client application sample code
 *
 * Copyright (c) 2022, CEST
 *
 */

#include "j2735.h"
#include <time.h>
#include <errno.h> // errno
#include <sys/socket.h> // socket functions
#include <netinet/in.h> // sock structures
#include <netdb.h> // getaddrinfo()
#include <unistd.h> // close()

int fill_j2735_pvd(MessageFrame_t *dst, FILE *fd_pvd, struct PVDdatafromROS pvd)
{
  // 1. Variables
  // 1) To get UTC time
  time_t tnow;
  struct tm* t, t_b4;
  time(&tnow);
  t = (struct tm*)localtime(&tnow);
  
  // 2. Set dst (data before encoding)
  //ASN_STRUCT_RESET(asn_DEF_MessageFrame, dst);
  dst->messageId = 26; // J2735 표준문서 PDF 파일 참조 DE_DSRC_MessageID,  probeVehicleData DSRCmsgID ::= 26 -- PVD
  dst->value.present = MessageFrame__value_PR_ProbeVehicleData; // MessageFrame::value choice (asn1c)
  ProbeVehicleData_t *ptrPvd = &dst->value.choice.ProbeVehicleData;
  
  ptrPvd->timeStamp = NULL; // OPTIONAL, not to use
  ptrPvd->segNum = NULL;    // OPTIONAL, not to use
  ptrPvd->regional = NULL;  // OPTIONAL, not to use
  
  ptrPvd->probeID = malloc(sizeof(struct VehicleIdent));
  ptrPvd->probeID->name = NULL;         // OPTIONAL, not to use
  ptrPvd->probeID->ownerCode = NULL;    // OPTIONAL, not to use
  ptrPvd->probeID->vehicleClass = NULL; // OPTIONAL, not to use
  ptrPvd->probeID->vin = NULL;          // OPTIONAL, not to use
  ptrPvd->probeID->vehicleType = NULL;  // OPTIONAL, not to use
  ptrPvd->probeID->id = malloc(sizeof (struct VehicleID));
  ptrPvd->probeID->id->present = VehicleID_PR_entityID;   
  ptrPvd->probeID->id->present = VehicleID_PR_entityID;
  ptrPvd->probeID->id->choice.entityID.buf = (unsigned char *)malloc(4);
  ptrPvd->probeID->id->choice.entityID.size = 4; 
  ptrPvd->probeID->id->choice.entityID.buf[0] = 0xCE;      // (INPUT) <---- 할당된 대학별 ID 입력 [F]
  ptrPvd->probeID->id->choice.entityID.buf[1] = 0x24;      // (INPUT) <---- 할당된 대학별 ID 입력 [F]
  ptrPvd->probeID->id->choice.entityID.buf[2] = 0x67;      // (INPUT) <---- 할당된 대학별 ID 입력 [F]
  ptrPvd->probeID->id->choice.entityID.buf[3] = 0x09;      // (INPUT) <---- 할당된 대학별 ID 입력 [F]
  // data type of ~.entityID.buf[i] --> OCTECT_STRING (sequence of byte)
  
  //StartVector : PVD를 전송할 시점을 기준의 시간과 차량의 위치, 이동상태 값을 반영
  ptrPvd->startVector.utcTime = malloc(sizeof(struct DDateTime));  
  ptrPvd->startVector.utcTime->year = malloc(sizeof(DYear_t));
  ptrPvd->startVector.utcTime->month = malloc(sizeof(DMonth_t)); 
  ptrPvd->startVector.utcTime->day = malloc(sizeof(DDay_t)); 
  ptrPvd->startVector.utcTime->hour = malloc(sizeof(DHour_t)); 
  ptrPvd->startVector.utcTime->minute = malloc(sizeof(DMinute_t)); 
  ptrPvd->startVector.utcTime->second = malloc(sizeof(DSecond_t)); 
  ptrPvd->startVector.utcTime->offset = NULL; // OPTIONAL, not to use
  
  *ptrPvd->startVector.utcTime->year = t->tm_year+1900; // (INPUT) <--------------- 현재 UTC 시간 입력 (년도) [F]
  *ptrPvd->startVector.utcTime->month = t->tm_mon+1;   // (INPUT) <--------------- 현재 UTC 시간 입력 (년도) [F]
  *ptrPvd->startVector.utcTime->day = t->tm_mday;     // (INPUT) <--------------- 현재 UTC 시간 입력 (년도) [F]
  *ptrPvd->startVector.utcTime->hour = t->tm_hour;    // (INPUT) <--------------- 현재 UTC 시간 입력 (년도) [F]
  *ptrPvd->startVector.utcTime->minute = t->tm_min;  // (INPUT) <--------------- 현재 UTC 시간 입력 (년도) [F]
  *ptrPvd->startVector.utcTime->second = t->tm_sec;  // (INPUT) <--------------- 현재 UTC 시간 입력 (년도) [F]
  
  ptrPvd->startVector.elevation = malloc(sizeof(DSRC_Elevation_t));
  ptrPvd->startVector.heading = malloc(sizeof(Heading_t));
  ptrPvd->startVector.speed = malloc(sizeof(struct TransmissionAndSpeed));
  ptrPvd->startVector.posAccuracy = NULL;     // OPTIONAL, not to use
  ptrPvd->startVector.posConfidence = NULL;   // OPTIONAL, not to use
  ptrPvd->startVector.timeConfidence = NULL;  // OPTIONAL, not to use
  ptrPvd->startVector.speedConfidence = NULL; // OPTIONAL, not to use
  
  ptrPvd->startVector.Long = pvd.longitude;                   // (INPUT) <--------------- 현재 차량의 위치 (위도) (Longitude, DD 좌표계); long
  ptrPvd->startVector.lat = pvd.latitude;                     // (INPUT) <--------------- 현재 차량의 위치 (경도) (Latitude,  DD 좌표계); long
  *ptrPvd->startVector.elevation = pvd.elevation;             // (INPUT) <--------------- 현재 차량의 위치 (고도) (Elevation); long
  *ptrPvd->startVector.heading = pvd.heading;                 // (INPUT) <--------------- 현재 차량의 주행 방향 (북쪽 0도); long
  ptrPvd->startVector.speed->speed = pvd.speed;               // (INPUT) <--------------- 현재 차량의 속도; long
  ptrPvd->startVector.speed->transmisson = pvd.transmission;  // (INPUT) <--------------- 현재 차량의 변속기 상태; long
  ptrPvd->vehicleType.hpmsType = malloc(sizeof(VehicleType_t));
  ptrPvd->vehicleType.keyType = NULL;       // OPTIONAL, not to use
  ptrPvd->vehicleType.fuelType = NULL;      // OPTIONAL, not to use
  ptrPvd->vehicleType.iso3883 = NULL;       // OPTIONAL, not to use
  ptrPvd->vehicleType.regional = NULL;      // OPTIONAL, not to use
  ptrPvd->vehicleType.responderType = NULL; // OPTIONAL, not to use
  ptrPvd->vehicleType.responseEquip = NULL; // OPTIONAL, not to use
  ptrPvd->vehicleType.role = NULL;          // OPTIONAL, not to use
  ptrPvd->vehicleType.vehicleType = NULL;   // OPTIONAL, not to use
  *ptrPvd->vehicleType.hpmsType = VehicleType_car; 
  
  
  // PVD 전송 직전에 전송한 PVD startVector 시간, 위치, 이동상태를 입력
  ptrPvd->snapshots.list.count = 1; 
  ptrPvd->snapshots.list.array = malloc(sizeof(struct Snapshot *));
  ptrPvd->snapshots.list.array[0] = malloc(sizeof(struct Snapshot));
  struct Snapshot *ptrSnapshot = ptrPvd->snapshots.list.array[0]; 
  
  ptrSnapshot->thePosition.utcTime = malloc(sizeof(struct DDateTime));
  ptrSnapshot->thePosition.utcTime->year = malloc(sizeof(DYear_t));
  ptrSnapshot->thePosition.utcTime->month = malloc(sizeof(DMonth_t));
  ptrSnapshot->thePosition.utcTime->day = malloc(sizeof(DDay_t));
  ptrSnapshot->thePosition.utcTime->hour = malloc(sizeof(DHour_t));
  ptrSnapshot->thePosition.utcTime->minute = malloc(sizeof(DMinute_t));
  ptrSnapshot->thePosition.utcTime->second = malloc(sizeof(DSecond_t));
  ptrSnapshot->thePosition.utcTime->offset = NULL; // OPTIONAL, not to use
  
  ptrSnapshot->thePosition.elevation = malloc(sizeof(DSRC_Elevation_t));
  ptrSnapshot->thePosition.speed = malloc(sizeof(struct TransmissionAndSpeed));
  ptrSnapshot->thePosition.heading = malloc(sizeof(Heading_t));
  ptrSnapshot->thePosition.posAccuracy = NULL;     // OPTIONAL, not to use
  ptrSnapshot->thePosition.posConfidence = NULL;   // OPTIONAL, not to use
  ptrSnapshot->thePosition.timeConfidence = NULL;  // OPTIONAL, not to use
  ptrSnapshot->thePosition.speedConfidence = NULL; // OPTIONAL, not to use
  
  *ptrSnapshot->thePosition.utcTime->year = t->tm_year+1900;       // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (년도) [F]
  *ptrSnapshot->thePosition.utcTime->month = t->tm_mon+1;         // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (월) [F]
  *ptrSnapshot->thePosition.utcTime->day = t->tm_mday;           // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (일) [F]
  *ptrSnapshot->thePosition.utcTime->hour = t->tm_hour;          // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (시) [F]
  *ptrSnapshot->thePosition.utcTime->minute = t->tm_min;        // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (분) [F]
  *ptrSnapshot->thePosition.utcTime->second = t->tm_sec;        // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (초) [F]
  
  ptrSnapshot->thePosition.lat = pvd.latitude;                    // (INPUT) <--------------- 현재 차량의 위치 (위도) (Longitude, DD 좌표계)
  ptrSnapshot->thePosition.Long = pvd.longitude;                  // (INPUT) <--------------- 현재 차량의 위치 (경도) (Latitude,  DD 좌표계)
  *ptrSnapshot->thePosition.elevation = pvd.elevation;            // (INPUT) <--------------- 현재 차량의 위치 (고도) (Elevation)
  *ptrSnapshot->thePosition.heading = pvd.heading;                // (INPUT) <--------------- 현재 차량의 주행 방향 (북쪽 0도)
  ptrSnapshot->thePosition.speed->speed = pvd.speed;              // (INPUT) <-------------- -현재 차량의 속도
  ptrSnapshot->thePosition.speed->transmisson = pvd.transmission; // (INPUT) <--------------- 현재 차량의 변속기 상태
 
  fprintf(fd_pvd, "============================================\n");
  fprintf(fd_pvd, "probeID\n");
  fprintf(fd_pvd, "\t id\n");
  fprintf(fd_pvd, "\t\t entityID: 0x");
  for(int i = 0; i < ptrPvd->probeID->id->choice.entityID.size; i++){
  	fprintf(fd_pvd, "%02x", ptrPvd->probeID->id->choice.entityID.buf[i]);
  }
  fprintf(fd_pvd, "\n");
  fprintf(fd_pvd, "startVector\n");
  fprintf(fd_pvd, "\t utcTime\n");
  fprintf(fd_pvd, "\t\t year: %ld\n", *ptrPvd->startVector.utcTime->year);
  fprintf(fd_pvd, "\t\t month: %ld\n", *ptrPvd->startVector.utcTime->month);
  fprintf(fd_pvd, "\t\t day: %ld\n", *ptrPvd->startVector.utcTime->day);
  fprintf(fd_pvd, "\t\t hour: %ld\n", *ptrPvd->startVector.utcTime->hour);
  fprintf(fd_pvd, "\t\t minute: %ld\n", *ptrPvd->startVector.utcTime->minute);
  fprintf(fd_pvd, "\t\t second: %ld\n", *ptrPvd->startVector.utcTime->second);
  fprintf(fd_pvd, "\t Long: %ld\n", ptrPvd->startVector.Long);
  fprintf(fd_pvd, "\t lat: %ld\n", ptrPvd->startVector.lat);
  fprintf(fd_pvd, "\t elevation: %ld\n", *ptrPvd->startVector.elevation);
  fprintf(fd_pvd, "\t heading: %ld\n", *ptrPvd->startVector.heading);
  fprintf(fd_pvd, "\t speed\n");
  fprintf(fd_pvd, "\t\t speed: %ld\n", ptrPvd->startVector.speed->speed);
  fprintf(fd_pvd, "\t\t transmisson: %ld\n", ptrPvd->startVector.speed->transmisson);
  fprintf(fd_pvd, "vehicleType\n");	
  fprintf(fd_pvd, "\t hpmsType: %ld\n", *ptrPvd->vehicleType.hpmsType);
  fprintf(fd_pvd, "snapshots[0]\n");
  fprintf(fd_pvd, "\t thePosition\n");
  fprintf(fd_pvd, "\t\t utcTime\n");
  fprintf(fd_pvd, "\t\t\t year: %ld\n", *ptrSnapshot->thePosition.utcTime->year);
  fprintf(fd_pvd, "\t\t\t month: %ld\n", *ptrSnapshot->thePosition.utcTime->month);
  fprintf(fd_pvd, "\t\t\t day: %ld\n", *ptrSnapshot->thePosition.utcTime->day);
  fprintf(fd_pvd, "\t\t\t hour: %ld\n", *ptrSnapshot->thePosition.utcTime->hour);
  fprintf(fd_pvd, "\t\t\t minute: %ld\n", *ptrSnapshot->thePosition.utcTime->minute);
  fprintf(fd_pvd, "\t\t\t second: %ld\n", *ptrSnapshot->thePosition.utcTime->second);
  fprintf(fd_pvd, "\t\t Long: %ld\n", ptrSnapshot->thePosition.Long);
  fprintf(fd_pvd, "\t\t lat: %ld\n", ptrSnapshot->thePosition.lat);
  fprintf(fd_pvd, "\t\t elevation: %ld\n", *ptrSnapshot->thePosition.elevation);
  fprintf(fd_pvd, "\t\t heading: %ld\n", *ptrSnapshot->thePosition.heading);
  fprintf(fd_pvd, "\t\t speed\n");
  fprintf(fd_pvd, "\t\t\t speed: %ld\n", ptrSnapshot->thePosition.speed->speed);
  fprintf(fd_pvd, "\t\t\t transmisson: %ld\n", ptrSnapshot->thePosition.speed->transmisson);
  fprintf(fd_pvd, "============================================\n");
  return 0;
}

/*
 * UPER Based client application sample code
 *
 * Copyright (c) 2022, CEST
 *
 */

#include "j2735.h"
#include <signal.h>

void print_hex(char *data, int len, FILE *fp){
  fprintf(fp, "HEX[%d] : ",len);
  for(int i = 0 ; i < len ; i++){
    fprintf(fp, "%02X",(data[i] & 0xFF));
  }
  fprintf(fp, "\n");
}


int encode_j2735_uper(char *dst, unsigned short dstLen, MessageFrame_t *src)
{
    int res = -1;

    asn_enc_rval_t ret = uper_encode_to_buffer(&asn_DEF_MessageFrame,
                                               NULL,
                                               src,
                                               dst, dstLen);
      
    if (ret.encoded > 0)
        return ret.encoded; //  UPER Encoding Success
    else
    { 
        if (ret.failed_type != NULL)
            printf("encoded error value name = %s\n", ret.failed_type->name);

        return -1; // UPER Encoding failed
    }
}

int decode_j2735_uper(MessageFrame_t *dst, char *src, int size, FILE *fp){ 
  
    int res = -1;

    MessageFrame_t *ptrMsg = NULL; 

    asn_dec_rval_t ret = uper_decode(NULL,
                                     &asn_DEF_MessageFrame,
                                     (void **)&dst,
                                     src, size, 0, 0);

    if (ret.code != RC_OK)
        return res;
    
    res = ret.consumed;
 
    //asn_fprint(stdout,&asn_DEF_MessageFrame,dst);

    parse_decoded_j2735(dst, fp);

    return res;
}
 
int parse_decoded_j2735(MessageFrame_t *msg, FILE *fp)
{ 
  switch(msg->messageId){
  case DSRC_ID_BSM:
    fprintf(fp, ">> Parse J2735 : BSM\n");
    break;
  case DSRC_ID_SPAT:
    fprintf(fp, ">> Parse J2735 : SPAT\n");
    struct MessageFrame__value value = msg->value;
    union MessageFrame__value_u value_u = value.choice;
    SPAT_t spt = value_u.SPAT;
    parse_spat(&spt, fp);
    break;  
  case DSRC_ID_MAP:
    fprintf(fp, ">> Parse J2735 : MAP\n");
    break;
  }
  return 0;
}
 

int parse_map(MapData_t *map){ // Not needed

    for (int i = 0; i < map->intersections->list.count; i++)
    {  
         struct IntersectionGeometry *ptr= map->intersections->list.array[i]; 
        // MISSION : MAP 메시지에 포함된 IntersectionGeometry별 Intersection ID 추출
        //           Intersection 내 Ref Position을 기준으로 LaneSet의 Node 좌표 계산 및 출력
        //           Node로 연결된 차선의 Line 별 ID와 SignalGroupID를 출력 
    }

    return 0;
}
int parse_spat(SPAT_t *spat, FILE *fp){

    for (int i = 0; i < spat->intersections.list.count; i++)
    {
        struct IntersectionState *ptr = spat->intersections.list.array[i];

        // MISSION : SPAT 메시지에 포함된 Intersection ID 추출
        //           Intersection 내 Ref Position을 기준으로 Offset Node 좌표 추출
        //           Node로 연결된 차선의 Line 별 ID와 SignalGroupID를 출력
	fprintf(fp, "-----------------------------------\n");
	fprintf(fp, "name: %s\n", ptr->name->buf);
	fprintf(fp, "id \n");
	fprintf(fp, "    region : %ld\n", *(ptr->id.region));
	fprintf(fp, "    id: %ld\n", ptr->id.id);
	fprintf(fp, "revision: %ld\n", ptr->revision);
	/*
	for(int index = 0; index < ptr->status.size; index++)
	{
		printf("status: %d\n", ptr->status.buf[index]);
	}
	*/
	fprintf(fp, "status: %d\n", ptr->status.buf[1]);
	fprintf(fp, "moy: %ld\n", *(ptr->moy));
	fprintf(fp, "timeStamp: %ld\n", *(ptr->timeStamp));
	fprintf(fp, "-----------------------------------\n");
	for(int j = 0; j < ptr->states.list.count; j++){
	  struct MovementState *m_ptr = ptr->states.list.array[j];
	  fprintf(fp, "movementNames: %s\n", m_ptr->movementName->buf); // Signal Group ID
	  fprintf(fp, "signal group: %ld\n", m_ptr->signalGroup); // Signal Group ID
	  for(int k = 0; k < m_ptr->state_time_speed.list.count; k++){
	    struct MovementEvent *e_ptr = m_ptr->state_time_speed.list.array[k];
	    fprintf(fp, "state-time-speed\n");
	    fprintf(fp, "    eventState: %ld\n", e_ptr->eventState); // Event Sate
	    struct TimeChangeDetails *t_ptr = e_ptr->timing;
	    fprintf(fp, "    timing_minEndtime: %ld\n", t_ptr->minEndTime); // min end time
	  }
	  for(int k = 0; k < m_ptr->maneuverAssistList->list.count; k++){
	    struct ConnectionManeuverAssist *e_ptr = m_ptr->maneuverAssistList->list.array[k];
	    fprintf(fp, "maneuverAssistList\n");
	    fprintf(fp, "    connectionID: %ld\n", e_ptr->connectionID); 
	    fprintf(fp, "    pedBicycleDetect: %s \n", e_ptr->pedBicycleDetect ? "true" : "false"); 
	    // printf("\t pedBicycleDetect: %s\n", e_ptr->pedBicycleDetect); 
	  }
	  fprintf(fp, "-----------------------------------\n");
	}
    }

    return 0;
}
int parse_bsm(BasicSafetyMessage_t *bsm){
 
    // MISSION : BSM 내 temporary ID 추출
    //           차량의 위치(위도,경도,고도)와 주행 방향, 속도 출력
 
    return 0;
}

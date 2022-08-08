/*
 * UPER Based client application sample code
 *
 * Copyright (c) 2022, CEST
 *
 */

#include <stdio.h> 
#include "sample.h"
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <sys/types.h> // waitpid()
#include <sys/wait.h> // waitpid()
#include <sys/socket.h> // socket functions
#include <netinet/in.h> // sock structures
#include <netdb.h> // getaddrinfo()
#include <unistd.h> // close()
#include <string.h> // memset()
#include <stdlib.h> // exit()

#define _CRT_SECURE_NO_WARNINGS // To prevent compile error due to fopen security issue [?]

#define MAXPORT 6 // The range of port number is 0 to 65535
#define BACKLOG 10 // The maximum number of pending connections queue that will be held

//#define TEST_SERVER


// For handling multiple processing; Not needed in PHAROS
void sigchld_handler(int s){
  // waitpid() might overwrite errno, so we save
  int saved_errno = errno;
  while(waitpid(-1, NULL, WNOHANG) > 0);
  errno = saved_errno;
}

void *v2x_main(void *arg); // to operate original main code (Handle msgs & transmission)
void *v2x_pvd(void *arg); // to handle TCP server which updates pvd data
struct PVDdatafromROS pvd; // PVD data (shared for each thread)
pthread_mutex_t mutex; // To create mutex(locking) system

void sig_handler(int signo)
{
  /*
  if (signo == SIGINT)
    fprintf(stderr, "SIGINT! \n");
  */
  //fflush();
  exit(EXIT_SUCCESS);
}

unsigned long long get_clock_time(){  
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);  
    uint64_t clock = ts.tv_sec * 1000 + (ts.tv_nsec / 1000000); // unit: ms
    return clock;
}  
 
int connect_obu_uper_tcp(char*ip, unsigned short port){

    int sockFd = -1;
    // 소켓 파일디스크립터 생성
    sockFd = socket(PF_INET, SOCK_STREAM, 0);
    
    // 소켓 생성이 실패 여부 확인
    if (sockFd < 0)
    {
        printf("DEBUG : step sock create error\n");
        return -1;
    }

    // TCP 서버 주소, 포트 입력
    struct sockaddr_in addr; 
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(ip);
    addr.sin_port = htons(port);

    // TCP 서버 연결 
    if (connect(sockFd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        printf("DEBUG : step sock connect error\n");
        close(sockFd);
        return -1;
    }

    // 논 블록킹 소켓으로 동작
    int flag = fcntl(sockFd, F_GETFL, 0);
    fcntl(sockFd, F_SETFL, flag | O_NONBLOCK);

    printf("DEBUG : OBU TCP [%s:%d] Connected \n",ip,port); // TCP 접속 완료 

    return sockFd;
}


/*
 * sockFd: socket file descriptor
 * buffer: received data
 * storedsize: data size already stored in buffer
 * uperBuffer: uper payload part (if exists, update)
 * uperBufferSize: WTF?? [QUESTION] MAX_UPER_SIZE WHY?
 * uperRes: uper payload size to handle (only RX_WAVE_UPER case, otherwise, 0)
 * return value : -1 (No received data), stored size
 */
int receive_from_obu(int sockFd, char *buffer, unsigned short bufferSize, int storedSize,
                     char *uperBuffer, unsigned short uperBufferSize, int *uperRes, FILE *fp)
{
    int rxSize = -1;
    *uperRes = 0;
    if(sockFd < 0) // if ther is no socket connection
        return rxSize;
    int leftBufferSize = bufferSize - storedSize; // left buffer size
    if(leftBufferSize <= 0) // if there is no left buffer size -> deadlock (-1)
    {
      fprintf(stderr, "socket receive deadlock\n"); 
      return -1;
    }
    rxSize = read(sockFd, buffer + storedSize, leftBufferSize); // size of data read from obu
    //read 리턴값이 0 인 경우, Socket 로부터 Read가 불가능한 상태를 의미하며,
    //대표적으로 소켓 연결이 종료된 상태가 있음
    //리턴값이 -1 인 경우, non-blocking 소켓에 대해 수신된 데이터가 존재하지 않는 경우를 의미
    if (rxSize == 0) // if TCP fails
       return -1; 
    if(rxSize < 0 && storedSize == 0) // if there is no received data & no stored data
        return 0;
    if(rxSize > 0) // if receive data
      storedSize += rxSize; // update stored size (stored + received)
    // stored data + No received data | stored data + received data
    
    // CEST OBU TCP 헤더 파싱 부문
    int headerByteLen = sizeof(struct CestObuUperPacketHeader); // OBU TCP Header
    int packetLen = headerByteLen; // current packetlen = headerBytelen
    if(storedSize < headerByteLen){ // if stored data is shorter than header bytes
        return storedSize;
    }
    
    struct CestObuUperPacketHeader header; // header variable
    memcpy(&header,buffer,headerByteLen); // copy header part from [buffer] to [header]
    packetLen += header.payloadLen; // packetlen = header len + payload len

    if (storedSize < packetLen) // if storedSize is smaller than packet len (not complete) return it
        return storedSize; 

    memcpy(uperBuffer,buffer + headerByteLen,header.payloadLen); // copy UPER payload to uperBuffer
    *uperRes = header.payloadLen; // store length of UPER data
    
    storedSize -= packetLen; // update stored size
    memcpy(buffer,buffer+packetLen,storedSize); // update buffer (rest part of data)

    if(header.messageType == 0x3411){ // if header is Tx_WAVE_UPER_RESULT (ACK)
        struct TxWaveUperResultPayload *payload = (struct TxWaveUperResultPayload*)uperBuffer;
	// now uper is Tx_WAVE_UPER_RESULT form
        fprintf(fp, "RX - \"TX_WAVE_UPER_ACK\" [%d/%d/%d]\n",payload->txWaveUperSeq,payload->resultCode
	       ,payload->size);
	// print out the members of Tx_WAVE_UPER_RESULT
        *uperRes = 0;
	
    }else 
      fprintf(fp, "RX - \"RX_WAVE_UPER\" [%d] \n",header.payloadLen); 

    return storedSize; // return rest stored data size
}


/**
 * @brief TX_WAVE_UPER 메시지 전송 : PVD TRANSFER!
 * @param sockFd OBU Client socket 
 * @param uper   UPER 인코딩된 MessageFrame
 * @param uperLength  UPER 인코딩된 MessageFrame의 Byte 길이
 * @return int  -1 : 전송 실패 , >0 전송된 바이트 수
 */
int request_tx_wave_obu(int sockFd, char *uper,unsigned short uperLength, FILE *fp){

    if(sockFd < 0)
        return -1;

    int packetLen = uperLength + sizeof(struct CestObuUperPacketHeader);
    char packet[OBU_RECEIVE_BUFFER_SIZE];  // tcp header size + uper binary size
    // update ptrHeader part
    struct CestObuUperPacketHeader *ptrHeader = (struct CestObuUperPacketHeader *)&packet[0];
    ptrHeader->messageType = 0x4311; // TX_WAVE_UPER
    ptrHeader->seq = packetSeq++; // update sequence number, global variable
    ptrHeader->payloadLen = uperLength; // uperlength (param)
    ptrHeader->deviceType = 0xCE; // fixed
    memcpy(ptrHeader->deviceId,clientDeviceId,3); // vehicle system ID [QUESTION] Default??
    memcpy(packet + sizeof(struct CestObuUperPacketHeader), uper, uperLength); // copy uper data part
    int res = write(sockFd,packet,packetLen); // write!
    if (res > 0)
    {
      fprintf(fp, "TX - \"TX_WAVE_UPER\" SEQ[%d] = ", ptrHeader->seq);
      print_hex(uper,uperLength, fp);
      
      if (res != packetLen)
        {            
	  printf("DEBUG tcp tx purge\n"); // write fail (did not send all data, only send part of it)
	  return -1;
        }else
	return res; // size of written data
    }
}

/**
 * @brief PVD 메시지 생성 및 WAVE 전송  = PVD Create
 * @param sockFd   OBU Client socket 
 * @param time     PVD 전송 Timer Pointer
 * @return int     -1 : 전송 실패, 0 : there is no data transmitted
 */
int tx_v2i_pvd(int sockFd, unsigned long long *time, FILE *fp, FILE *fp_pvd, struct PVDdatafromROS pvd)
{ 
    unsigned long long interval = get_clock_time() - *time;  // msec;
    if(interval < PVD_INTERVAL) // PVD_INTERVAL: 1sec -> PVD_Transfer_Period [CHECK]
        return 0;
    *time += (interval - interval%PVD_INTERVAL); // To control period as multiple of 0.2s
    // [QUESTION] what about if interval is larger than 200ms??
    // ex. time: 0, gettime: 430ms -> time: 400, gettime: 30(passed already!)
    MessageFrame_t msg; // PVD msg (before UPER encoding)
    char uper[MAX_UPER_SIZE]; 

    fill_j2735_pvd(&msg, fp_pvd, pvd); // current pvd data [QUESTION] where this data comes from? A. ROS!
    int encodedBits = encode_j2735_uper(uper,MAX_UPER_SIZE,&msg); // encode msg to uper
    if(encodedBits < 0) // 인코딩 실패로 전송이 불가능한 상태
        return 0;
    int byteLen = encodedBits / 8 + ((encodedBits % 8)? 1:0);
    //print_hex(uper,byteLen);    
    return request_tx_wave_obu(sockFd,uper,byteLen,fp); // Transmit!!
}


// we do not need BSM this year ====================================================>
int tx_v2v_bsm(int sockFd, unsigned long long *time, FILE *fp){

    unsigned long long interval = get_clock_time() - *time;
 
    if(interval < BSM_INTERVAL)
        return 0; 
    
    *time += (interval - interval%BSM_INTERVAL);

    MessageFrame_t *msg = malloc(sizeof(MessageFrame_t)); 
    char uper[MAX_UPER_SIZE]; 

    fill_j2735_bsm(msg);
       
    int encodedBits = encode_j2735_uper(uper,MAX_UPER_SIZE,msg);
    
    if (encodedBits < 0) 
        return 0;
  
    int byteLen = encodedBits / 8 + ((encodedBits % 8)? 1:0);

    //print_hex(uper, byteLen);
 
    return request_tx_wave_obu(sockFd, uper, byteLen, fp);
} 
// we do not need this here ====================================================>


int main(void)
{
  memset(&pvd, 0, sizeof(struct PVDdatafromROS)); // Initialize PVD data
  pthread_t main_, tcp_; // threads
  pthread_mutex_init(&mutex, NULL); // Initialize the mutex
  // Thread1
  int main_id = pthread_create(&main_, NULL, v2x_main, NULL);
  if(main_id < 0){
    perror("thread create error : ");
    exit(EXIT_FAILURE);
  }
  // Thread2
  int tcp_id = pthread_create(&tcp_, NULL, v2x_pvd, NULL);
  if(tcp_id < 0){
    perror("thread create error : ");
    exit(EXIT_FAILURE);
  }
  //int tcp_server = pthread_create(&tcp_server, NULL, v2x_)
  pthread_join(main_, NULL); // Wait thread [main_]
  pthread_join(tcp_, NULL); // Wait thread [tcp_]
  pthread_mutex_destroy(&mutex); // Destroy the mutex system
  return 1;
}


void *v2x_main(void *arg)
{
  //--------------------------------------------------------------------------------------
  int sockFd = -1;  // OBU와 연결하기 위한 TCP Client 소켓 File descriptor
  char rxBuffer[OBU_RECEIVE_BUFFER_SIZE] , rxUperBuffer[MAX_UPER_SIZE];
  int storedSize = 0, uperSize;
  unsigned long long txPvd = get_clock_time(), txBsm = get_clock_time();
  
  /* Open Log File*/
  FILE *fp = fopen("V2X_LOG.txt", "w");
  FILE *fp_pvd = fopen("V2X_LOG_PVD.txt", "w");
  /* Set new sig_handler for SIGINT (NOT USED)*/
  if (signal(SIGINT, sig_handler) == SIG_ERR)
    printf("\ncan't catch SIGINT\n");
  
  while(1){    
    // 소켓이 연결되지 않은 경우(sockFd == -1) , OBU TCP 소켓 연결 시도!
    if(sockFd < 0)
      {
  // 192.168.10.10 [OBU], 118.45.183.36 [V2X TEST SERVER]
#ifdef TEST_SERVER
        sockFd = connect_obu_uper_tcp("118.45.183.36",23000);
#else
        sockFd = connect_obu_uper_tcp("192.168.10.10",23000);
#endif
	storedSize = 0;
	if(sockFd < 0){
	  fprintf(stderr, "DEBUG : connect failed, retry\n");
	  sleep(1);
	  continue;
	} 
      }
    // 소켓이 연결된 상태인 경우, OBU로부터 TCP 패킷 수신
    // WAVE 통신으로 수신된 데이터가 없을 경우, 수신되는 데이터 X
    // Sample Code Operation Process (Figure!) --> receive data -> BSM -> PVD
    // receive_from_obu : received data from OBU/ tx_v2v_bsm & tx_v2i_pvd : transmit BSM & PVD msgs
    pthread_mutex_lock(&mutex);
    if (((storedSize = receive_from_obu(sockFd,
					rxBuffer, OBU_RECEIVE_BUFFER_SIZE, storedSize,
					rxUperBuffer, MAX_UPER_SIZE, &uperSize,
					fp)) < 0) ||
	(tx_v2v_bsm(sockFd,&txBsm, fp) < 0) ||
	(tx_v2i_pvd(sockFd,&txPvd, fp, fp_pvd, pvd) < 0))
      {
	// OBU와 TCP 연결이 끊어진 경우, 연결 재시도
	close(sockFd);
	sockFd = -1;
	continue;
      }
    pthread_mutex_unlock(&mutex);
    // decode if there is any received data
    if (uperSize > 0)
      {
	// OBU로부터 수신된 WAVE 메시지가 존재할 경우, UPER 디코딩 -> J2735 메시지 파싱
	MessageFrame_t *msgFrame = NULL;
	decode_j2735_uper(msgFrame, rxUperBuffer, uperSize, fp); // [FILE]
	ASN_STRUCT_FREE(asn_DEF_MessageFrame, msgFrame);
      }
    usleep(1000); //1msec sleep
  }
  fclose(fp);
  close(sockFd);
  // --------------------------------------------------------------------------------------------- 
}

void *v2x_pvd(void *arg)
{
  // 1. Variables for TCP connection
  int sockfd, new_fd; // listen on sockfd, new connection on new_fd
  struct addrinfo hints, *servinfo, *p; // hints for setting, servinfo: server info, p: for iteration
  struct sockaddr_storage their_addr; // Connector's address information
  socklen_t sin_size; // to use accept()
  struct sigaction sa; // to handle multiple processing
  int yes = 1; // to avoid annoying reuse-error
  int rv; // for error checking
  struct PVDdatafromROS rmsg; // msg struct to recv
  
  // 2. Set IP & Portnumber ===============================>>>> Manually!
  char port[MAXPORT] = "7684";
  // Get addr info
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET; // IPv4
  hints.ai_socktype = SOCK_STREAM; // TCP 
  hints.ai_flags = AI_PASSIVE; // use my IP (Local operating V2X)
  // store server addr info to ther servinfo
  if(rv = getaddrinfo(NULL, port, &hints, &servinfo) != 0){ // NULL when flags: AI_PASSIVE
    fprintf(stderr, "[Error] Server: getaddrinfo: %s\n",gai_strerror(rv));
    exit(EXIT_FAILURE);
  }
  // loop through all the results and bind to the first one we can
  for(p = servinfo; p != NULL; p = p->ai_next){
    // socket
    if((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol))== -1)
      {
	fprintf(stderr, "[Error] Server: socket: %d, %s\n",
		errno, strerror(errno));
	continue;
      }
    // to avoid annoying reuse-error
    if(setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1){
      fprintf(stderr, "[Error] Server: setsockopt\n");
      exit(EXIT_FAILURE);
    }
    // bind
    if(bind(sockfd, p->ai_addr, p->ai_addrlen) == -1){
      close(sockfd); // if bind fails
      fprintf(stderr, "[Error] Server: bind: %d, %s\n",
	      errno, strerror(errno));
      continue;
    }
    break;
  }
  freeaddrinfo(servinfo); // all done with this structure
  if (p == NULL){ // if fail to bind
    fprintf(stderr, "[Error] Server: failed to bind\n");
    exit(EXIT_FAILURE);
  }
  
  if (listen(sockfd, BACKLOG) == -1){
    fprintf(stderr, "[Error] Server: failed to listen\n"); // if fail to listen
    exit(EXIT_FAILURE);
  }
  
  // set new handler to reap all dead processes
  sa.sa_handler = sigchld_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_RESTART;
  if(sigaction(SIGCHLD, &sa, NULL) == -1){
    fprintf(stderr, "[Error] Server: sigaction\n");
    exit(EXIT_FAILURE);
  }

  while(1){ // main accept() loop
    sin_size = sizeof their_addr;
    new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &sin_size);
    if(new_fd == -1){ // if fail to accept
      fprintf(stderr, "[Error] Server: accept : %d, %s\n", errno, strerror(errno));
      continue;
    }
    memset(&rmsg, 0, sizeof(struct PVDdatafromROS)); // initialize rmsg buffer
    if((rv = recv(new_fd, &rmsg, sizeof(struct PVDdatafromROS), 0)) == -1){ // if fail to recv
      fprintf(stderr, "[Error] Server: recv: %d, %s\n", errno, strerror(errno));
      continue;
    }
    if(rv < sizeof(struct PVDdatafromROS)){
      continue;
    }
    // Update PVD information!
    pthread_mutex_lock(&mutex);
    pvd.longitude = rmsg.longitude;
    pvd.latitude = rmsg.latitude;
    pvd.elevation = rmsg.elevation;
    pvd.heading = rmsg.heading;
    pvd.speed = rmsg.speed;
    pvd.transmission = rmsg.transmission;
    pthread_mutex_unlock(&mutex);
    close(new_fd);
  }
  close(sockfd);
  return (void *)1;
}

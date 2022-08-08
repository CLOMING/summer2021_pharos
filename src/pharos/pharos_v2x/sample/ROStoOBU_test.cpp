/*
CAN_GATEWAY
subscriber
ROS package
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <assert.h>
#include <time.h>

#define ROS_INTERVAL 50 // 50ms
#define PAUSE 10000 // 10ms

typedef enum _boolean {
    FALSE,
    TRUE
} boolean;

/* struct to store PVD data from ROS */
struct PVDdata2Local{
  long longitude;
  long latitude;
  long elevation;
  long heading;
  long speed;
  long transmission;
}__attribute__((packed));

/*
 * [get_clock_time]
 * return current time in the unit of ms
 */
unsigned long long get_clock_time(){  
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);  
    uint64_t clock = ts.tv_sec * 1000 + (ts.tv_nsec / 1000000); // unit: ms
    return clock;
}  

/*
 * [sendall]
 * send all msg to the sock with sockfd
 * return -1 on failure, 0 on success
 * If we need to wait to send, do not wait and return -1 immediately.
 */
int sendall(int sockfd, const void *msg, int len){
  assert(sockfd >= 0);
  assert(msg != NULL);
  int total = 0;
  int byteleft = len;
  int n;
  
  while (byteleft > 0)
    {
      if ((n = send(sockfd, msg+total, byteleft, MSG_DONTWAIT)) == -1)
        {
	  return -1;
	}
      total += n;
      byteleft -= n;
    }
  return 0;
}

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int main(int argc, char *argv[])
{
  /* Handle in the callback function & set period as 5Hz!*/
  // 1. Variables
  int sockfd, portno, n;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  struct PVDdata2Local pvd;
  pvd.longitude = 0;
  pvd.latitude = 1;
  pvd.elevation = 2;
  pvd.heading = 3;
  pvd.speed = 4;
  pvd.transmission = 5;
  int count = 0; // number of connection
  unsigned long long time_now, time_prev, interval;
  time_prev = get_clock_time();
  while(TRUE){
    // Set trasmission period as 50ms
    time_now = get_clock_time();
    interval = time_now - time_prev;
    if(interval < ROS_INTERVAL)
      {
      	// if too fast, pause
      	usleep(PAUSE);
      	continue;
      }
    // 2. Set PVD data
    pvd.longitude++;
    pvd.latitude++;
    pvd.elevation++;
    pvd.heading++;
    pvd.speed++;
    //pvd.transmission++;
    // 3. Set port number and IP address
    portno = atoi("7684"); // Fixed port number: 7684
    server = gethostbyname("localhost"); // Fixed ip address (local computer operating V2X)
    if (server == NULL) { // if hostname is invalid
      fprintf(stderr,"ERROR, no such host\n");
      exit(EXIT_FAILURE);
    }
    memset((char *)&serv_addr, 0, sizeof(serv_addr)); // initialize address
    serv_addr.sin_family = AF_INET; // IPv4
    memcpy((char *)&serv_addr.sin_addr.s_addr, (char *)server->h_addr, server->h_length); // set address
    serv_addr.sin_port = htons(portno); // set port #
    // 4. Create socket!
    sockfd = socket(AF_INET, SOCK_STREAM, 0); // AF_INET: IPv4, SOCK_STREAM: TCP/IP, 0: default
    if (sockfd < 0)
    {
      error("ERROR opening socket"); // If creating socket is failed
      continue;
    }
    // 5. Connect to the server in OBU (sample.c)
    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
    	error("ERROR connecting"); // if connection fails
    	continue;
    }
    // 6. Now, connected! Send all of them immediately with no blocking
    count++;
    printf("ROStoOBU: TCP Connection from ROS to OBU is esbalished: %d\n", count);
    if(sendall(sockfd, &pvd, sizeof(struct PVDdata2Local)) < 0)
    {
      error("ERROR writing to socket"); // if sendall() fails
      continue;
    }
    time_prev = time_now - (interval%ROS_INTERVAL); // Update previous time with considering period (50ms)
    close(sockfd);
  }
  return 0;
}

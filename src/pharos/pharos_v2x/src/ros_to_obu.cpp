#include <ros/ros.h>
#include <ublox_msgs/NavPVT.h>
#include <pharos_msgs/CAN_GWAY_header.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <assert.h>

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

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


class ROS_to_OBU {
private:
    ros::NodeHandle nh; 
    
public:
    ros::Subscriber subPVD;
    
    ros::Subscriber subUbloxFix;
    ros::Subscriber subUbloxNavpvt;
    ros::Subscriber subCanGateway;

    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    struct PVDdata2Local pvd;

    bool pvd_CB1_init_ = false;
    bool pvd_CB2_init_ = false;

    ROS_to_OBU(){

      subUbloxNavpvt = nh.subscribe<ublox_msgs::NavPVT>("/ublox/navpvt", 1, &ROS_to_OBU::pvd_CB1, this);
      subCanGateway = nh.subscribe<pharos_msgs::CAN_GWAY_header>("/CAN_Gateway", 1, &ROS_to_OBU::pvd_CB2, this);
      
    }

private:
    int count = 0; // number of connection

public: 
    void pvd_CB1(const ublox_msgs::NavPVT msg)
    {
        if(pvd_CB1_init_ == false){
            ROS_WARN("ublox_msgs::NavPVT INIT");
            pvd_CB1_init_ = true;
        }
        pvd.longitude = msg.lon;
        pvd.latitude = msg.lat;
        pvd.elevation = msg.height/1000. /0.1; // {height}/ mm2m / [0.1m]
        pvd.heading = msg.heading/10000. / 0.125; // {heading} / [10^4deg] / [0.0125deg]
    }

    void pvd_CB2(const pharos_msgs::CAN_GWAY_header msg)
    {
        if(pvd_CB2_init_ == false){
            ROS_WARN("ublox_msgs::NavPVT INIT");
            pvd_CB2_init_ = true;
        }
        pvd.speed = ((msg.GWAY1.Wheel_Velocity_RL + msg.GWAY1.Wheel_Velocity_RL) / 2.) / 3.6 / 0.02; // {Rear wheel mean vel} / kph2mps / [0.02m/s]
        pvd.transmission = msg.GWAY3.GearSelDisp;
    }

    void tcp_send(){
        if(pvd_CB1_init_ == false || pvd_CB2_init_ == false) return;

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
            return;
        }
        // 5. Connect to the server in OBU (sample.c)
        if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        {
            error("ERROR connecting"); // if connection fails
            return;
        }
        // 6. Now, connected! Send all of them immediately with no blocking
        count++;
        printf("ROStoOBU: TCP Connection from ROS to OBU is esbalished: %d\n", count);
        if(sendall(sockfd, &pvd, sizeof(struct PVDdata2Local)) < 0)
        {
            error("ERROR writing to socket"); // if sendall() fails
            return;
        }
        close(sockfd);
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ROStoOBU_node");

    ROS_to_OBU ros2obu;

    ros::Rate r(5); // 5 hz
    while (1)
    {
        ros2obu.tcp_send();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

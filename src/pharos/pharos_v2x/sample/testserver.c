#include <stdio.h>
#include <errno.h> // errno
#include <sys/socket.h> // socket functions
#include <sys/types.h> // waitpid()
#include <sys/wait.h> // waitpid()
#include <netinet/in.h> // sock structures
#include <netdb.h> // getaddrinfo()
#include <unistd.h> // close()
#include <string.h> // memset()
#include <stdlib.h> // exit()

#define MAXPORT 6 // The range of port number is 0 to 65535
#define BACKLOG 10 // The maximum number of pending connections queue that will be held




/* struct to store PVD data from ROS */
struct PVDdata2Local{
  long longitude;
  long latitude;
  long elevation;
  long heading;
  long speed;
  long transmission;
}__attribute__((packed));

void sigchld_handler(int s){
  // waitpid() might overwrite errno, so we save
  int saved_errno = errno;
  while(waitpid(-1, NULL, WNOHANG) > 0);
  errno = saved_errno;
}

int main(){
  int sockfd, new_fd; // listen on sockfd, new connection on new_fd
  struct addrinfo hints, *servinfo, *p; // hints for setting, servinfo: server info, p: for iteration
  struct sockaddr_storage their_addr; // Connector's address information
  socklen_t sin_size; // to use accept()
  struct sigaction sa; // to handle multiple processing
  int yes = 1; // to avoid annoying reuse-error
  int rv; // for error checking
  struct PVDdata2Local rmsg; // msg struct to recv
  int count = 0;
  
  // Set Port number manually (7684)
  char port[MAXPORT] = "7684";
  // Get addr info
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET; // IPv4
  hints.ai_socktype = SOCK_STREAM; // TCP 
  hints.ai_flags = AI_PASSIVE; // use my IP (Local operating V2X)
  // store server addr info to ther servinfo
  if(rv = getaddrinfo(NULL, port, &hints, &servinfo) != 0){ // NULL when flags: AI_PASSIVE
    fprintf(stderr, "[Error] Server: getaddrinfo: %s\n",gai_strerror(rv));
    return 0;
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
    return 0;
  }
  
  if (listen(sockfd, BACKLOG) == -1){
    fprintf(stderr, "[Error] Server: failed to listen\n"); // if fail to listen
    return 0;
  }
  
  // set new handler to reap all dead processes
  sa.sa_handler = sigchld_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_RESTART;
  if(sigaction(SIGCHLD, &sa, NULL) == -1){
    fprintf(stderr, "[Error] Server: sigaction\n");
    return 0;
  }

  while(1){ // main accept() loop
    sin_size = sizeof their_addr;
    new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &sin_size);
    if(new_fd == -1){ // if fail to accept
      fprintf(stderr, "[Error] Server: accept : %d, %s\n", errno, strerror(errno));
      return 0;
    }
    memset(&rmsg, 0, sizeof(struct PVDdata2Local)); // initialize rmsg buffer
    if((rv = recv(new_fd, &rmsg, sizeof(struct PVDdata2Local), 0)) == -1){ // if fail to recv
      fprintf(stderr, "[Error] Server: recv: %d, %s\n", errno, strerror(errno));
      return 0;
    }
    if(rv < sizeof(struct PVDdata2Local)){
      continue;
    }
    count++;
    printf("========================================================\n");
    printf("Received PVD data: #%d\n", count);
    printf("Longitude: %ld\n", rmsg.longitude);
    printf("Latitude: %ld\n", rmsg.latitude);
    printf("Elevation: %ld\n", rmsg.elevation);
    printf("Heading: %ld\n", rmsg.heading);
    printf("Speed: %ld\n", rmsg.speed);
    printf("Transmission: %ld\n", rmsg.transmission);
    close(new_fd);
  }
  close(sockfd);
  return 1;
}

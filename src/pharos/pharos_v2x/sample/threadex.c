#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>


void *addTarget(void *arg);
void *subtractTarget(void *arg);
int target = 0;
pthread_mutex_t mutex; // To create mutex(locking) system


void *addTarget(void *param)
{
  int limit = *(int *)param;
  int i;
  for (i=0; i< limit; i++){
    pthread_mutex_lock(&mutex);
    target++;
    printf("[ADD] target : %d\n", target);
    pthread_mutex_unlock(&mutex);
  }
}

void *subtractTarget(void *param){
  int limit = *(int *)param;
  int i;
  
  for (i=0; i < limit; i++)
    {
      pthread_mutex_lock(&mutex);
      target--;
      printf("[SUBTRACT] target : %d\n", target);
      pthread_mutex_unlock(&mutex);
    }
}


int main(){
  pthread_t add, sub;
  int param = 100;

  pthread_mutex_init(&mutex, NULL); // Initialize the mutex
  // Thread 1
  int add_id = pthread_create(&add, NULL, addTarget, &param);
  if(add_id < 0){
    perror("thread create error : ");
    exit(EXIT_FAILURE);
  }
  // Thread 2
  int sub_id = pthread_create(&sub, NULL, subtractTarget, &param);
  if(sub_id < 0){
    perror("thread create error : ");
    exit(EXIT_FAILURE);
  }
  pthread_join(add, NULL); // Wait thread [add]
  pthread_join(sub, NULL); // Wait thread [sub]
  //pthread_detach(add); // Destroy thread after finished [add]
  //pthread_detach(sub); // Destroy thread after finished [sub]
  printf("target: %d\n", target); // Final result of target
  pthread_mutex_destroy(&mutex); // Destroy the mutex system
  return 0;
}

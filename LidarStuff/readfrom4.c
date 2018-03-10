#include "vl6180_pi.c"
#include <time.h>

struct timespec start, stop; // init clock_gettime variables

void getdist(handle0){ //, handle2, handle3, handle4){
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start); // Start timing
  int dist = get_distance(handle0); // Get distance
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &stop); //Stop Timing
  double result = (stop.tv_sec - start.tv_sec) * 1e6 + (stop.tv_nsec - start.tv_nsec) / 1e3; // TODO figure out why time is in thousands
  printf("DIstance: %dmm       %e        \n ", dist, result); //Display
}

int main(){

  vl6180 handle;

  printf("starting to test...\n");
  
  vl6180 handle0 = vl6180_initialise_address(1, 0x21);
  if(handle0<=0){
    printf("0 not initiated");  
    return 1;
  }
  vl6180 handle1 = vl6180_initialise_address(1, 0x22);
  if(handle1<=0){
    printf("1 not initiated");
    return 1;
  }
  vl6180 handle2 = vl6180_initialise_address(1, 0x23);
  if(handle2<=0){
    printf("2 not initiated");
    return 1;
  }
  vl6180 handle3 = vl6180_initialise_address(1, 0x24);
  if(handle3<=0){
    printf("3 not initiated");
    return 1;
  }
  
  int distance0 = get_distance(handle0);
  int distance1 = get_distance(handle1); 
  int distance2 = get_distance(handle2); 
  int distance3 = get_distance(handle3); 


  printf(" %d %d %d \n", distance1, distance2, distance3); 

  int x = 100;
  while(x > 0){ // Get distances for a short time.
    distance0 = get_distance(handle0);
    distance1 = get_distance(handle1); 
    distance2 = get_distance(handle2); 
    distance3 = get_distance(handle3); 

    x = x - 1;
    printf(" %d %d %d %d \n", distance0, distance1, distance2, distance3); 
  }

  /*
  if ((handle = vl6180_initialise(1)) < 0){
   return handle;
  }

  vl6180_change_addr(handle, 0x24);

  //set_scaling(handle, 1); It is scale 1 by default

  int distance = get_distance(handle);

  printf("Distance: %d \n", distance);

*/
  return 0;
}

#include "vl6180_pi.c"

int main(){

  vl6180 handle;

  if ((handle = vl6180_initialise(1)) < 0){
   return handle;
  }

  vl6180_change_addr(handle, 0x21);
  
  //set_scaling(handle, 1); It is scale 1 by default 

  int distance = get_distance(handle);

  printf("Distance: %d \n", distance);
  
  return 0;
}

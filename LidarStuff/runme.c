#include "vl6180_pi.c"

int main(){

  vl6180 handle = vl6180_initialise(1);

  vl6180_change_addr(handle, 0x24);

  int distance = get_distance(handle);

  printf("Distance: %d \n", distance);
  
  return 0;
}

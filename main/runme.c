#include "../vl6180_pi.c"

int main(){

  vl6180 handle; // Define the vl6180 type

  if ((handle = vl6180_initialise(1)) < 0){
    printf("Error: %d \n", handle);
    return handle; // If it doesn't initialize, return the error code
  }

  vl6180_change_addr(handle, 0x23);

  int distance = get_distance(handle); // Test getting a distance reading
  printf("Distance: %d \n", distance);
  
  return 0;
}

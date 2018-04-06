/*
Exposes these functions:
	void init()				-- call this at the start

	double servo_angle_ne()	-- returns angle of ne servo in revolutions (radians / 2pi)
	double servo_angle_nw()	-- returns angle of nw servo in revolutions (radians / 2pi)
	double servo_angle_sw()	-- returns angle of sw servo in revolutions (radians / 2pi)
	double servo_angle_se()	-- returns angle of se servo in revolutions (radians / 2pi)
	
	int lidar_distance_north()	-- distance in millimeters
	int lidar_distance_west()
	int lidar_distance_south()
	int lidar_distance_east()

	void terminate()		-- call before exiting
*/

#include <iostream>

#include "pigpiod_if2.h"
#include "vl6180_pi.c"

constexpr double DUTY_CYCLE_MIN = 0.029;
constexpr double DUTY_CYCLE_MAX = 0.971;

constexpr int SERVO_OUTPUT_NE = 4;
constexpr int SERVO_OUTPUT_NW = 17;
constexpr int SERVO_OUTPUT_SW = 22;
constexpr int SERVO_OUTPUT_SE = 10;

constexpr int SERVO_INPUT_NE = 27;
constexpr int SERVO_INPUT_NW = 9;
constexpr int SERVO_INPUT_SW = 11;
constexpr int SERVO_INPUT_SE = 5;

constexpr int LIDAR_OUTPUT_NORTH = 13;
constexpr int LIDAR_OUTPUT_WEST = 6;
constexpr int LIDAR_OUTPUT_SOUTH = 26;
constexpr int LIDAR_OUTPUT_EAST = 19;

constexpr int LIDAR_NORTH	= 0x21;
constexpr int LIDAR_WEST	= 0x20;
constexpr int LIDAR_SOUTH	= 0x23;
constexpr int LIDAR_EAST	= 0x22;

double clamp(double x, double min, double max)
{
	if (x < min)
		return min;
	if (x > max)
		return max;
	return x;
}

struct servo_feedback_reader
{
	uint32_t last_pulse_started;
	uint32_t last_pulse_ended;
	double duty_cycle;

	double angle_in_revolutions();
};

double servo_feedback_reader::angle_in_revolutions()
{
	duty_cycle = clamp(duty_cycle, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);
	return (duty_cycle - DUTY_CYCLE_MIN) / (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN);
}

template<servo_feedback_reader& reader>
void feedback_state_changed(int pi, unsigned int gpio, unsigned int level, uint32_t tick)
{
	if (level == 1) {
		reader.duty_cycle = (double) (reader.last_pulse_ended - reader.last_pulse_started) / (tick - reader.last_pulse_started);
		reader.last_pulse_started = tick;
	}
	else if (level == 0) {
		reader.last_pulse_ended = tick;
	}
}

int pi_handle;

vl6180 reset_lidar(int gpio_address, int i2c_address)
{
	gpio_write(pi_handle, gpio_address, 0);
	gpio_write(pi_handle, gpio_address, 1);
  
  vl6180 handle;
  printf("initing %d on %d", gpio_address, i2c_address);

  if ((handle = vl6180_initialise(1)) < 0){ // Initialize lidar on 0x29 (default)
    printf("Error: %d \n", handle);
    return handle; // If it doesn't initialize, return the error code 
  }
  printf("initted");
  vl6180_change_addr(handle, i2c_address); // Change its address*/
  printf("returning");

  int distance = get_distance(handle); // Test getting a distance reading
  printf("Distance: %d \n", distance);

  return handle;
//	return vl6180_initialise_address(1, i2c_address);
  }

servo_feedback_reader feedback_ne;
servo_feedback_reader feedback_nw;
servo_feedback_reader feedback_sw;
servo_feedback_reader feedback_se;

vl6180 lidar_north;
vl6180 lidar_west;
vl6180 lidar_south;
vl6180 lidar_east;

extern "C" {
	void init()
	{
		pi_handle = pigpio_start(0, 0);
		set_mode(pi_handle, SERVO_OUTPUT_NE, PI_OUTPUT);
		set_mode(pi_handle, SERVO_OUTPUT_NW, PI_OUTPUT);
		set_mode(pi_handle, SERVO_OUTPUT_SW, PI_OUTPUT);
		set_mode(pi_handle, SERVO_OUTPUT_SE, PI_OUTPUT);

		set_mode(pi_handle, SERVO_INPUT_NE, PI_INPUT);
		set_mode(pi_handle, SERVO_INPUT_NW, PI_INPUT);
		set_mode(pi_handle, SERVO_INPUT_SW, PI_INPUT);
		set_mode(pi_handle, SERVO_INPUT_SE, PI_INPUT);

		callback(pi_handle, SERVO_INPUT_NE, EITHER_EDGE, feedback_state_changed<feedback_ne>);
		callback(pi_handle, SERVO_INPUT_NW, EITHER_EDGE, feedback_state_changed<feedback_nw>);
		callback(pi_handle, SERVO_INPUT_SW, EITHER_EDGE, feedback_state_changed<feedback_sw>);
		callback(pi_handle, SERVO_INPUT_SE, EITHER_EDGE, feedback_state_changed<feedback_se>);
/*
		lidar_north = reset_lidar(LIDAR_OUTPUT_NORTH, LIDAR_NORTH);
		lidar_west = reset_lidar(LIDAR_OUTPUT_WEST, LIDAR_WEST);
		lidar_south = reset_lidar(LIDAR_OUTPUT_SOUTH, LIDAR_SOUTH);
		lidar_east = reset_lidar(LIDAR_OUTPUT_EAST, LIDAR_EAST);
*/
    printf("running addresses"); 
		lidar_north = vl6180_initialise_address(1, LIDAR_NORTH);
    if(lidar_north<=0){
      printf("1 not initiated");
    }
		lidar_south = vl6180_initialise_address(1, LIDAR_SOUTH);
		lidar_east = vl6180_initialise_address(1, LIDAR_EAST);
		lidar_west = vl6180_initialise_address(1, LIDAR_WEST);
	}

	double servo_angle_ne() { return feedback_ne.angle_in_revolutions(); }
	double servo_angle_nw() { return feedback_nw.angle_in_revolutions(); }
	double servo_angle_sw() { return feedback_sw.angle_in_revolutions(); }
	double servo_angle_se() { return feedback_se.angle_in_revolutions(); }

	int lidar_distance_north() { return get_distance(lidar_north);}
	int lidar_distance_west() { return get_distance(lidar_west); }
	int lidar_distance_south() { return get_distance(lidar_south); }
	int lidar_distance_east() { return get_distance(lidar_east); }

	void terminate() { pigpio_stop(pi_handle); }
}

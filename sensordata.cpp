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

#include "pigpiod_if2.h"
#include "vl6180_pi.h"

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

constexpr int LIDAR_NORTH	= 22;
constexpr int LIDAR_WEST	= 21;
constexpr int LIDAR_SOUTH	= 24;
constexpr int LIDAR_EAST	= 23;

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

servo_feedback_reader feedback_ne;
servo_feedback_reader feedback_nw;
servo_feedback_reader feedback_sw;
servo_feedback_reader feedback_se;

vl6180 lidar_north;
vl6180 lidar_west;
vl6180 lidar_south;
vl6180 lidar_east;

int pi_handle;
#include <iostream>
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

		set_mode(pi_handle, 6, PI_OUTPUT);
		set_mode(pi_handle, 13, PI_OUTPUT);
		set_mode(pi_handle, 19, PI_OUTPUT);
		set_mode(pi_handle, 26, PI_OUTPUT);

		gpio_write(pi_handle, 13, 1);
		//lidar_north = vl6180_initialise_address(1, LIDAR_NORTH);
		lidar_north = vl6180_initialise(1);
		vl6180_change_addr(lidar_north, LIDAR_NORTH);

		gpio_write(pi_handle, 6, 1);
//		lidar_west = vl6180_initialise_address(1, LIDAR_WEST);
		lidar_west = vl6180_initialise(1);
		vl6180_change_addr(lidar_west, LIDAR_WEST);

		gpio_write(pi_handle, 26, 1);
//		lidar_south = vl6180_initialise_address(1, LIDAR_SOUTH);
		lidar_south = vl6180_initialise(1);
		vl6180_change_addr(lidar_south, LIDAR_SOUTH);

		gpio_write(pi_handle, 19, 1);
//		lidar_east = vl6180_initialise_address(1, LIDAR_EAST);
		lidar_east = vl6180_initialise(1);
		vl6180_change_addr(lidar_east, LIDAR_EAST);
	}

	double servo_angle_ne() { return feedback_ne.angle_in_revolutions(); }
	double servo_angle_nw() { return feedback_nw.angle_in_revolutions(); }
	double servo_angle_sw() { return feedback_sw.angle_in_revolutions(); }
	double servo_angle_se() { return feedback_se.angle_in_revolutions(); }

	int lidar_distance_north() { return get_distance(lidar_north); }
	int lidar_distance_west() { return get_distance(lidar_west); }
	int lidar_distance_south() { return get_distance(lidar_south); }
	int lidar_distance_east() { return get_distance(lidar_east); }

	void terminate() { pigpio_stop(pi_handle); }
}

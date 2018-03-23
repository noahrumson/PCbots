#include <iostream>

#include "pigpio.h"

double cycle_min = 0.029;
double cycle_max = 0.971;
int feedback_period = 990;//1e6 / 910; 990 works better for some reason

int last_pulse_width;
void state_changed(int gpio, int level, uint32_t tick)
{
	static uint32_t lastTick = 0;
	if (lastTick == 0) {
		lastTick = tick;
	}
	if (level == 0) {
		last_pulse_width = tick - lastTick;
	}
	lastTick = tick;
}

double clamp(double x, double min, double max)
{
	if (x < min)
		return min;
	if (x > max)
		return max;
	return x;
}

/*
plug in wire with one end in port 27 and other in port 22
*/
void test_read_pi_pwm()
{
	double dcycle_out;
	std::cin >> dcycle_out;
	gpioInitialise();
	gpioSetMode(27, PI_OUTPUT);
	gpioSetMode(22, PI_INPUT);
	gpioSetAlertFunc(22, state_changed);
	gpioSetPWMfrequency(27, 910);
	gpioPWM(27, 255 * dcycle_out);
	gpioDelay(1e6);
	gpioPWM(27, 0);

	double duty_cycle = (double)last_pulse_width / feedback_period;
	std::cout << "duty cycle: " << duty_cycle << std::endl;
}

void test_read_servo_feedback()
{
	int pulse_width;
	do {
		std::cin >> pulse_width;
	} while (pulse_width > 1700 || pulse_width < 1500);

	gpioInitialise();
	gpioSetMode(4, PI_OUTPUT);
	gpioSetMode(27, PI_INPUT);
	gpioSetAlertFunc(27, state_changed);
	gpioSetPWMfrequency(4, 50);
	gpioServo(4, pulse_width);
	gpioDelay(1e6);
	gpioPWM(4, 0);

	double duty_cycle = (double)last_pulse_width / feedback_period;
	duty_cycle = clamp(duty_cycle, cycle_min, cycle_max);
	std::cout << "duty cycle: " << duty_cycle << std::endl;

	double angle = (duty_cycle - cycle_min) / (cycle_max - cycle_min);
	std::cout << "revolutions: " << angle << std::endl;
}

extern "C" double run()
{
	test_read_servo_feedback();
	return ((double)last_pulse_width / feedback_period - cycle_min) / (cycle_max - cycle_min);
}

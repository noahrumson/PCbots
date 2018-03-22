#include <iostream>

#include "pigpio.h"

double cycle_min = 2.9;
double cycle_max = 97.1;

int on_time = 0.0;
int total_time = 0.0;
void state_changed(int gpio, int level, uint32_t tick)
{
	static uint32_t lastTick = 0;
	if (lastTick == 0) {
		lastTick = tick;
	}
	uint32_t dt = tick - lastTick;
	if (level == 0) {
		on_time += dt;
	}
	total_time += dt;
	lastTick = tick;
}

double timer_dt = 1.0 / 100.0;
void timer_check()
{
	int state = gpioRead(27);
	if (state) {
		on_time += 1000 * timer_dt;
	}
	total_time += 1000 * timer_dt;
	if (total_time > 1000 && total_time < 1200) {
		double duty_cycle = 100.0 * (double)on_time / total_time;
		double revolutions = (duty_cycle - cycle_min) / (cycle_max - cycle_min + 1.0);
		std::cout << "duty cycle: " << duty_cycle << std::endl;
		std::cout << "angle: " << revolutions << std::endl;
		total_time = 0.0;
		on_time = 0.0;
	}
}

int main()
{
	double rpm1 = 6;
	//std::cin >> rpm1;
	gpioInitialise();
	gpioSetMode(4, PI_OUTPUT);
	gpioSetMode(27, PI_INPUT);
	gpioSetAlertFunc(27, state_changed);
//	gpioSetTimerFunc(0, 1000 * timer_dt, timer_check);
	gpioPWM(4, rpm1);
	gpioSetPWMfrequency(4, 50);
	gpioDelay(1e6);
	gpioPWM(4, 0);

	return 0;
}

//==================================================================================
// Name        : lab3_adc.cpp
// Author      : TAs of CS 684 Spring 2020
// Description : To get 8-bit ADC data of three white line sensors
//				 and 5th IR proximity sensor, print data of 5th IR proximity sensor
//==================================================================================


#include "eBot_Sandbox.h"


// Main Function
int main(int argc, char* argv[])
{
	int init_setup_success = 0;

	init_setup_success = init_setup();

	if (init_setup_success)
	{
		#ifdef NON_MATLAB_PARSING
			std::thread t_1(thread_calls);
			_delay_ms(5000);
		#endif

		send_sensor_data();

		#ifdef NON_MATLAB_PARSING
			clean_up();
			t_1.detach();
		#endif
	}
	else
	{
	}
}

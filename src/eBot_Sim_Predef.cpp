/*
 * eBot_Sim_Predef.cpp
 *
 *  Created on: 11-Jan-2021
 *      Author: TAs of CS 684 Spring 2020
 */


//---------------------------------- INCLUDES ----------------------------------

#include "eBot_Sim_Predef.h"
#include <string> // for string and to_string()
#include <iostream>
//------------------------------ GLOBAL VARIABLES -------------------------------

int number_of_robots = 2;
const float PI = 3.142857;
// store the client id for connection with CoppeliaSim
simxInt client_id = -1;

// store the ping time before stopping the connection with CoppeliaSim
simxInt *ping_time = new simxInt[1]();

// store the return code of various Remote API calls
simxInt return_code = -1;

// get the handle of robot with object named as 'eBot' in the CoppeliaSim scene
simxInt *eBot_handle[5] = {new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1]()};

// get the handle of robot's left motor with object named as 'LeftJoint' in the CoppeliaSim scene
simxInt *left_joint_handle[5] = {new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1]()};

// get the handle of robot's right motor with object named as 'RightJoint' in the CoppeliaSim scene
simxInt *right_joint_handle[5] = {new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1]()};

// flags for setting direction of robot's left and right motors
simxInt dir_left_motor[5] = {0, 0, 0, 0, 0};
simxInt dir_right_motor[5] = {0, 0, 0, 0, 0};

// to provide variable linear velocity to robot's left and right motors
simxFloat linear_velocity_left_motor[5] = {0, 0, 0, 0, 0};
simxFloat linear_velocity_right_motor[5] = {0, 0, 0, 0, 0};

// get the handle of robot's white line sensor with object named as 'LineSensor' in the CoppeliaSim scene
simxInt *white_line_sensor_handle = new simxInt[1]();
simxInt *left_sensor_handle[5] = {new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1]()};
simxInt *middle_sensor_handle[5] = {new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1]()};
simxInt *right_sensor_handle[5] = {new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1]()};
// store the white line sensor data of the object named as 'LineSensor' in the CoppeliaSim scene

// simxUChar *white_line_sensor_data = new simxUChar[7]();
//simxUChar *left_line_sensor_data = new simxUChar[1]();
//simxUChar *middle_line_sensor_data = new simxUChar[1]();
//simxUChar *right_line_sensor_data = new simxUChar[1]();

simxUChar left_line_sensor_data[5] = {0, 0, 0, 0, 0};
simxUChar middle_line_sensor_data[5] = {0, 0, 0, 0, 0};
simxUChar right_line_sensor_data[5] = {0, 0, 0, 0, 0};

// store the resolution of white line sensor data received
simxInt *resolution = new simxInt[2]();

// get the handle of robot's proximity sensor placed in front with object named as 'ProximitySensorFront' in the CoppeliaSim scene
simxInt *front_prox_sensor_handle[5] = {new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1]()};
simxInt *left_prox_sensor_handle[5] = {new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1]()};
simxInt *right_prox_sensor_handle[5] = {new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1]()};

// store the front proximity sensor detection state of the object named as 'ProximitySensorFront' in the CoppeliaSim scene
simxUChar front_prox_detection_state[5] = {0, 0, 0, 0, 0};
simxUChar left_prox_detection_state[5] = {0, 0, 0, 0, 0};
simxUChar right_prox_detection_state[5] = {0, 0, 0, 0, 0};

// store the front proximity sensor detected point of the object named as 'ProximitySensorFront' in the CoppeliaSim scene
simxFloat front_prox_detected_point[5][3];
simxFloat left_prox_detected_point[5][3];
simxFloat right_prox_detected_point[5][3];

// store the computed front proximity sensor distance w.r.t. the obstacle of the object named as 'ProximitySensorFront' in the CoppeliaSim scene
unsigned char front_prox_sensor_distance = 140;
unsigned char left_prox_sensor_distance = 140;
unsigned char right_prox_sensor_distance = 140;

//  My newly added variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5

float current_x_co = -6.7687;
float current_y_co = 0.31593;
float current_alpha_co = 0;
float current_beta_co = -1*PI/180;
float current_gama_co = 0;

simxInt *eBot_body_handle[5] = {new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1](),new simxInt[1]()};

simxFloat bot_position[3] = {-6.7687,0.31593,0};
simxFloat (*position_pointer)[3] = &bot_position;
//simxFloat bot_new_position[3] = {0,0,0};
//simxFloat(*New_position_pointer)[3] = &bot_new_position;

simxFloat bot_orientation[3] = {0,-1*PI/2,0};
simxFloat (*orientation_pointer)[3] = &bot_orientation;
//simxFloat bot_new_orientation[3] = {0,0,0};
//simxFloat(*New_orientation_pointer)[3] = &bot_new_orientation;

void set_bot_position(void);

//   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5

//---------------------------------- FUNCTIONS ----------------------------------

/**
 * @brief      Initializes the connection to CoppeliaSim Remote API Server
 */
void init_remote_api_server(void)
{
	int port_number = 19997;

	// just in case, close all opened connections
	simxFinish(-1);

	// connect to CoppeliaSim
	client_id = simxStart("127.0.0.1", port_number, true, true, 5000, 5);
}


//std::string get_const_char(std::string name, int robot_number)
//{
//	std::string s = name + std::to_string(robot_number);
//
//	const char* p = s.c_str();
//	printf("%s\n",p);
//	return s;
//}

/**
 * @brief      Collect the handles of all objects in the current CoppeliaSim scene
 */
void get_object_handles(void)
{
	return_code = -1;

	for (int i = 0; i < number_of_robots; i++)
	{
//		std::string robot_name = get_const_char("LineTracer#",i);
//		const char* p = robot_name.c_str();
		return_code = simxGetObjectHandle(client_id, ("LineTracer#" + std::to_string(i)).c_str(), eBot_handle[i], simx_opmode_blocking);
	}


	if (return_code == simx_return_ok)
	{
		for (int i = 0; i < number_of_robots; i++)
		{
			simxGetObjectHandle(client_id, ("DynamicLeftJoint#" + std::to_string(i)).c_str(), left_joint_handle[i], simx_opmode_blocking);
			simxGetObjectHandle(client_id, ("DynamicRightJoint#" + std::to_string(i)).c_str(), right_joint_handle[i], simx_opmode_blocking);
			simxGetObjectHandle(client_id, ("LeftSensor#" + std::to_string(i)).c_str(), left_sensor_handle[i], simx_opmode_blocking);
			simxGetObjectHandle(client_id, ("MiddleSensor#" + std::to_string(i)).c_str(), middle_sensor_handle[i], simx_opmode_blocking);
			simxGetObjectHandle(client_id, ("RightSensor#" + std::to_string(i)).c_str(), right_sensor_handle[i], simx_opmode_blocking);
			simxGetObjectHandle(client_id, ("ProximitySensorMiddle#" + std::to_string(i)).c_str(), front_prox_sensor_handle[i], simx_opmode_blocking);
			simxGetObjectHandle(client_id, ("ProximitySensorLeft#" + std::to_string(i)).c_str(), left_prox_sensor_handle[i], simx_opmode_blocking);
			simxGetObjectHandle(client_id, ("ProximitySensorRight#" + std::to_string(i)).c_str(), right_prox_sensor_handle[i], simx_opmode_blocking);
			// my own code is below %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			simxGetObjectHandle(client_id, ("DynamicBody#" + std::to_string(i)).c_str(), eBot_body_handle[i], simx_opmode_blocking);
			// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		}
//		simxGetObjectHandle(client_id, "LineSensor", white_line_sensor_handle, simx_opmode_blocking);
//		simxGetObjectHandle(client_id, "ProximitySensorFront", front_prox_sensor_handle, simx_opmode_blocking);
	}
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  my additional funtions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void set_bot_position(void)
{
	//simxGetObjectPosition(client_id , *eBot_body_handle[0],-1,*position_pointer,simx_opmode_oneshot_wait);

	//bot_new_position[0] = bot_position[0];
	//bot_new_position[1] = bot_position[1];
	//bot_new_position[2] = bot_position[2];

		bot_position[0] = current_x_co;
		bot_position[1] = current_y_co;
		bot_position[2] = 0.055265;

		bot_orientation[0] = current_alpha_co;
		bot_orientation[1] = current_beta_co;
		bot_orientation[2] = current_gama_co;

	simxSetObjectOrientation(client_id, *eBot_body_handle[0], -1, *orientation_pointer, simx_opmode_oneshot_wait);

	printf("\t\nX alpha: %03f \t Y beta: %03f \t Z gama: %03f", bot_orientation[0], bot_orientation[1], bot_orientation[2]);

	simxSetObjectPosition(client_id, *eBot_body_handle[0], -1, *position_pointer, simx_opmode_oneshot_wait);

	printf("\t\nX position: %03f \t Y position: %03f \t Z position: %03f", bot_position[0], bot_position[1], bot_position[2]);

	//simxGetObjectOrientation(client_id, *eBot_body_handle[0], -1, *orientation_pointer, simx_opmode_oneshot_wait);

	//bot_new_orientation[0] = bot_position[0];
	//bot_new_orientation[1] = bot_position[1];
	//bot_new_orientation[2] = bot_position[2];


	//printf("\t\nX orientation: %03f \t Y orientation: %03f \t Z orientation: %03f", bot_orientation[0], bot_orientation[1], bot_orientation[2]);

}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

/**
 * @brief      Starts the simulation in CoppeliaSim
 */
void start_simulation(void)
{
	return_code = -1;

	printf("\n\tEnter 'Y' or 'y' to start the simulation: ");

	char input_cmd = getchar();

	if (input_cmd == 'Y' || input_cmd == 'y')
	{
		do
		{
			return_code = simxStartSimulation(client_id, simx_opmode_oneshot_wait);
		} while (return_code != simx_return_ok);
	}
}


void get_white_line_sensor_data(void)
{
	return_code = -1;
	for (int i = 0; i < number_of_robots; i++)
	{
		return_code = simxReadVisionSensor(client_id, *left_sensor_handle[i], &left_line_sensor_data[i], NULL, NULL, simx_opmode_buffer);
		return_code = simxReadVisionSensor(client_id, *middle_sensor_handle[i], &middle_line_sensor_data[i], NULL, NULL, simx_opmode_buffer);
		return_code = simxReadVisionSensor(client_id, *right_sensor_handle[i], &right_line_sensor_data[i], NULL, NULL, simx_opmode_buffer);
	}
}



void init_vision_sensors(void)
{
	return_code = -1;
	for (int i = 0; i < number_of_robots; i++)
	{
		do
		{
			return_code = simxReadVisionSensor(client_id, *left_sensor_handle[i], &left_line_sensor_data[i], NULL, NULL, simx_opmode_streaming);
		} while (return_code != simx_return_ok || return_code == simx_return_novalue_flag);

		do
		{
			return_code = simxReadVisionSensor(client_id, *middle_sensor_handle[i], &middle_line_sensor_data[i], NULL, NULL, simx_opmode_streaming);
		} while (return_code != simx_return_ok || return_code == simx_return_novalue_flag);

		do
		{
			return_code = simxReadVisionSensor(client_id, *right_sensor_handle[i], &right_line_sensor_data[i], NULL, NULL, simx_opmode_streaming);
		} while (return_code != simx_return_ok || return_code == simx_return_novalue_flag);
	}
	for (int i = 0; i < VIS_SEN_INITIAL_VAL; i++)
	{
		get_white_line_sensor_data();
	}
}


void get_prox_sensor_data(void)
{
	return_code = -1;
	for (int i = 0; i < number_of_robots; i++)
	{
		return_code = simxReadProximitySensor(client_id, *front_prox_sensor_handle[i], &front_prox_detection_state[i],
															front_prox_detected_point[i], NULL, NULL, simx_opmode_blocking);
		return_code = simxReadProximitySensor(client_id, *left_prox_sensor_handle[i], &left_prox_detection_state[i],
																	left_prox_detected_point[i], NULL, NULL, simx_opmode_blocking);
		return_code = simxReadProximitySensor(client_id, *right_prox_sensor_handle[i], &right_prox_detection_state[i],
																			right_prox_detected_point[i], NULL, NULL, simx_opmode_blocking);

	}
//	if (prox_sensor_num == FRONT_IR_PROX_NUMBER)
//	{
//		return_code = simxReadProximitySensor(client_id, *front_prox_sensor_handle, &front_prox_detection_state,
//													front_prox_detected_point, NULL, NULL, simx_opmode_blocking);
//	}
//
//	if (front_prox_detection_state != 0)
//	{
//		front_prox_detection_state = 0;
//		front_prox_sensor_distance = (unsigned char)(front_prox_detected_point[2] * 1000);
//	}
//	else
//	{
//		front_prox_sensor_distance = 200;
//	}
//
//	return front_prox_sensor_distance;
}


void init_prox_sensors(void)
{
	return_code = -1;
	do
	{
		get_prox_sensor_data();
	} while (return_code != simx_return_ok); // || return_code == simx_return_novalue_flag);

}


/**
 * @brief      Initialize all sensor objects in CoppeliaSim
 */
void init_sensors(void)
{
	init_vision_sensors();
	init_prox_sensors();
}


/**
 * @brief      Initializes the setup by connecting to CoppeliaSim and starting the simulation
 */
int init_setup(void)
{
	// Initialize the connection to CoppeliaSim Remote API Server
	printf("\n\tCoppeliaSim Remote API Server initiated.");
	printf("\n\tTrying to connect to CoppeliaSim Remote API Server ...");

	init_remote_api_server();

	if (client_id == -1)
	{
		printf("\n\tFailed connecting to remote API server!");
		printf("\n\t[WARNING] Make sure the CoppeliaSim software is running and");
		printf("\n\t[WARNING] Make sure the Port number for Remote API Server is set to 19997.");

		return 0;
	}
	else
	{
		printf("\n\tConnected successfully to Remote API Server in CoppeliaSim!");

		// Get the handles of all objects in the scene if connection to CoppeliaSim was successful
		get_object_handles();

		if (return_code == simx_return_ok)
		{
			printf("\n\tCollected handles of all objects in the current CoppeliaSim scene.");

			// Start the simulation
			start_simulation();

			if (return_code == simx_return_ok)
			{
				printf("\n\tSimulation started correctly.");

				// Initialize the sensors in the scene
				init_sensors();

				if (return_code == simx_return_ok)
				{
					printf("\n\tInitialized all sensors in the current CoppeliaSim scene.");

					return 1;
				}
			}
		}
	}

	return 0;
}


unsigned char convert_analog_channel_data(int robot_number, unsigned char sensor_channel_num)
{
	// Left White Line Sensor
	if (sensor_channel_num == 1)
		return left_line_sensor_data[robot_number];
	// Middle White Line Sensor
	else if (sensor_channel_num == 2)
		return middle_line_sensor_data[robot_number];
	// Right White Line Sensor
	else if (sensor_channel_num == 3)
		return right_line_sensor_data[robot_number];
	// Front Proximity Sensor
	else if (sensor_channel_num == 5)
	{
		if (front_prox_detection_state[robot_number] != 0)
		{
			front_prox_detection_state[robot_number] = 0;
			front_prox_sensor_distance = (unsigned char)(front_prox_detected_point[robot_number][2] * 1000);
			if (front_prox_sensor_distance > 200)
				front_prox_sensor_distance = 200;
		}
		else
		{
			front_prox_sensor_distance = 200;
		}
		return front_prox_sensor_distance;
	}
	// Left Proximity Sensor
	else if (sensor_channel_num == 4)
	{
		if (left_prox_detection_state[robot_number] != 0)
		{
			left_prox_detection_state[robot_number] = 0;
			left_prox_sensor_distance = (unsigned char)(left_prox_detected_point[robot_number][2] * 1000);
			if (left_prox_sensor_distance > 200)
				left_prox_sensor_distance = 200;
		}
		else
		{
			left_prox_sensor_distance = 200;
		}
		return left_prox_sensor_distance;
	}
	// Right Proximity Sensor
	else if (sensor_channel_num == 6)
	{
		if (right_prox_detection_state[robot_number] != 0)
		{
			right_prox_detection_state[robot_number] = 0;
			right_prox_sensor_distance = (unsigned char)(right_prox_detected_point[robot_number][2] * 1000);
			if (right_prox_sensor_distance > 200)
				right_prox_sensor_distance = 200;
		}
		else
		{
			right_prox_sensor_distance = 200;
		}
		return right_prox_sensor_distance;
	}
	return 0;
}


int print_ir_prox_5_data(unsigned char left_wl_sensor_data,unsigned char center_wl_sensor_data,unsigned char right_wl_sensor_data)
{
	printf("\n\tEnter 'Y' / 'y' to take the next sensor reading or 'Q' / 'q' to quit: ");

	char input_cmd = getchar();
	input_cmd = getchar();

	if (input_cmd == 'Y' || input_cmd == 'y')
	{
		printf("\t\nLeft Sensor data: %03d \t Middle Sensor data: %03d \t Right Sensor data: %03d", left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data);
		return 1;
	}
	else if (input_cmd == 'Q' || input_cmd == 'q')
	{
		return 0;
	}
	return 0;

}


/**
 * @brief      Updates the speed of robot's left and right motors
 */
void set_motor_velocities(void)
{
	for (int i = 0; i < number_of_robots; i++)
	{
		simxSetJointTargetVelocity(client_id, *left_joint_handle[i], linear_velocity_left_motor[i], simx_opmode_oneshot);
		simxSetJointTargetVelocity(client_id, *right_joint_handle[i], linear_velocity_right_motor[i], simx_opmode_oneshot);
	}
}



/**
 * @brief      Makes the robot move forward
 */
void forward(int robot_number)
{
	dir_left_motor[robot_number] = dir_right_motor[robot_number] = 1;
	linear_velocity_left_motor[robot_number] = linear_velocity_right_motor[robot_number] = LINEAR_VELOCITY_MAX;
}


/**
 * @brief      Makes the robot move backward
 */
void back(int robot_number)
{
	dir_left_motor[robot_number] = dir_right_motor[robot_number] = -1;
	linear_velocity_left_motor[robot_number] = linear_velocity_right_motor[robot_number] = -LINEAR_VELOCITY_MAX;
}


/**
 * @brief      Makes the robot take hard left turn
 */
void left(int robot_number)
{
	dir_left_motor[robot_number] = -1;
	dir_right_motor[robot_number] = 1;
	linear_velocity_left_motor[robot_number] = -LINEAR_VELOCITY_MAX;
	linear_velocity_right_motor[robot_number] = LINEAR_VELOCITY_MAX;
}


/**
 * @brief      Makes the robot take hard right turn
 */
void right(int robot_number)
{
	dir_left_motor[robot_number] = 1;
	dir_right_motor[robot_number] = -1;
	linear_velocity_left_motor[robot_number] = LINEAR_VELOCITY_MAX;
	linear_velocity_right_motor[robot_number] = -LINEAR_VELOCITY_MAX;
}


/**
 * @brief      Makes the robot take soft left turn
 */
void soft_left(int robot_number)
{
	dir_left_motor[robot_number] = 0;
	dir_right_motor[robot_number] = 1;
	linear_velocity_left_motor[robot_number] = 0;
	linear_velocity_right_motor[robot_number] = LINEAR_VELOCITY_MAX;
}


/**
 * @brief      Makes the robot take soft right turn
 */
void soft_right(int robot_number)
{
	dir_left_motor[robot_number] = 1;
	dir_right_motor[robot_number] = 0;
	linear_velocity_left_motor[robot_number] = LINEAR_VELOCITY_MAX;
	linear_velocity_right_motor[robot_number] = 0;
}


/**
 * @brief      Stops the robot's left and right motors
 */
void stop(int robot_number)
{
	dir_left_motor[robot_number] = 0;
	dir_right_motor[robot_number] = 0;
	linear_velocity_left_motor[robot_number] = 0;
	linear_velocity_right_motor[robot_number] = 0;
}


/**
 * @brief      Configures the speed of robot's left and right motors
 */
void velocity(int robot_number,int left_motor_speed, int right_motor_speed)
{
	if (left_motor_speed > 255)
		left_motor_speed = 255;

	else if (left_motor_speed < 0)
		left_motor_speed = 0;

	if (right_motor_speed > 255)
		right_motor_speed = 255;

	else if (right_motor_speed < 0)
		right_motor_speed = 0;

	linear_velocity_left_motor[robot_number] = ( dir_left_motor[robot_number] * LINEAR_VELOCITY_MAX * left_motor_speed ) / float(255.0);
	linear_velocity_right_motor[robot_number] = ( dir_right_motor[robot_number] * LINEAR_VELOCITY_MAX * right_motor_speed ) / float(255.0);
}


/**
 * @brief      Starts the thread to set the motor velocities, get white line sensor and front proximity sensor data
 */
void thread_calls(void)
{
	while (client_id != -1)
	{
		set_motor_velocities();
		get_white_line_sensor_data();
//		front_prox_sensor_distance = get_front_prox_sensor_distance(FRONT_IR_PROX_NUMBER);
		get_prox_sensor_data();

		// my own added %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		set_bot_position();
		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
	}
}


/**
 * @brief      Generates the delay for the provided number of milliseconds
 */
void _delay_ms(unsigned int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}


/**
 * @brief      Stops the simulation in CoppeliaSim
 */
void stop_simulation(void)
{
	return_code = -1;

	do
	{
		return_code = simxStopSimulation(client_id, simx_opmode_oneshot);
	} while (return_code != simx_return_novalue_flag);
}


/**
 * @brief      Stops the connection to CoppeliaSim Remote API Server
 */
void exit_remote_api_server(void)
{
	// before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive
	simxGetPingTime(client_id, ping_time);

	// close the connection to CoppeliaSim
	simxFinish(-1);
}


/**
 * @brief      Stops the robot's left and right motor, stops the simulation and
 * 			   closes the connection with CoppeliaSim Remote API Server
 */
void clean_up(void)
{
//	velocity(0, 0);
	set_motor_velocities();

	// Stopping the simulation
	stop_simulation();

	if (return_code == simx_return_novalue_flag)
	{
		printf("\n\tSimulation stopped correctly.");

		// Stop the connection with CoppeliaSim Remote API server
		exit_remote_api_server();

		printf("\n\tDisconnected successfully to Remote API Server in CoppeliaSim!");

		// Reset the client_id
		client_id = -1;
	}
}

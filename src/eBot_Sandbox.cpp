/*
 * eBot_Sandbox.cpp
 *
 *  Created on: 11-Jan-2021
 *      Author: TAs of CS 684 Spring 2020
 */


//---------------------------------- INCLUDES -----------------------------------

#include "eBot_Sandbox.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
using namespace std;

const float PI = 3.142857;
const float tol = 0.1; // degree tolrance

// ----------------------------------extern Variable -----------------------------------
extern float current_x_co;
extern float current_y_co;
extern float current_alpha_co;
extern float current_beta_co;
extern float current_gama_co;
//------------------------------ GLOBAL VARIABLES -------------------------------

// To store 8-bit data of left, center and right white line sensors
unsigned int left_wl_sensor_data, center_wl_sensor_data, right_wl_sensor_data;

// To store 8-bit data of 5th IR proximity sensors
unsigned char left_ir_sensor_data, center_ir_sensor_data, right_ir_sensor_data;
const int total_robot = 1;

void actuation_bot0(int);
int decision_maker(int,int,int,int,int,int);
bool is_visited_before_at(int,int,int);
bool is_get_parking_at(int,int);
int Next_X_co_founder(int,int,int,int,int);
int Next_Y_co_founder(int,int,int,int,int);
int Nxs_finder(int,int,int,int);
int Nxr_finder(int,int,int,int);
int Nxl_finder(int,int,int,int);
int Nxb_finder(int,int,int,int);
int Nys_finder(int,int,int,int);
int Nyr_finder(int,int,int,int);
int Nyl_finder(int,int,int,int);
int Nyb_finder(int,int,int,int);
bool is_parked(int,int,int);

void update_proximity(void);

float Next_gama_co_founder(int, float, float, float);
float Next_beta_co_founder(int, float, float, float);
float Next_alpha_co_founder(int, float, float, float);
float N_alpha_left_finder(float,float);
float N_alpha_right_finder(float,float);
float N_alpha_back_finder(float,float);
float N_alpha_stright_finder(float,float);
float N_beta_left_finder(float,float);
float N_beta_right_finder(float,float);
float N_beta_back_finder(float,float);
float N_beta_stright_finder(float,float);
float N_gama_left_finder(float); 
float N_gama_right_finder(float);
float N_gama_back_finder(float);
float N_gama_stright_finder(float);
float convert_to_actual_co_x(int,int);
float convert_to_actual_co_y(int,int);
void actuate(float,float,float,float,float);

const bool is_parking_node[9][18] = {{false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false},{false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false},{false,true,false,false,true,true,false,false,true,true,false,false,true,true,false,false,true,false},{false,true,false,false,true,true,false,false,true,true,false,false,true,true,false,false,true,false},{false,true,false,false,true,true,false,false,true,true,false,false,true,true,false,false,true,false},{false,true,false,false,true,true,false,false,true,true,false,false,true,true,false,false,true,false},{false,true,false,false,true,true,false,false,true,true,false,false,true,true,false,false,true,false},{false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false},{false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false}};

const float location_x[7][16] = {{-6.7687,-4.9816,-4.0316,-3.2316,-2.5316,-1.9916,-1.0116,-0.23159,0.36841,1.0084,1.9884,2.7584,3.3584,3.9684,4.9484,6.5484},{-5.4674,-4.9816,-4.0316,-3.2316,-2.5316-1.9916,-1.0116,-0.23159,0.36841,1.0084,1.9884,2.7584,3.3584,3.9684,4.9484,5.5094},{-5.4674,-4.9816,-4.0316,-3.2316,-2.5316-1.9916,-1.0116,-0.23159,0.36841,1.0084,1.9884,2.7584,3.3584,3.9684,4.9484,5.5094},{-5.4674,-4.9816,-4.0316,-3.2316,-2.5316-1.9916,-1.0116,-0.23159,0.36841,1.0084,1.9884,2.7584,3.3584,3.9684,4.9484,5.5094},{-5.4674,-4.9816,-4.0316,-3.2316,-2.5316-1.9916,-1.0116,-0.23159,0.36841,1.0084,1.9884,2.7584,3.3584,3.9684,4.9484,5.5094},{-5.4674,-4.9816,-4.0316,-3.2316,-2.5316-1.9916,-1.0116,-0.23159,0.36841,1.0084,1.9884,2.7584,3.3584,3.9684,4.9484,5.5094},{-6.7687,-4.9816,-4.0316,-3.2316,-2.5316-1.9916,-1.0116,-0.23159,0.36841,1.0084,1.9884,2.7584,3.3584,3.9684,4.9484,6.5484}};

const float location_y[7][16] = {{0.31593,0.31593,0.31593,0.31593,0.31593,0.31593,0.31593,0.31593,0.31593,0.31593,0.31593,0.31593,0.31593,0.31593,0.31593,0.31593},{1.4909,1.4909,1.4909,1.4909,1.4909,1.4909,1.4909,1.4909,1.4909,1.4909,1.4909,1.4909,1.4909,1.4909,1.4909,1.4909},{2.2289,2.2289,2.2289,2.2289,2.2289,2.2289,2.2289,2.2289,2.2289,2.2289,2.2289,2.2289,2.2289,2.2289,2.2289,2.2289},{3.0069,3.0069,3.0069,3.0069,3.0069,3.0069,3.0069,3.0069,3.0069,3.0069,3.0069,3.0069,3.0069,3.0069,3.0069,3.0069},{3.7269,3.7269,3.7269,3.7269,3.7269,3.7269,3.7269,3.7269,3.7269,3.7269,3.7269,3.7269,3.7269,3.7269,3.7269,3.7269},{4.4868,4.4868,4.4868,4.4868,4.4868,4.4868,4.4868,4.4868,4.4868,4.4868,4.4868,4.4868,4.4868,4.4868,4.4868,4.4868},{5.6319,5.6319,5.6319,5.6319,5.6319,5.6319,5.6319,5.6319,5.6319,5.6319,5.6319,5.6319,5.6319,5.6319,5.6319,5.6319}};

// currently it kept constant assuming that the sence is constant but we will chang letter
bool is_parking_ocupied[7][16] = {{false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false},{true,false,false,false,true,false,false,false,true,false,false,true,false,false,false,true},{false,false,false,true,false,false,false,false,false,false,false,false,true,false,false,true},{false,false,false,false,true,false,false,true,false,false,false,true,false,false,false,false},{true,false,false,true,true,false,false,false,false,false,false,false,true,false,false,false},{true,false,false,true,false,false,false,false,true,false,false,true,false,false,false,true},{false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false}};


bool is_node_visited_parking[total_robot][9][18] = {false};
bool is_node_visited_leaving[total_robot][9][18] = {false};


//---------------------------------- FUNCTIONS ----------------------------------


/**
 * @brief      Executes the logic to achieve the aim of project1
 */
void send_sensor_data(void)
{
	int entry_gate = 1;
	// int return_code;

	for (int i = 0; i < total_robot; i++)
		{
			for (int j=0; j<9; j++)
			{
				for (int k=0; k<18; k++)
				{
					if(j==0 or k==0 or j==8 or k==17)
					{
						is_node_visited_parking[i][j][k] = true;
						is_node_visited_leaving[i][j][k] = true;
					}
				}
			}

		}

	for (int i = 0; i < total_robot; i++)
			{
				for (int j=0; j<7; j++)
				{
					for (int k=0; k<16; k++)
					{
						if(is_parking_ocupied[j][k] == true)
						{
							is_node_visited_parking[i][j+1][k+1] = true;
							is_node_visited_leaving[i][j+1][k+1] = true;
						}
					}
				}

			}
/*

	for (int i = 0; i < total_robot; i++)
			{
				for (int j=0; j<9; j++)
				{
					for (int k=0; k<18; k++)
					{
						printf("\t%d",is_node_visited_parking[i][j][k]);
					}
					printf("\n");
				}
				printf("\n");
			}
*/

	while (1)
	{
		//left_wl_sensor_data = convert_analog_channel_data(0, left_wl_sensor_channel);
		//center_wl_sensor_data = convert_analog_channel_data(0, center_wl_sensor_channel);
		//right_wl_sensor_data = convert_analog_channel_data(0, right_wl_sensor_channel);

		update_proximity();
		
		//printf("\n\t left: %d \t mid: %d \t right: %d",left_ir_sensor_data,center_ir_sensor_data,right_ir_sensor_data);
		actuation_bot0(entry_gate);

		//return_code = print_ir_prox_5_data(left_ir_sensor_data, center_ir_sensor_data, right_ir_sensor_data);


		//left_wl_sensor_data = convert_analog_channel_data(1, left_wl_sensor_channel);
		//center_wl_sensor_data = convert_analog_channel_data(1, center_wl_sensor_channel);
		//right_wl_sensor_data = convert_analog_channel_data(1, right_wl_sensor_channel);
		//left_ir_sensor_data	= convert_analog_channel_data(1, left_ir_sensor_channel);
		//center_ir_sensor_data = convert_analog_channel_data(1, center_ir_sensor_channel);
		//right_ir_sensor_data = convert_analog_channel_data(1, right_ir_sensor_channel);

		//return_code = print_ir_prox_5_data(left_ir_sensor_data, center_ir_sensor_data, right_ir_sensor_data);

		_delay_ms(500000);
	}
}

void update_proximity(void)
{
	left_ir_sensor_data	= convert_analog_channel_data(0, right_ir_sensor_channel); // left & right are swaped
	center_ir_sensor_data = convert_analog_channel_data(0, center_ir_sensor_channel);
	right_ir_sensor_data = convert_analog_channel_data(0, left_ir_sensor_channel);
}

void actuation_bot0(int entry_gate)
{
int cx=0, cy=0, px=0, py=0,nx=0, ny=0, where_to_go, bot_number = 0;
float c_alpha =0, c_beta = 0, c_gama = 0, n_alpha, n_beta, n_gama, co_x, co_y;


// %% intial state
	update_proximity();
	if(entry_gate == 1 or entry_gate == 3)
	{
	cx = 0;

	c_alpha = 0;
	c_beta = -1*PI/2;
	c_gama = 0;
	} else if(entry_gate == 2 or entry_gate == 4)
	{
	cx = 15;

	c_alpha = 0;
	c_beta = PI/2;
	c_gama = PI;
	} 
	
	if(entry_gate == 1 or entry_gate == 2)
	{
	cy = 6;
	} else if(entry_gate == 3 or entry_gate == 4)
	{
	cy = 0;
	} 

is_node_visited_parking[0][cy+1][cx+1] = true;
printf("\n\t left: %d \t mid: %d \t right: %d",left_ir_sensor_data,center_ir_sensor_data,right_ir_sensor_data);
where_to_go = 1; // means go stright

co_x = convert_to_actual_co_x(cx, cy);
co_y = convert_to_actual_co_y(cx, cy);

printf("\n\t cx: %d, \t cy: %d",cx,cy);
printf("\n\t c_alpha: %f, \t c_beta: %f \t c_gama: %f",c_alpha,c_beta,c_gama);

actuate(co_x,co_y,c_alpha,c_beta, c_gama);
_delay_ms(3000);


// second stage after going stright
update_proximity();
if(entry_gate == 1 or entry_gate == 3)
	{
	px = 0;
	cx = 1; 

	c_alpha = 0;
	c_beta = -1*PI/2;
	c_gama = 0;

	} else if(entry_gate == 2 or entry_gate == 4)
	{
	px = 15;
	cx = 14;

	c_alpha = 0;
	c_beta = PI/2;
	c_gama = PI;

	} 
	
	if(entry_gate == 1 or entry_gate == 2)
	{
	py = 6;
	cy = 6;
	} else if(entry_gate == 3 or entry_gate == 4)
	{
	py = 0;
	cy = 0;
	} 
is_node_visited_parking[0][cy+1][cx+1] = true;

co_x = convert_to_actual_co_x(cx, cy);
co_y = convert_to_actual_co_y(cx, cy);

printf("\n\t cx: %d, \t cy: %d",cx,cy);
printf("\n\t c_alpha: %f, \t c_beta: %f \t c_gama: %f",c_alpha,c_beta,c_gama);


actuate(co_x,co_y,c_alpha,c_beta, c_gama);
_delay_ms(3000);
// make decision for next step 

do{
update_proximity();
printf("\n\t left: %d \t mid: %d \t right: %d",left_ir_sensor_data,center_ir_sensor_data,right_ir_sensor_data);
where_to_go = decision_maker(cx,cy,px,py,bot_number,entry_gate);

nx = Next_X_co_founder(where_to_go,cx,px,cy,py);
ny = Next_Y_co_founder(where_to_go,cx,px,cy,py);
n_alpha = Next_alpha_co_founder(where_to_go,c_alpha,c_beta,c_gama);
n_beta = Next_beta_co_founder(where_to_go,c_alpha,c_beta,c_gama);
n_gama = Next_gama_co_founder(where_to_go,c_alpha,c_beta,c_gama);

px = cx;
py = cy;

cx = nx;
cy = ny;

c_alpha = n_alpha;
c_beta = n_beta;
c_gama = n_gama;

co_x = convert_to_actual_co_x(cx, cy);
co_y = convert_to_actual_co_y(cx, cy);

printf("\n\t cx: %d, \t cy: %d",cx,cy);

actuate(co_x,co_y,c_alpha,c_beta, c_gama);
_delay_ms(3000);


is_node_visited_parking[bot_number][cy+1][cx+1] = true;
} while (!is_parked(bot_number, cx, cy));

}


void actuate(float co_x,float co_y,float c_alpha,float c_beta, float c_gama)
{
	current_x_co = co_x;
	current_y_co = co_y;
	current_alpha_co = c_alpha;
	current_beta_co = c_beta;
	current_gama_co = c_gama;

}


float convert_to_actual_co_x(int cx, int cy)
{
	return location_x[cy][cx];
}

float convert_to_actual_co_y(int cx, int cy)
{
	return location_y[cy][cx];
}

bool is_parked(int bot_number, int cx, int cy)
{
	if(is_parking_node[cy+1][cx+1] == true)
		{
		return true;
		}
	else
		{
		return false;
		}
}

int Next_X_co_founder(int where_to_go,int cx,int px,int cy,int py)
{
int nx;
switch(where_to_go)
{
case 1: // stright
nx = Nxs_finder(cx,cy,px,py);
break;

case 10: // right
nx = Nxr_finder(cx,cy,px,py);
break;

case -10: // left
nx = Nxl_finder(cx,cy,px,py);
break;

case -1: // back
nx = Nxb_finder(cx,cy,px,py);
break;

default:
printf("\nProblem: next X can't find");
}
return nx;
}

int Nxs_finder(int cx,int cy,int px,int py)
{
int nxs;
nxs = (2*cx)-px;
return nxs;
}

int Nxr_finder(int cx,int cy,int px,int py)
{
int nxr;
nxr = cx+cy-py;
return nxr;
}

int Nxl_finder(int cx,int cy,int px,int py)
{
int nxl;
nxl = cx+py-cy;
return nxl;
}

int Nxb_finder(int cx,int cy,int px,int py)
{
int nxb;
nxb = px;
return nxb;
}


int Next_Y_co_founder(int where_to_go,int cx,int px,int cy,int py)
{
int ny;
switch(where_to_go)
{
case 1: // stright
ny = Nys_finder(cx,cy,px,py);
break;

case 10: // right
ny = Nyr_finder(cx,cy,px,py);
break;

case -10: // left
ny = Nyl_finder(cx,cy,px,py);
break;

case -1: // back
ny = Nyb_finder(cx,cy,px,py);
break;

default:
printf("\nProblem: next Y can't find");
}
return ny;
}

int Nys_finder(int cx,int cy,int px,int py)
{
int nys;
nys = (2*cy)-py;
return nys;
}

int Nyr_finder(int cx,int cy,int px,int py)
{
int nyr;
nyr = cy+px-cx;
return nyr;
}

int Nyl_finder(int cx,int cy,int px,int py)
{
int nyl;
nyl = cy+cx-px;
return nyl;
}

int Nyb_finder(int cx,int cy,int px,int py)
{
int nyb;
nyb = py;
return nyb;
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5


float Next_alpha_co_founder(int where_to_go,float c_alpha, float c_beta, float c_gama)
{
float n_alpha;
switch(where_to_go)
{
case 1: // stright 
n_alpha = N_alpha_stright_finder(c_alpha, c_beta);
break;

case 10: // right
n_alpha = N_alpha_right_finder(c_alpha, c_beta);
break;

case -10: // left
n_alpha = N_alpha_left_finder(c_alpha, c_beta);
break;

case -1: // back
n_alpha = N_alpha_back_finder(c_alpha, c_beta);
break;

default:
printf("\nProblem: next alpha can't find");
}
return n_alpha;
}


float N_alpha_left_finder(float c_alpha, float c_beta) // return in radiunse
{
float c_alpha_deg, c_beta_deg, n_alpha_deg;
 
c_alpha_deg = c_alpha*180/PI;
c_beta_deg = c_beta*180/PI;

if ((90-tol) < abs(c_alpha_deg) && abs(c_alpha_deg) < (90+tol))
{
n_alpha_deg = 0;
}
else if (abs(c_alpha_deg) < tol)
{
n_alpha_deg = -1*c_beta_deg;
}

return PI*n_alpha_deg/180;
}



float N_alpha_right_finder(float c_alpha, float c_beta) // return in radiunse
{
float c_alpha_deg, c_beta_deg, n_alpha_deg;
 
c_alpha_deg = c_alpha*180/PI;
c_beta_deg = c_beta*180/PI;

if ((90-tol) < abs(c_alpha_deg) && abs(c_alpha_deg) < (90+tol))
{
n_alpha_deg = 0;
}
else if (abs(c_alpha_deg) < tol)
{
n_alpha_deg = c_beta_deg;
}

return PI*n_alpha_deg/180;
}


float N_alpha_back_finder(float c_alpha, float c_beta) // return in radiunse
{
return -1*c_alpha;
}

float N_alpha_stright_finder(float c_alpha, float c_beta) // return in radiunse
{
return c_alpha;
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


float Next_beta_co_founder(int where_to_go,float c_alpha, float c_beta, float c_gama)
{
float n_beta;
switch(where_to_go)
{
case 1: // stright 
n_beta = N_beta_stright_finder(c_alpha, c_beta);
break;

case 10: // right
n_beta = N_beta_right_finder(c_alpha, c_beta);
break;

case -10: // left
n_beta = N_beta_left_finder(c_alpha, c_beta);
break;

case -1: // back
n_beta = N_beta_back_finder(c_alpha, c_beta);
break;

default:
printf("\nProblem: next beta can't find");
}
return n_beta;
}

float N_beta_left_finder(float c_alpha, float c_beta) // return in radiunse
{
float c_alpha_deg, c_beta_deg, n_beta_deg;
 
c_alpha_deg = c_alpha*180/PI;
c_beta_deg = c_beta*180/PI;

if ((90-tol) < abs(c_beta_deg) && abs(c_beta_deg) < (90+tol))
{
n_beta_deg = 0;
}
else if (abs(c_beta_deg) < tol)
{
n_beta_deg = c_alpha_deg;
}

return PI*n_beta_deg/180;
}


float N_beta_right_finder(float c_alpha, float c_beta) // return in radiunse
{
float c_alpha_deg, c_beta_deg, n_beta_deg;
 
c_alpha_deg = c_alpha*180/PI;
c_beta_deg = c_beta*180/PI;

if ((90-tol) < abs(c_beta_deg) && abs(c_beta_deg) < (90+tol))
{
n_beta_deg = 0;
}
else if (abs(c_alpha_deg) < tol)
{
n_beta_deg = -1*c_alpha_deg;
}

return PI*n_beta_deg/180;
}


float N_beta_back_finder(float c_alpha, float c_beta) // return in radiunse
{
return -1*c_beta;
}

float N_beta_stright_finder(float c_alpha, float c_beta) // return in radiunse
{
return c_beta;
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5

float Next_gama_co_founder(int where_to_go,float c_alpha, float c_beta, float c_gama)
{
float n_gama;
switch(where_to_go)
{
case 1: // stright 
n_gama = N_gama_stright_finder(c_gama);
break;

case 10: // right
n_gama = N_gama_right_finder(c_gama);
break;

case -10: // left
n_gama = N_gama_left_finder(c_gama);
break;

case -1: // back
n_gama = N_gama_back_finder(c_gama);
break;

default:
printf("\nProblem: next gama can't find");
}
return n_gama;
}


float N_gama_left_finder(float c_gama) // return in radiunse
{
float c_gama_deg, n_gama_deg;
 
c_gama_deg = c_gama*180/PI;

if (abs(c_gama_deg + 90) < 180)
{
n_gama_deg = c_gama_deg + 90;
}
else if (abs(c_gama_deg + 90) >= 180)
{
n_gama_deg = c_gama_deg -270;
}

return PI*n_gama_deg/180;
}

float N_gama_right_finder(float c_gama) // return in radiunse
{
float c_gama_deg, n_gama_deg;
 
c_gama_deg = c_gama*180/PI;

if (abs(c_gama_deg - 90) < 180)
{
n_gama_deg = c_gama_deg - 90;
}
else if (abs(c_gama_deg - 90) >= 180)
{
n_gama_deg = c_gama_deg +270;
}

return PI*n_gama_deg/180;
}

float N_gama_back_finder(float c_gama) // return in radiunse
{
float c_gama_deg, n_gama_deg;
 
c_gama_deg = c_gama*180/PI;

if (abs(c_gama_deg + 180) < 180)
{
n_gama_deg = c_gama_deg + 180;
}
else if (abs(c_gama_deg + 180) >= 180)
{
n_gama_deg = c_gama_deg -180;
}

return PI*n_gama_deg/180;
}

float N_gama_stright_finder(float c_gama) // return in radiunse
{

return c_gama;
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
// %% functions 
int decision_maker(int cx,int cy,int px,int py,int bot_number,int entry_gate)
{

int Nxs=0, Nys=0, Nxl=0, Nyl=0, Nxr=0, Nyr=0, output = 0;
bool flag_r=0, flag_l=0, flag_s=0,never_l=0,never_r=0,never_s=0;

if(center_ir_sensor_data >= 150) // stright path is clear
	{
	Nxs = Nxs_finder(cx,cy,px,py);
	Nys = Nys_finder(cx,cy,px,py);
	flag_s = true;
	}
else
{
flag_s = false;
}

if(left_ir_sensor_data >= 150) // left path is clear
{
Nxl = Nxl_finder(cx,cy,px,py);
Nyl = Nyl_finder(cx,cy,px,py);
flag_l = true;
}
else
{
flag_l = false;
}

if(right_ir_sensor_data >= 150) // right path is clear
{
Nxr = Nxr_finder(cx,cy,px,py);
Nyr = Nyr_finder(cx,cy,px,py);
flag_r = true;
}
else
{
flag_r = false;
}

if(flag_s == 0 && flag_r == 0 && flag_l ==0)
	{	
	return -1; // means go back
	}
if (flag_s == true)
	{
	if (is_get_parking_at(Nxs,Nys))
	{
		output = 1;
		return output; // means go stright
	}
	}
if (flag_r == true)
	{
	if(is_get_parking_at(Nxr,Nyr))
	{
		output = 10;
		return output; // means go right
	}
	}
if (flag_l == true)
	{
	if(is_get_parking_at(Nxl,Nyl))
	{
		output = -10;
		return output; // means go left
	}
	}

if (entry_gate == 2 or entry_gate == 3){
if(flag_l == true)
	{
	if(!is_visited_before_at(Nxl,Nyl,bot_number))
	{
		output = -10;
		return output; // means go left
	}
	}	
if(flag_r == true)
	{
	if(!is_visited_before_at(Nxr,Nyr,bot_number))
	{
		output = 10;
		return output; // means go right
	}
	}
}
else if (entry_gate == 1 or entry_gate == 4){

	if(flag_r == true)
		{
		if(!is_visited_before_at(Nxr,Nyr,bot_number))
		{
			output = 10;
			return output; // means go right
		}
		}
	if(flag_l == true)
		{
		if(!is_visited_before_at(Nxl,Nyl,bot_number))
		{
			output = -10;
			return output; // means go left
		}
		}
}

if(flag_s == true)
	{
	if(!is_visited_before_at(Nxs,Nys,bot_number))
	{
		output = 1;
		return output; // means go stright
	}
	}


output = -1;
return output;


}


bool is_get_parking_at(int nx,int ny)
{
	bool output;
	if(is_parking_node[ny+1][nx+1] == true)
	{
		if(is_parking_ocupied[ny][nx] == true)
		{
			output = false;
			return output;
		}
		else 
		{
			output = true;
			return output;
		}
	}
	else 
	{
		output = false;
		return output;
	}

}

bool is_visited_before_at(int nx,int ny,int bot_number)
{
	if(is_node_visited_parking[bot_number][ny+1][nx+1] == true)
	{
	return true;
	}
	else
	{
	return false;
	}
	
}


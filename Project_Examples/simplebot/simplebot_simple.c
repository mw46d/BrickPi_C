/*
########################################################################                                                                 
# Program Name: simplebot_simple.c                                     
# ================================     
# This code is for moving the simplebot                                    
# http://www.dexterindustries.com/                                                                
# History
# ------------------------------------------------
# Author     Date      Comments
# Karan      04/11/13  Initial Authoring
#                                                                  
# These files have been made available online through a Creative Commons Attribution-ShareAlike 3.0  license.
# (http://creativecommons.org/licenses/by-sa/3.0/)           
#
########################################################################
*/
//Commands:
//	w-Move forward
//	a-Move left
//	d-Move right
//	s-Move back
//	x-Stop

#include <stdio.h>
#include <math.h>
#include <time.h>
#include "tick.h"
#include <wiringPi.h>
#include "BrickPi.h"
#include <linux/i2c-dev.h>  
#include <fcntl.h>
// gcc -o simplebot_simple simplebot_simple.c -lrt -lm -L/usr/local/lib -lwiringPi
// ./simplebot_simple

int result,speed = 70;//Set the speed
int motor1,motor2;
#undef DEBUG
//Move Forward
void fwd(void)
{
	motorSetSpeed(motor1 | motor2, speed);
}
//Move Left
void left(void)
{
	motorSetSpeed(motor1, speed);
	motorSetSpeed(motor2, -speed);
}
//Move Right
void right(void)
{
	motorSetSpeed(motor1, -speed);
	motorSetSpeed(motor2, speed);
}
//Move backward
void back(void)
{
	motorSetSpeed(motor1 | motor2, -speed);
}
//Stop
void stop(void)
{
	motorStop(motor1 | motor2, MOTOR_NEXT_ACTION_BRAKE);
}	

int main() 
{
	char inp;
	ClearTick();

	result = BrickPiSetup();
	// printf("BrickPiSetup: %d\n", result);
	if(result)
	return 0;

	BrickPi.Address[0] = 1;
	BrickPi.Address[1] = 2;

	motor1 = MOTOR_PORT_B;	//Select the ports to be used by the motors
	motor2 = MOTOR_PORT_C; 

	result = motorBankReset(motor1);
	result = motorBankReset(motor2);

	result = BrickPiSetupSensors();		//Set up the properties of sensors for the BrickPi
	//printf("BrickPiSetupSensors: %d\n", result); 
	BrickPi.Timeout=3000;				//Set timeout value for the time till which to run the motors after the last command is pressed
	BrickPiSetTimeout();				//Set the timeout
	if(!result)
	{
		while(1)
		{
			scanf("%c",&inp);	//Take input from the terminal
			//Move the bot
			if(inp=='w')
				fwd();  
			else if (inp=='a')
				left();
			else if (inp=='d')
				right();
			else if (inp=='s')
				back();
			else if (inp=='x')
				stop();
			BrickPiUpdateValues();	//Update the motor values
			usleep(20000);			//sleep for 20 ms
		}
	}
	return 0;
}

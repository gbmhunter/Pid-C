//!
//! @file 		Pid.h
//! @author 	Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
//! @edited 	Geoffrey Hunter (gbmhunter@gmail.com)
//! @date 		01/10/2012
//! @brief 		Header file for Pid.c
//! @details
//!		<b>Last Modified:			</b> 28/11/2011					\n
//!		<b>Version:					</b> v1.0.0						\n
//!		<b>Company:					</b> Beta Solutions				\n
//!		<b>Project:					</b> VicTrust BLDC Controller	\n
//!		<b>Language:				</b> C							\n
//!		<b>Compiler:				</b> GCC						\n
//! 	<b>uC Model:				</b> PSoC5						\n
//!		<b>Computer Architecture:	</b> ARM						\n
//! 	<b>Operating System:		</b> FreeRTOS v7.2.0			\n
//!		<b>Documentation Format:	</b> Doxygen					\n
//!		<b>License:					</b> GPLv3						\n
//!
//!		See Pid.c for a detailed description.
//!		

#ifndef _PID_H
#define _PID_H


//Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1

//commonly used functions **************************************************************************
void Pid_Init(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
    double, double, double, int);     //   Setpoint.  Initial tuning parameters are also set here

void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

void Compute();                       // * performs the PID calculation.  it should be
                                      //   called every time loop() cycles. ON/OFF and
                                      //   calculation frequency can be set using SetMode
                                      //   SetSampleTime respectively

void Pid_SetOutputLimits(double, double); //clamps the output to a specific range. 0-255 by default, but
									  //it's likely the user will want to change this depending on
									  //the application



//available but not commonly used functions ********************************************************
void Pid_SetTunings(double, double,       // * While most users will set the tunings once in the 
                double);         	  //   constructor, this function gives the user the option
                                      //   of changing tunings during runtime for Adaptive control
void Pid_SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
									  //   means the output will increase when error is positive. REVERSE
									  //   means the opposite.  it's very unlikely that this will be needed
									  //   once it is set in the constructor.
void Pid_SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                      //   the PID calculation is performed.  default is 100
									  
									  
									  
//Display functions ****************************************************************
double Pid_GetKp();						  // These functions query the pid for interal values.
double Pid_GetKi();						  //  they were created mainly for the pid front-end,
double Pid_GetKd();						  // where it's important to know what is actually 
int Pid_GetMode();						  //  inside the PID.
int Pid_GetDirection();					  //


void Pid_Initialize();




#endif


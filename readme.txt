Controls any lights (e.g. LEDS). More...

#include <device.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "PublicDefinesAndTypeDefs.h"
#include "Config.h"
#include "./Pid/Pid.h"
Functions

void 	Pid_Init (pidData_t *pidData, double kp, double ki, double kd, ctrlDir_t controllerDir, uint32_t samplePeriodMs)
 	Init function. 
void 	Pid_Run (pidData_t *pidData, double input)
 	Computes new PID values. 
void 	Pid_SetSetPoint (pidData_t *pidData, double setPoint)
void 	Pid_SetTunings (pidData_t *pidData, double kp, double ki, double kd)
 	This function allows the controller's dynamic performance to be adjusted. 
void 	Pid_SetSamplePeriod (pidData_t *pidData, uint32_t newSamplePeriodMs)
 	Changes the sample time. 
void 	Pid_SetOutputLimits (pidData_t *pidData, double min, double max)
void 	Pid_SetControllerDirection (pidData_t *pidData, ctrlDir_t controllerDir)
double 	Pid_GetKp (pidData_t *pidData)
 	Returns the porportional constant. 
double 	Pid_GetKi (pidData_t *pidData)
 	Returns the integral constant. 
double 	Pid_GetKd (pidData_t *pidData)
 	Returns the derivative constant. 
ctrlDir_t 	Pid_GetDirection (pidData_t *pidData)
 	Returns the direction. 
double 	Pid_GetError (pidData_t *pidData)
 	Returns the last calculated error. 
double 	Pid_GetDTerm (pidData_t *pidData)
 	Returns the last calculated derivative term. 
double 	Pid_GetInputChange (pidData_t *pidData)
 	Returns the last calculated input change. 
Detailed Description

Controls any lights (e.g. LEDS).

Author:
Geoffrey Hunter <gbmhunter@gmail.com> (www.cladlabs.com) n/a
Date:
09/11/2012
Last Modified: 15/10/2012 
Version: v1.1.0 
Company: CladLabs 
Project: Free Code Modules 
Language: C 
Compiler: GCC 
uC Model: PSoC5 
Computer Architecture: ARM 
Operating System: FreeRTOS v7.2.0 
Documentation Format: Doxygen 
License: GPLv3 
Since all parameters are passed in by sturcture pointer into PID controller functions, this file can work with as many seperate PID control loops as you wish.

CHANGELOG: v1.1.0 -> Changed the PID variables to they are stored in an external structure (passed in by pointer), so that this file can be used to control as many PID loops as you wish (kind of object-orientated).

Function Documentation

void Pid_Init	(	pidData_t * 	pidData,
double 	kp,
double 	ki,
double 	kd,
ctrlDir_t 	controllerDir,
uint32_t 	samplePeriodMs 
)		
Init function.

The parameters specified here are those for for which we can't set up reliable defaults, so we need to have the user set them.

void Pid_Run	(	pidData_t * 	pidData,
double 	input 
)		
Computes new PID values.

Call once per sampleTimeMs. Output is stored in the pidData structure.

void Pid_SetControllerDirection	(	pidData_t * 	pidData,
ctrlDir_t 	controllerDir 
)		
The PID will either be connected to a direct acting process (+error leads to +output, aka inputs are positive) or a reverse acting process (+error leads to -output, aka inputs are negative)

void Pid_SetTunings	(	pidData_t * 	pidData,
double 	kp,
double 	ki,
double 	kd 
)		
This function allows the controller's dynamic performance to be adjusted.

It's called automatically from the init function, but tunings can also be adjusted on the fly during normal operation
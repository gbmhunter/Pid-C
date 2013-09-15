//!
//! @file 		Pid.c
//! @author 	Geoffrey Hunter <gbmhunter@gmail.com> (www.cladlabs.com)
//! @edited 	n/a
//! @date 		09/10/2012
//! @brief 		Generic PID controller designed for microcontrollers that supports multiple control loops.
//! @details
//!				See the README in the root dir for more info.

//===============================================================================================//
//=========================================== GUARDS ============================================//
//===============================================================================================//

#ifdef __cplusplus
extern "C" {
#endif

//===============================================================================================//
//========================================= INCLUDES ============================================//
//===============================================================================================//

// GCC
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// User
#include "./include/Pid.h"

//===============================================================================================//
//================================== PRECOMPILER CHECKS =========================================//
//===============================================================================================//

#ifndef configPRINT_DEBUG_PID
	#warning configPRINT_DEBUG_PID is not defined.
#endif

//===============================================================================================//
//=================================== PRIVATE TYPEDEF's =========================================//
//===============================================================================================//

// none

//===============================================================================================//
//==================================== PRIVATE VARIABLES ========================================//
//===============================================================================================//

// none (all variables are held in structure passed into PID functions).

//===============================================================================================//
//====================================  FUNCTION PROTOTYPES =====================================//
//===============================================================================================//



//===============================================================================================//
//===================================== PUBLIC FUNCTIONS ========================================//
//===============================================================================================//

// See Doxygen documentation or function declarations in Pid.h for descriptions

void Pid_Init(
	pidData_t *pidData,
	double kp,
	double ki,
	double kd,
	ctrlDir_t controllerDir,
	uint32_t samplePeriodMs)
{

	Pid_SetOutputLimits(pidData, 0.0, 2000.0);		

    pidData->samplePeriodMs = samplePeriodMs;				// Default controller sample time is 0.1 seconds

    Pid_SetControllerDirection(pidData, controllerDir);
    Pid_SetTunings(pidData, kp, ki, kd);
	pidData->prevInput = 0;
	pidData->prevOutput = 0;
		
}
 

void Pid_Run(pidData_t *pidData, double input)
{
	// Compute all the working error variables
	//double input = *_input;
	
	pidData->error = pidData->setPoint - input;
	
	// Integral calcs
	
	pidData->iTerm += (pidData->zKi * pidData->error);
	// Perform min/max bound checking on integral term
	if(pidData->iTerm > pidData->outMax) 
	pidData->iTerm = pidData->outMax;
	else if(pidData->iTerm < pidData->outMin)
	pidData->iTerm = pidData->outMin;

	pidData->inputChange = (input - pidData->prevInput);
	pidData->dTerm = -pidData->zKd*pidData->inputChange;

	// Compute PID Output
	pidData->output = pidData->prevOutput + pidData->zKp*pidData->error + pidData->iTerm + pidData->dTerm;

	if(pidData->output > pidData->outMax) 
		pidData->output = pidData->outMax;
	else if(pidData->output < pidData->outMin)
		pidData->output = pidData->outMin;
	
	// Remember input value to next call
	pidData->prevInput = input;
	// Remember last output for next call
	pidData->prevOutput = pidData->output;
	  
}

void Pid_SetSetPoint(pidData_t *pidData, double setPoint)
{
	pidData->setPoint = setPoint;
}


void Pid_SetTunings(pidData_t *pidData, double kp, double ki, double kd)
{
   	if (kp<0 || ki<0 || kd<0) 
   		return;
 
 	pidData->actualKp = kp; 
	pidData->actualKi = ki;
	pidData->actualKd = kd;
   
   double sampleTimeInSec = ((double)pidData->samplePeriodMs)/1000.0;  
   
   // Calculate time-step-scaled PID terms
   pidData->zKp = kp;
   pidData->zKi = ki * sampleTimeInSec;
   pidData->zKd = kd / sampleTimeInSec;
 
  if(pidData->controllerDir == PID_REVERSE)
   {
      pidData->zKp = (0 - pidData->zKp);
      pidData->zKi = (0 - pidData->zKi);
      pidData->zKd = (0 - pidData->zKd);
   }
}


void Pid_SetSamplePeriod(pidData_t *pidData, uint32_t newSamplePeriodMs)
{
   if (newSamplePeriodMs > 0)
   {
      double ratio  = (double)newSamplePeriodMs
                      / (double)pidData->samplePeriodMs;
      pidData->zKi *= ratio;
      pidData->zKd /= ratio;
      pidData->samplePeriodMs = newSamplePeriodMs;
   }
}
 
void Pid_SetOutputLimits(pidData_t *pidData, double min, double max)
{
	if(min >= max) 
   		return;
   	pidData->outMin = min;
   	pidData->outMax = max;
 
}


void Pid_SetControllerDirection(pidData_t *pidData, ctrlDir_t controllerDir)
{
	if(controllerDir != pidData->controllerDir)
	{
   		// Invert control constants
		pidData->zKp = (0 - pidData->zKp);
    	pidData->zKi = (0 - pidData->zKi);
    	pidData->zKd = (0 - pidData->zKd);
	}   
   pidData->controllerDir = controllerDir;
}
 

double Pid_GetKp(pidData_t *pidData)
{ 
	return pidData->actualKp;
}


double Pid_GetKi(pidData_t *pidData)
{ 
	return pidData->actualKi;
}


double Pid_GetKd(pidData_t *pidData)
{ 
	return pidData->actualKd;
}


ctrlDir_t Pid_GetDirection(pidData_t *pidData)
{ 
	return pidData->controllerDir;
}


double Pid_GetError(pidData_t *pidData)
{
	return pidData->error;
}


double Pid_GetDTerm(pidData_t *pidData)
{
	return pidData->dTerm;
}


double Pid_GetInputChange(pidData_t *pidData)
{
	return pidData->inputChange;
}

#ifdef __cplusplus
} // extern "C" {
#endif

// EOF

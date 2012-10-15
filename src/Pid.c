//!
//! @file 		Pid.c
//! @author 	Geoffrey Hunter <gbmhunter@gmail.com> (www.cladlabs.com)
//! @edited 	n/a
//! @date 		09/11/2012
//! @brief 		Generic PID controller that supports multiple control loops.
//! @details
//!		<b>Last Modified:			</b> 15/10/2012					\n
//!		<b>Version:					</b> v1.1.0						\n
//!		<b>Company:					</b> CladLabs					\n
//!		<b>Project:					</b> Free Code Modules			\n
//!		<b>Language:				</b> C							\n
//!		<b>Compiler:				</b> GCC						\n
//! 	<b>uC Model:				</b> PSoC5						\n
//!		<b>Computer Architecture:	</b> ARM						\n
//! 	<b>Operating System:		</b> FreeRTOS v7.2.0			\n
//!		<b>Documentation Format:	</b> Doxygen					\n
//!		<b>License:					</b> GPLv3						\n
//!
//! 	Since all parameters are passed in by sturcture pointer into PID controller functions,
//!		this file can work with as many seperate PID control loops as you wish.
//!
//! 	CHANGELOG:
//! 	v1.1.0 -> Changed the PID variables to they are stored in an external structure
//!		(passed in by pointer), so that this file can be used to control as many PID loops
//!		as you wish (kind of object-orientated).
//!					


//===============================================================================================//
//========================================= INCLUDES ============================================//
//===============================================================================================//

// PSoC
#include <device.h>

// GCC
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// User
#include "PublicDefinesAndTypeDefs.h"
#include "Config.h"
#include "./Pid/Pid.h"

//===============================================================================================//
//================================== PRECOMPILER CHECKS =========================================//
//===============================================================================================//

#ifndef configPRINT_DEBUG_PID
	#error Please define the switch configPRINT_DEBUG_PID
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

// See function definitions for descriptions

//===============================================================================================//
//===================================== PUBLIC FUNCTIONS ========================================//
//===============================================================================================//

//! @brief 		Init function
//! @details   	The parameters specified here are those for for which we can't set up 
//!    			reliable defaults, so we need to have the user set them.
//! @public
void Pid_Init(pidData_t *pidData, double kp, double ki, double kd, ctrlDir_t controllerDir, uint32_t samplePeriodMs)
{

	Pid_SetOutputLimits(pidData, 0.0, 2000.0);		

    pidData->samplePeriodMs = samplePeriodMs;				// Default controller sample time is 0.1 seconds

    Pid_SetControllerDirection(pidData, controllerDir);
    Pid_SetTunings(pidData, kp, ki, kd);
	pidData->prevInput = 0;
	pidData->prevOutput = 0;
		
}
 
//! @brief 		Computes new PID values
//! @details 	Call once per sampleTimeMs. Output is stored in the pidData structure.
//! @public
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

//! @brief		This function allows the controller's dynamic performance to be adjusted. 
//! @details	It's called automatically from the init function, but tunings can also
//! 			be adjusted on the fly during normal operation
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

//! @brief		Changes the sample time
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

//! @details	The PID will either be connected to a direct acting process (+error leads to +output, aka inputs are positive) 
//!				or a reverse acting process (+error leads to -output, aka inputs are negative)
//! @public
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
 
//! @brief		Returns the porportional constant
//! @public
double Pid_GetKp(pidData_t *pidData)
{ 
	return pidData->actualKp;
}

//! @brief		Returns the integral constant
//! @public
double Pid_GetKi(pidData_t *pidData)
{ 
	return pidData->actualKi;
}

//! @brief		Returns the derivative constant
//! @public
double Pid_GetKd(pidData_t *pidData)
{ 
	return pidData->actualKd;
}

//! @brief		Returns the direction
//! @public
ctrlDir_t Pid_GetDirection(pidData_t *pidData)
{ 
	return pidData->controllerDir;
}

//! @brief		Returns the last calculated error
//! @public
double Pid_GetError(pidData_t *pidData)
{
	return pidData->error;
}

//! @brief		Returns the last calculated derivative term
//! @public
double Pid_GetDTerm(pidData_t *pidData)
{
	return pidData->dTerm;
}

//! @brief		Returns the last calculated input change
//! @public
double Pid_GetInputChange(pidData_t *pidData)
{
	return pidData->inputChange;
}

// EOF
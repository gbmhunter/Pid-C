//!
//! @file 		Pid.c
//! @author 	Geoffrey Hunter <gbmhunter@gmail.com> (www.cladlabs.com)
//! @edited 	n/a
//! @date 		09/11/2012
//! @brief 		Controls any lights (e.g. LEDS).
//! @details
//!		<b>Last Modified:			</b> 02/10/2012					\n
//!		<b>Version:					</b> v1.0.0						\n
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


//===============================================================================================//
//========================================= INCLUDES ============================================//
//===============================================================================================//

// PSoC
#include <device.h>

// GCC
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

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

double _dispKp;				// * we'll hold on to the tuning parameters in user-entered 
double _dispKi;				//   format for display purposes
double _dispKd;				//

double _kp;                  // * (P)roportional Tuning Parameter
double _ki;                  // * (I)ntegral Tuning Parameter
double _kd;                  // * (D)erivative Tuning Parameter

controllerDirection_t _controllerDirection;

double* _input;              // * Pointers to the Input, Output, and Setpoint variables
double* _output;             //   This creates a hard link between the variables and the 
double* _setPoint;           //   PID, freeing the user from having to constantly tell us
                              //   what these values are.  with pointers we'll just know.
		  
double _error;
double _dTerm;
		  
uint32_t _lastTime;
double _iTerm;
double _lastInput;
double _inputChange;

uint32_t _sampleTimeMs;
double _outMin, _outMax;
bool_t _inAuto;

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
void Pid_Init(double kp, double ki, double kd, controllerDirection_t controllerDirection, uint32_t sampleTimeMs)
{
	Pid_SetOutputLimits(0, 255);				//default output limit corresponds to 
												//the arduino pwm limits

    _sampleTimeMs = sampleTimeMs;				// Default controller sample time is 0.1 seconds

    Pid_SetControllerDirection(controllerDirection);
    Pid_SetTunings(kp, ki, kd);

    //lastTime = millis()-SampleTime;				
    //_inAuto = FALSE;
    //_output = output;
    //_input = input;
    //_setPoint = setPoint;
		
}
 
//! @brief 		Computes new PID values
//! @details 	Call once per sampleTimeMs as determine when Pid_Init() is called
//! @public
double Pid_Compute(double setPoint, double input)
{
	static double lastInput = 0;
	static double prevOutput = 0;
	// Compute all the working error variables
	//double input = *_input;
	_error = setPoint - input;
	
	// Integral calcs
	
	_iTerm += (_ki * _error);
	// Perform min/max bound checking on integral term
	if(_iTerm > _outMax) 
	_iTerm = _outMax;
	else if(_iTerm < _outMin)
	_iTerm = _outMin;

	_inputChange = (input - lastInput);
	_dTerm = -_kd*_inputChange;

	// Compute PID Output
	double output = prevOutput + _kp*_error + _iTerm + _dTerm;

	if(output > _outMax) 
		output = _outMax;
	else if(output < _outMin)
		output = _outMin;
	
	lastInput = input;
	prevOutput = output;

	return output;
	  
}

//! @brief		This function allows the controller's dynamic performance to be adjusted. 
//! @details	It's called automatically from the init function, but tunings can also
//! 			be adjusted on the fly during normal operation
void Pid_SetTunings(double kp, double ki, double kd)
{
   	if (kp<0 || ki<0 || kd<0) 
   		return;
 
 	_dispKp = kp; 
	_dispKi = ki;
	_dispKd = kd;
   
   double sampleTimeInSec = ((double)_sampleTimeMs)/1000.0;  
   _kp = kp;
   _ki = ki * sampleTimeInSec;
   _kd = kd / sampleTimeInSec;
 
  if(_controllerDirection == PID_REVERSE)
   {
      _kp = (0 - _kp);
      _ki = (0 - _ki);
      _kd = (0 - _kd);
   }
}

//! @brief		Changes the sample time
void Pid_SetSampleTime(uint32_t newSampleTimeMs)
{
   if (newSampleTimeMs > 0)
   {
      double ratio  = (double)newSampleTimeMs
                      / (double)_sampleTimeMs;
      _ki *= ratio;
      _kd /= ratio;
      _sampleTimeMs = newSampleTimeMs;
   }
}
 
void Pid_SetOutputLimits(double min, double max)
{
	if(min >= max) 
   		return;
   	_outMin = min;
   	_outMax = max;
 
}

//! @details	The PID will either be connected to a direct acting process (+error leads to +output, aka inputs are positive) 
//!				or a reverse acting process (+error leads to -output, aka inputs are negative)
//! @public
void Pid_SetControllerDirection(controllerDirection_t controllerDirection)
{
	if(_inAuto && controllerDirection != _controllerDirection)
	{
   		// Invert control constants
		_kp = (0 - _kp);
    	_ki = (0 - _ki);
    	_kd = (0 - _kd);
	}   
   _controllerDirection = controllerDirection;
}
 
//! @brief		Returns the porportional constant
//! @public
double Pid_GetKp()
{ 
	return _dispKp;
}

//! @brief		Returns the integral constant
//! @public
double Pid_GetKi()
{ 
	return _dispKi;
}

//! @brief		Returns the derivative constant
//! @public
double Pid_GetKd()
{ 
	return _dispKd;
}

//! @brief		Returns the direction
//! @public
controllerDirection_t Pid_GetDirection()
{ 
	return _controllerDirection;
}

//! @brief		Returns the last calculated error
//! @public
double Pid_GetError()
{
	return _error;
}

//! @brief		Returns the last calculated derivative term
//! @public
double Pid_GetDTerm()
{
	return _dTerm;
}

//! @brief		Returns the last calculated input change
//! @public
double Pid_GetInputChange()
{
	return _inputChange;
}
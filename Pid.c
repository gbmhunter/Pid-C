//!
//! @file 		Pid.c
//! @author 	Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
//! @edited 	Geoffrey Hunter (gbmhunter@gmail.com)
//! @date 		09/11/2012
//! @brief 		Controls any lights (e.g. LEDS).
//! @details
//!		<b>Last Modified:			</b> 28/11/2011					\n
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
//! From the Ardunio PID library v1.0.1.
//! 
//! Converted from C++ to C by Geoff. Modified comments for Doxygen formatting
//! Ported from Ardunio to PSoC. Removed millis() function calls.
//! Changed behaviour slightly, now designed to be called
//! from interrupt or task at the correct time rather than
//! contiuously called from a infinite loop.

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

#include "./Pid/PID_v1.h"

double dispKp;				// * we'll hold on to the tuning parameters in user-entered 
double dispKi;				//   format for display purposes
double dispKd;				//

double kp;                  // * (P)roportional Tuning Parameter
double ki;                  // * (I)ntegral Tuning Parameter
double kd;                  // * (D)erivative Tuning Parameter

int controllerDirection;

double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
double *myOutput;             //   This creates a hard link between the variables and the 
double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                              //   what these values are.  with pointers we'll just know.
		  
unsigned long lastTime;
double ITerm;
double lastInput;

unsigned long SampleTime;
double outMin, outMax;
bool_t inAuto;

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void Pid_Init(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
{
	Pid_SetOutputLimits(0, 255);				//default output limit corresponds to 
												//the arduino pwm limits

    SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

    Pid_SetControllerDirection(ControllerDirection);
    Pid_SetTunings(Kp, Ki, Kd);

    //lastTime = millis()-SampleTime;				
    inAuto = FALSE;
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
		
}
 
//! @brief 		Computes new PID values
//! @details 	This, as they say, is where the magic happens.  this function should be called
//!   			every time "void loop()" executes.  the function will decide for itself whether a new
//!   			pid Output needs to be computed
void Pid_Compute()
{
   if(!inAuto) return;

      // Compute all the working error variables
	  double input = *myInput;
      double error = *mySetpoint - input;
      ITerm+= (ki * error);
	  
	  // Perform min/max bound checking on integral term
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
	  
      double dInput = (input - lastInput);
 
      /*Compute PID Output*/
      double output = kp * error + ITerm- kd * dInput;
      
	  if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	  *myOutput = output;
	  
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void Pid_SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
 
   dispKp = Kp; dispKi = Ki; dispKd = Kd;
   
   double SampleTimeInSec = ((double)SampleTime)/1000;  
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
 
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void Pid_SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void Pid_SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
 
   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;
	 
	   if(ITerm > outMax) ITerm= outMax;
	   else if(ITerm < outMin) ITerm= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void Pid_SetMode(int Mode)
{
    bool_t newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        Pid_Initialize();
    }
    inAuto = newAuto;
}
 
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void Pid_Initialize()
{
   ITerm = *myOutput;
   lastInput = *myInput;
   if(ITerm > outMax) ITerm = outMax;
   else if(ITerm < outMin) ITerm = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void Pid_SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	  kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }   
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double Pid_GetKp(){ return  dispKp; }
double Pid_GetKi(){ return  dispKi;}
double Pid_GetKd(){ return  dispKd;}
int Pid_GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int Pid_GetDirection(){ return controllerDirection;}


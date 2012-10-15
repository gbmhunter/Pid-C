//!
//! @file 		Pid.h
//! @author 	Geoffrey Hunter <gbmhunter@gmail.com> (www.cladlabs.com)
//! @edited 	n/a
//! @date 		09/11/2012
//! @brief 		Header file for Pid.c
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
//!		See the Doxygen documentation or Pid.c for a detailed description on this module.
//!		

// Start of header guard
#ifndef _PID_H
#define _PID_H

//===============================================================================================//
//=================================== PUBLIC TYPEDEFS ===========================================//
//===============================================================================================//

typedef enum			//!< Enumerates the controller direction modes
{
	PID_DIRECT,			//!< Direct drive (+error gives +output)
	PID_REVERSE			//!< Reverse driver (+error gives -output)
} ctrlDir_t;

//! Used to create a PID object for passing into functions.
//! Not all these variables have to be remembered, but storing them is useful for debugging.
typedef struct
{
	double zKp;					//!< Time-step scaled proportional constant for quick calculation (equal to actualKp)
	double zKi;					//!< Time-step scaled integral constant for quick calculation
	double zKd;					//!< Time-step scaled derivative constant for quick calculation
	double actualKp;			//!< Actual (non-scaled) proportional constant
	double actualKi;			//!< Actual (non-scaled) integral constant
	double actualKd;			//!< Actual (non-scaled) derivative constant
	double prevInput;			//!< Actual (non-scaled) proportional constant
	double inputChange;			//!< The change in input between the current and previous value
	double setPoint;			//!< The set-point the PID control is trying to make the output converge to.
	double error;				//!< The error between the set-point and actual output (set point - output, positive
								//!< when actual output is lagging set-point
	double output;				//!< The control output. This is updated when Pid_Go() is called
	double prevOutput;			//!< The output value calculated the previous time Pid_Go was called
	double samplePeriodMs;		//!< The sample period (in milliseconds) between successive Pid_Go() calls.
								//!< The constants with the z prefix are scaled according to this value.
	double pTerm;				//!< The proportional term that is summed as part of the output (calculated in Pid_Go())
	double iTerm;				//!< The integral term that is summed as part of the output (calculated in Pid_Go())
	double dTerm;				//!< The derivative term that is summed as part of the output (calculated in Pid_Go())
	double outMin;				//!< The minimum output value. Anything lower will be limited to this floor.
	double outMax;				//!< The maximum output value. Anything higher will be limited to this ceiling.
	ctrlDir_t controllerDir;
} pidData_t;

//===============================================================================================//
//=================================== PUBLIC FUNCTION PROTOTYPES ================================//
//===============================================================================================//

// See the Doxygen documentation or the function definitions in Pid.c for more information

void 	Pid_Init(pidData_t *pidData, double kp, double ki, double kd, ctrlDir_t controllerDir, uint32_t sampleTimeMs);
void 	Pid_Run(pidData_t *pidData, double input);
double 	Pid_GetError(pidData_t *pidData);      
void	Pid_SetSetPoint(pidData_t *pidData, double setPoint);
void 	Pid_SetControllerDirection(pidData_t *pidData, ctrlDir_t controllerDir);
void 	Pid_SetSamplePeriod(pidData_t *pidData, uint32_t newSamplePeriodMs);
double 	Pid_GetKp(pidData_t *pidData);						
double 	Pid_GetKi(pidData_t *pidData);						 
double 	Pid_GetKd(pidData_t *pidData);						  
ctrlDir_t Pid_GetDirection(pidData_t *pidData);					
double 	Pid_GetDTerm(pidData_t *pidData);
double 	Pid_GetInputChange(pidData_t *pidData);
void 	Pid_SetTunings(pidData_t *pidData, double kp, double ki, double kd);
void	Pid_SetOutputLimits(pidData_t *pidData, double min, double max);

#endif

// EOF
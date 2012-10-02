//!
//! @file 		Pid.h
//! @author 	Geoffrey Hunter <gbmhunter@gmail.com> (www.cladlabs.com)
//! @edited 	n/a
//! @date 		09/11/2012
//! @brief 		Header file for Pid.c
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
//!		See the Doxygen documentation or Pid.c for a detailed description.
//!		

#ifndef _PID_H
#define _PID_H

//===============================================================================================//
//=================================== PUBLIC TYPEDEFS ===========================================//
//===============================================================================================//

typedef enum			//!< Enumerates the controller direction modes
{
	PID_DIRECT,			//!< Direct drive (+error gives +output)
	PID_REVERSE			//!< Reverse driver (+error gives -output)
} controllerDirection_t;

//===============================================================================================//
//=================================== PUBLIC FUNCTION PROTOTYPES ================================//
//===============================================================================================//

// See the Doxygen documentation or the function definitions in Pid.c for more information

void 	Pid_Init(double kp, double ki, double kd, controllerDirection_t controllerDirection, uint32_t sampleTimeMs);
double 	Pid_Compute(double setPoint, double input);
double 	Pid_GetError();      
void 	Pid_SetControllerDirection(controllerDirection_t controllerDirection);
void 	Pid_SetSampleTime(uint32_t newSampleTimeMs);
double 	Pid_GetKp();						
double 	Pid_GetKi();						 
double 	Pid_GetKd();						  
controllerDirection_t Pid_GetDirection();					
double 	Pid_GetDTerm();
double 	Pid_GetInputChange();
void 	Pid_SetTunings(double kp, double ki, double kd);
void	Pid_SetOutputLimits();

#endif

// EOF
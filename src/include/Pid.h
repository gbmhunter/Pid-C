//!
//! @file 		Pid.h
//! @author 	Geoffrey Hunter <gbmhunter@gmail.com> (www.cladlabs.com)
//! @edited 	n/a
//! @date 		09/10/2012
//! @brief 		Generic PID controller designed for microcontrollers that supports multiple control loops.
//! @details
//!				See the README in root dir for more info.

//===============================================================================================//
//=========================================== GUARDS ============================================//
//===============================================================================================//

// Start of header guard
#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

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
	ctrlDir_t controllerDir;	//!< The control direction for the PID instance.
} pidData_t;

//===============================================================================================//
//=================================== PUBLIC FUNCTION PROTOTYPES ================================//
//===============================================================================================//

//! @brief 		Init function
//! @details   	The parameters specified here are those for for which we can't set up 
//!    			reliable defaults, so we need to have the user set them.
//! @public
void Pid_Init(
	pidData_t *pidData,
	double kp,
	double ki,
	double kd,
	ctrlDir_t controllerDir,
	uint32_t samplePeriodMs);

//! @brief 		Computes new PID values
//! @details 	Call once per sampleTimeMs. Output is stored in the pidData structure.
//! @public
void 	Pid_Run(pidData_t *pidData, double input);

//! @brief		Returns the last calculated error
//! @public
double 	Pid_GetError(pidData_t *pidData);      
void	Pid_SetSetPoint(pidData_t *pidData, double setPoint);

//! @details	The PID will either be connected to a direct acting process (+error leads to +output, aka inputs are positive) 
//!				or a reverse acting process (+error leads to -output, aka inputs are negative)
//! @public
void 	Pid_SetControllerDirection(pidData_t *pidData, ctrlDir_t controllerDir);

//! @brief		Changes the sample time
void 	Pid_SetSamplePeriod(pidData_t *pidData, uint32_t newSamplePeriodMs);

//! @brief		Returns the porportional constant
//! @public
double 	Pid_GetKp(pidData_t *pidData);	

//! @brief		Returns the integral constant
//! @public					
double 	Pid_GetKi(pidData_t *pidData);	

//! @brief		Returns the derivative constant
//! @public					 
double 	Pid_GetKd(pidData_t *pidData);		

//! @brief		Returns the direction
//! @public				  
ctrlDir_t Pid_GetDirection(pidData_t *pidData);	

//! @brief		Returns the last calculated derivative term
//! @public				
double 	Pid_GetDTerm(pidData_t *pidData);

//! @brief		Returns the last calculated input change
//! @public
double 	Pid_GetInputChange(pidData_t *pidData);

//! @brief		This function allows the controller's dynamic performance to be adjusted. 
//! @details	It's called automatically from the init function, but tunings can also
//! 			be adjusted on the fly during normal operation
void 	Pid_SetTunings(pidData_t *pidData, double kp, double ki, double kd);
void	Pid_SetOutputLimits(pidData_t *pidData, double min, double max);

#ifdef __cplusplus
} // extern "C" {
#endif

#endif // #ifndef PID_H

// EOF

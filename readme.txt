Detailed Description

Generic PID controller that supports multiple control loops.

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
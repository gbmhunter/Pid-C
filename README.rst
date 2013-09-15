==============================================================
Reusable C PID Library
==============================================================

- Author: gbmhunter <gbmhunter@gmail.com> (http://www.cladlab.com)
- Created: 2012/11/09
- Last Modified: 2013/09/16
- Version: v1.1.2.0
- Company: CladLabs
- Project: n/a
- Language: C
- Compiler: GCC	
- uC Model: all
- Computer Architecture: all
- Operating System: n/a
- Documentation Format: Doxygen
- License: GPLv3

Description
===========

Reusable PID controller that supports multiple control loops.

Internal Dependencies
=====================

None

External Dependencies
=====================

None

Issues
======

See GitHub issues section

Limitations
===========

None documented

Usage
=====

Since all parameters are passed in by structure pointer into PID controller functions, this file can work with as many separate PID control loops as you wish.

::

	coming soon...
	
Changelog
=========

======== ========== ===================================================================================================
Version  Date       Comment
======== ========= ===================================================================================================
v1.1.2.0 2013/09/16 Deleted old Mercurial repo files (.hgignore and .hgtags). Fixed capaitilisation of some of the README section names.
v1.1.1.1 2013/06/04 Modified README.txt to README.rst.
v1.1.1.0 2012/11/04 Moved function documentation from Pid.c into Pid.h. Currently v1.1.1.0
v1.1.0.0 2012/10/14 Made PID functions use external structures, added readme file. PID variables stored in struct.
v1.0.0.0 2012/09/30 Initial commit.
======== ========== ===================================================================================================

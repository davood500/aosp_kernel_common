/****************************************************************************
*
*    Copyright (C) 2005 - 2010 by Vivante Corp.
*
*    This program is free software; you can redistribute it and/or modify
*    it under the terms of the GNU General Public Lisence as published by
*    the Free Software Foundation; either version 2 of the license, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*    GNU General Public Lisence for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not write to the Free Software
*    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*
*****************************************************************************/




#ifndef __gcoptions_h_
#define __gcoptions_h_

/*
    USE_EVENT_QUEUE

    This define enables the new event management code.  Instead of using one
    interrupt per event, this define will create one interrupt per commit.
*/
#define USE_EVENT_QUEUE				1

/*
    USE_MEMORY_HEAP

    This define enables the user memory heap.  This will reduce the system
    memory fragmentation and increase performance.
*/
#define USE_MEMORY_HEAP				1

/*
    USE_SHADER_SYMBOL_TABLE

    This define enables the symbol table in shader object.
*/
#define USE_SHADER_SYMBOL_TABLE		1

/*
    USE_SUPER_SAMPLING

    This define enables super-sampling support.
*/
#define USE_SUPER_SAMPLING			0

/*
    PROFILE_HAL_COUNTERS

    This define enables HAL counter profiling support.
    HW and SHADER Counter profiling depends on this.
*/
#define PROFILE_HAL_COUNTERS		1

/*
    PROFILE_HW_COUNTERS

    This define enables HW counter profiling support.
*/
#define PROFILE_HW_COUNTERS			1

/*
    PROFILE_SHADER_COUNTERS

    This define enables SHADER counter profiling support.
*/
#define PROFILE_SHADER_COUNTERS		1

/*
    USE_VALIDATION

    This define enables local validation code and means different things
	depending on the context.  This is used for debugging only.
*/
#define USE_VALIDATION				0

/*
    COMMAND_PROCESSOR_VERSION

    The version of the command buffer and task manager.
*/
#define COMMAND_PROCESSOR_VERSION	1

/*
    USE_COMMAND_BUFFER_POOL

    This define enables the new command buffer pool code.
*/
#define USE_COMMAND_BUFFER_POOL		1

/*
	gcdDUMP

	This define is used to turn on dumping for playback.
*/
#ifndef gcdDUMP
#  define gcdDUMP					0
#endif

/*
	gcdNULL_DRIVER
*/
#define gcdNULL_DRIVER				0

/*
	gcdENABLE_TIMEOUT_DETECTION

	Enable timeout detection.
*/
#define gcdENABLE_TIMEOUT_DETECTION	0

#endif /* __gcoptions_h_ */


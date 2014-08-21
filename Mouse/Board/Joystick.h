/*
			 LUFA Library
			 Copyright (C) Dean Camera, 2014.

			 dean [at] fourwalledcubicle [dot] com
			 www.lufa-lib.org
			 */

/*
  Copyright 2014  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
  */

/** \file
 *  \brief Digital joystick board hardware driver.
 *
 *  This file is the master dispatch header file for the board-specific Joystick driver, for boards containing a
 *  digital joystick.
 *
 *  User code should include this file, which will in turn include the correct joystick driver header file for the
 *  currently selected board.
 *
 *  If the \c BOARD value is set to \c BOARD_USER, this will include the \c /Board/Joystick.h file in the user project
 *  directory.
 *
 *  For possible \c BOARD makefile values, see \ref Group_BoardTypes.
 */

/** \ingroup Group_BoardDrivers
 *  \defgroup Group_Joystick Joystick Driver - LUFA/Drivers/Board/Joystick.h
 *  \brief Digital joystick board hardware driver.
 *
 *  \section Sec_Joystick_Dependencies Module Source Dependencies
 *  The following files must be built with any user project that uses this module:
 *    - None
 *
 *  \section Sec_Joystick_ModDescription Module Description
 *  Hardware Joystick driver. This module provides an easy to use interface to control the hardware digital Joystick
 *  located on many boards.
 *
 *  If the \c BOARD value is set to \c BOARD_USER, this will include the \c /Board/Joystick.h file in the user project
 *  directory. Otherwise, it will include the appropriate built-in board driver header file.
 *
 *  For possible \c BOARD makefile values, see \ref Group_BoardTypes.
 *
 *  \section Sec_Joystick_ExampleUsage Example Usage
 *  The following snippet is an example of how this module may be used within a typical
 *  application.
 *
 *  \code
 *      // Initialize the board Joystick driver before first use
 *      Joystick_Init();
 *
 *      printf("Waiting for joystick movement...\r\n");
 *
 *      // Loop until a the joystick has been moved
 *      uint8_t JoystickMovement;
 *      while (!(JoystickMovement = Joystick_GetStatus())) {};
 *
 *      // Display which direction the joystick was moved in
 *      printf("Joystick moved:\r\n");
 *
 *      if (JoystickMovement & (JOY_UP | JOY_DOWN))
 *        printf("%s ", (JoystickMovement & JOY_UP) ? "Up" : "Down");
 *
 *      if (JoystickMovement & (JOY_LEFT | JOY_RIGHT))
 *        printf("%s ", (JoystickMovement & JOY_LEFT) ? "Left" : "Right");
 *
 *      if (JoystickMovement & JOY_PRESS)
 *        printf("Pressed");
 *  \endcode
 *
 *  @{
 */

#ifndef __JOYSTICK_ARDUINO_H__
#define __JOYSTICK_ARDUINO_H__

/* Includes: */
#include <LUFA/Common/Common.h>

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

	/* Preprocessor Checks: */
#if !defined(__INCLUDE_FROM_JOYSTICK_H)
#error Do not include this file directly. Include LUFA/Drivers/Board/Joystick.h instead.
#endif

#define JOY_DOWN 0
#define JOY_UP 0
#define JOY_LEFT 0
#define JOY_RIGHT 0
#define JOY_PRESS 0

	/* Pseudo-Functions for Doxygen: */
#if !defined(__DOXYGEN__)
	/** Initializes the joystick driver so that the joystick position can be read. This sets the appropriate
	 *  I/O pins to inputs with their pull-ups enabled.
	 *
	 *  This must be called before any Joystick driver functions are used.
	 */
		static inline void Joystick_Init(void){

	}

	/** Disables the joystick driver, releasing the I/O pins back to their default high-impedance input mode. */
		static inline void Joystick_Disable(void){


	}

	/** Returns the current status of the joystick, as a mask indicating the direction the joystick is
	 *  currently facing in (multiple bits can be set).
	 *
	 *  \return Mask of \c JOYSTICK_* constants indicating the current joystick direction(s).
	 */
		static inline uint_reg_t Joystick_GetStatus(void) ATTR_WARN_UNUSED_RESULT;
		static inline uint_reg_t Joystick_GetStatus(void) {
		return 0;
	}
#endif

	/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

#endif

/** @} */


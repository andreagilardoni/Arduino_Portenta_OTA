/*
   This file is part of Arduino_Portenta_OTA.

   Copyright 2024 ARDUINO SA (http://www.arduino.cc/)

   This software is released under the GNU General Public License version 3,
   which covers the main part of arduino-cli.
   The terms of this license can be found at:
   https://www.gnu.org/licenses/gpl-3.0.en.html

   You can be released from the requirements of the above licenses by purchasing
   a commercial license. Buying such a license is mandatory if you want to modify or
   otherwise use the software for commercial activities involving the Arduino
   software without disclosing the source code of your own applications. To purchase
   a commercial license, send an email to license@arduino.cc.
*/

#ifndef ARDUINO_PORTENTA_OTA_DEBUG_H_
#define ARDUINO_PORTENTA_OTA_DEBUG_H_

/******************************************************************************
 * INCLUDE
 ******************************************************************************/

#if defined __has_include
  #if __has_include (<Arduino_DebugUtils.h>)
    #include <Arduino_DebugUtils.h>
  #else
    #define DEBUG_ERROR(fmt, ...)
    #define DEBUG_WARNING(fmt, ...)
    #define DEBUG_INFO(fmt, ...)
    #define DEBUG_DEBUG(fmt, ...)
    #define DEBUG_VERBOSE(fmt, ...)
  #endif
#else
    #define DEBUG_ERROR(fmt, ...)
    #define DEBUG_WARNING(fmt, ...)
    #define DEBUG_INFO(fmt, ...)
    #define DEBUG_DEBUG(fmt, ...)
    #define DEBUG_VERBOSE(fmt, ...)
#endif

#endif /* ARDUINO_PORTENTA_OTA_DEBUG_H_ */

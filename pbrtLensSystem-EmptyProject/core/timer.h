
/*
    pbrt source code Copyright(c) 1998-2010 Matt Pharr and Greg Humphreys.

    This file is part of pbrt.

    pbrt is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.  Note that the text contents of
    the book "Physically Based Rendering" are *not* licensed under the
    GNU GPL.

    pbrt is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 */

#ifndef PBRT_TIMER_H
#define PBRT_TIMER_H
// timer.h*
#include "pbrt.h"
#if defined ( WIN32 )
#include <windows.h>
#else
#include <sys/time.h>
#endif
#if (_MSC_VER >= 1400)
#include <stdio.h>
#define snprintf _snprintf
#endif
// Timer Declarations
class COREDLL Timer {
public:
	// Public Timer Methods
	Timer();
	~Timer();
	
	void Start();
	void Stop();
	void Reset();
	
	double Time();
private:
	// Private Timer Data
	double time0, elapsed;
	bool running;
	double GetTime();
#if defined( IRIX ) || defined( IRIX64 )
	// Private IRIX Timer Data
	int fd;
	unsigned long long counter64;
	unsigned int counter32;
	unsigned int cycleval;
	
	typedef unsigned long long iotimer64_t;
	typedef unsigned int iotimer32_t;
	volatile iotimer64_t *iotimer_addr64;
	volatile iotimer32_t *iotimer_addr32;
	
	void *unmapLocation;
	int unmapSize;
#elif defined( WIN32 )
	// Private Windows Timer Data
	LARGE_INTEGER performance_counter, performance_frequency;
	double one_over_frequency;
#else
	// Private UNIX Timer Data
	struct timeval timeofday;
#endif
};
#endif // PBRT_TIMER_H

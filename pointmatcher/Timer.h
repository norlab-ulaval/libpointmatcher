// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2011,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef __POINTMATCHER_TIMER_H
#define __POINTMATCHER_TIMER_H

#include <sys/time.h>

namespace PointMatcherSupport
{
	/*
		High-precision timer class, using gettimeofday().
		The interface is a subset of the one boost::timer provides,
		but the implementation is much more precise
		on systems where clock() has low precision, such as glibc.
	*/
	struct timer
	{
		typedef unsigned long long Time;
		
		timer():_start_time(curTime()){ } 
		void restart() { _start_time = curTime(); }
		double elapsed() const                  // return elapsed time in seconds
		{ return  double(curTime() - _start_time) / double(1000000000); }

	private:
		Time curTime() const {
			struct timespec ts;
			clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
			return Time(ts.tv_sec) * Time(1000000000) + Time(ts.tv_nsec);
		}
		Time _start_time;
	};
} // namespace PointMatcherSupport

#endif // __POINTMATCHER_TIMER_H

// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
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

#include "PointMatcher.h"
#include "PointMatcherPrivate.h"

//! Construct without parameter
template<typename T>
PointMatcher<T>::Inspector::Inspector()
{}

//! Construct with parameters
template<typename T>
PointMatcher<T>::Inspector::Inspector(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params):
	Parametrizable(className,paramsDoc,params)
{}

//! virtual destructor
template<typename T>
PointMatcher<T>::Inspector::~Inspector()
{}

//! Start a new ICP operation or sequence
template<typename T>
void PointMatcher<T>::Inspector::init()
{}

// performance statistics

//! Add a value for statistics name, create it if new
template<typename T>
void PointMatcher<T>::Inspector::addStat(const std::string& name, double data)
{}

//! Dump all statistics in CSV format
template<typename T>
void PointMatcher<T>::Inspector::dumpStats(std::ostream& stream)
{}

//! Dump header for all statistics
template<typename T>
void PointMatcher<T>::Inspector::dumpStatsHeader(std::ostream& stream)
{}

// data statistics 

//! Dump the state of a given iteration
template<typename T>
void PointMatcher<T>::Inspector::dumpIteration(const size_t iterationNumber, const TransformationParameters& parameters, const DataPoints& filteredReference, const DataPoints& reading, const Matches& matches, const OutlierWeights& outlierWeights, const TransformationCheckers& transformationCheckers)
{}

//! Tell the inspector the ICP operation is completed
template<typename T>
void PointMatcher<T>::Inspector::finish(const size_t iterationCount)
{}

template<typename T>
void PointMatcher<T>::Inspector::setAndValidateDumpPath(std::string path) {
	if (!path.empty() && path.back() != '/') {
		path += "/";
	}
	dumpPath = path;
}

template<typename T>
std::string PointMatcher<T>::Inspector::getDumpPath() const {
	return dumpPath;
}

template struct PointMatcher<float>::Inspector;
template struct PointMatcher<double>::Inspector;

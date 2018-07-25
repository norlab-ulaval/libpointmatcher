// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2018,
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
#include "RemoveSensorBias.h"

#include "PointMatcherPrivate.h"

#include <string>
#include <vector>

#include <boost/format.hpp>


// RemoveSensorBiasDataPointsFilter
// Constructor
template<typename T>
RemoveSensorBiasDataPointsFilter<T>::RemoveSensorBiasDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("RemoveSensorBiasDataPointsFilter", 
		RemoveSensorBiasDataPointsFilter::availableParameters(), params),
	sensorType(SensorType(Parametrizable::get<std::uint8_t>("sensorType")))
{
}

// RemoveSensorBiasDataPointsFilter
// Compute
template<typename T>
typename PointMatcher<T>::DataPoints 
RemoveSensorBiasDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void RemoveSensorBiasDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	//Check if there is normals info
	if (!cloud.descriptorExists("normals"))
		throw InvalidField("RemoveSensorBiasDataPointsFilter: Error, cannot find normals in descriptors.");
	//Check if there is normals info
	if (!cloud.descriptorExists("observationDirections"))
		throw InvalidField("RemoveSensorBiasDataPointsFilter: Error, cannot find observationDirections in descriptors.");
		
	const auto& normals = cloud.getDescriptorViewByName("normals");
	const auto& observationDirections = cloud.getDescriptorViewByName("observationDirections");

	switch(sensorType)
	{
		case LMS_1XX: 
		{
			/* do something */
			break;
		}
		case HDL_32E: 
		{
			/* do something */
			break;
		}
		default:
		throw InvalidParameter(
			(boost::format("RemoveSensorBiasDataPointsFilter: Error, cannot remove bias for sensorType id %1% .") % sensorType).str());
	}
}

template struct RemoveSensorBiasDataPointsFilter<float>;
template struct RemoveSensorBiasDataPointsFilter<double>;

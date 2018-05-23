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
#include "MaxPointCount.h"

// MaxPointCountDataPointsFilter
// Constructor
template<typename T>
MaxPointCountDataPointsFilter<T>::MaxPointCountDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("MaxPointCountDataPointsFilter", 
		MaxPointCountDataPointsFilter::availableParameters(), params),
	maxCount(Parametrizable::get<unsigned>("maxCount"))
{
	try 
	{
		seed = Parametrizable::get<unsigned>("seed");
	} 
	catch (const InvalidParameter& e) 
	{
		seed = static_cast<unsigned int> (1); // rand default seed number
	}
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints 
MaxPointCountDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void MaxPointCountDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	unsigned N = static_cast<unsigned> (cloud.features.cols());
	if (maxCount < N) 
	{
		DataPoints cloud_filtered = cloud.createSimilarEmpty(maxCount);
		std::srand(seed);

		unsigned top = N - maxCount;
		unsigned i = 0;
		unsigned index = 0;
		for (size_t n = maxCount; n >= 2; --n)
		{
			const float V = static_cast<float>(std::rand () / double (RAND_MAX));
			unsigned S = 0;
			float quot = static_cast<float> (top) / static_cast<float> (N);
			while (quot > V)
			{
				++S;
				--top;
				--N;
				quot = quot * static_cast<float> (top) / static_cast<float> (N);
			}
			index += S;
			cloud_filtered.setColFrom(i++, cloud, index++);
			--N;
		}
		//FIXME
		index += N * static_cast<unsigned> (static_cast<float>(std::rand() / double (RAND_MAX)));
		cloud_filtered.setColFrom(i++, cloud, index++);
		
		PointMatcher<T>::swapDataPoints(cloud, cloud_filtered);
		cloud.conservativeResize(i);
	}
}

template struct MaxPointCountDataPointsFilter<float>;
template struct MaxPointCountDataPointsFilter<double>;


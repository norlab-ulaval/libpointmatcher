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
#include <random>

// MaxPointCountDataPointsFilter
// Constructor
template<typename T>
MaxPointCountDataPointsFilter<T>::MaxPointCountDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("MaxPointCountDataPointsFilter",
									  MaxPointCountDataPointsFilter::availableParameters(), params),
	maxCount(Parametrizable::get<size_t>("maxCount"))
{
	try
	{
		seed = this->template get<size_t>("seed");
	}
	catch (const InvalidParameter&)
	{
		seed = static_cast<size_t>(1); // rand default seed number
	}
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints MaxPointCountDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void MaxPointCountDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	const size_t N{static_cast<size_t>(cloud.getNbPoints() - 1)};

	if (maxCount <= N)
	{
		std::minstd_rand randomNumberGenerator(static_cast<std::uint_fast32_t>(seed));
		std::uniform_real_distribution<float> distribution{0, 1};

		for (size_t j{0u}; j < maxCount; ++j)
		{
			//Get a random index in [j; N]
			const size_t index{j + static_cast<size_t>((N - j) * distribution(randomNumberGenerator))};

			//Switch columns j and index
			cloud.swapCols(j, index);
		}
		//Resize the cloud
		cloud.conservativeResize(maxCount);
	}
}

template struct MaxPointCountDataPointsFilter<float>;
template struct MaxPointCountDataPointsFilter<double>;

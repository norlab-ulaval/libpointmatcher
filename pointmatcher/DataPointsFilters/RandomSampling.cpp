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
#include "RandomSampling.h"

#include <random>

// RandomSamplingDataPointsFilter
// Constructor
template<typename T>
RandomSamplingDataPointsFilter<T>::RandomSamplingDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("RandomSamplingDataPointsFilter", RandomSamplingDataPointsFilter::availableParameters(), params),
	prob(Parametrizable::get<double>("prob")),
	randomSamplingMethod(Parametrizable::get<int>("randomSamplingMethod")),
	seed(Parametrizable::get<int>("seed"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints
RandomSamplingDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
Eigen::VectorXf RandomSamplingDataPointsFilter<T>::sampleRandomIndices(const size_t nbPoints)
{
	std::minstd_rand randomNumberGenerator;
	if (seed == -1)
	{
		std::random_device randomDevice;
		randomNumberGenerator = std::minstd_rand(randomDevice());
	}
	else
	{
		randomNumberGenerator = std::minstd_rand(seed);
	}

	switch(randomSamplingMethod)
	{
		default:	// Direct RNG.
		{
			const float randomNumberRange{static_cast<float>(randomNumberGenerator.max() - randomNumberGenerator.min())};
			return Eigen::VectorXf::NullaryExpr(nbPoints, [&](float){return static_cast<float>(randomNumberGenerator() / randomNumberRange);});
		}
		case 1:		// Uniform distribution.
		{
			std::uniform_real_distribution<float> distribution(0, 1);
			return Eigen::VectorXf::NullaryExpr(nbPoints, [&](float){return distribution(randomNumberGenerator);});
		}
	}
}

// In-place filter
template<typename T>
void RandomSamplingDataPointsFilter<T>::inPlaceFilter(
	DataPoints& cloud)
{
	const size_t nbPointsIn = cloud.features.cols();
	const size_t nbPointsOut = nbPointsIn * prob;

	const Eigen::VectorXf randomNumbers{sampleRandomIndices(nbPointsIn)};
	size_t j{0u};
	for (size_t i{0u}; i < nbPointsIn && j<=nbPointsOut; ++i)
	{
		if (randomNumbers(i) < prob)
		{
			cloud.setColFrom(j, cloud, i);
			++j;
		}
	}
	cloud.conservativeResize(j);
}

template struct RandomSamplingDataPointsFilter<float>;
template struct RandomSamplingDataPointsFilter<double>;



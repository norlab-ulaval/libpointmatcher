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
#include "OrientNormals.h"

#include <boost/optional.hpp>

// OrientNormalsDataPointsFilter
// Constructor
template<typename T>
OrientNormalsDataPointsFilter<T>::OrientNormalsDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("OrientNormalsDataPointsFilter", 
		OrientNormalsDataPointsFilter::availableParameters(), params),
	towardCenter(Parametrizable::get<bool>("towardCenter"))
{
}

// OrientNormalsDataPointsFilter
// Compute
template<typename T>
typename PointMatcher<T>::DataPoints 
OrientNormalsDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void OrientNormalsDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	if (!cloud.descriptorExists("normals"))
		throw InvalidField("OrientNormalsDataPointsFilter: Error, cannot find normals in descriptors.");
	if (!cloud.descriptorExists("observationDirections"))
		throw InvalidField("OrientNormalsDataPointsFilter: Error, cannot find observation directions in descriptors.");

	assert(cloud.getDescriptorDimension("normals") == cloud.getDescriptorDimension("observationDirections"));
	auto normals(cloud.getDescriptorViewByName("normals"));
	const auto observationDirections(cloud.getDescriptorViewByName("observationDirections"));
	const unsigned nbPoints(cloud.getNbPoints());

	boost::optional<View> eigenValues;
	boost::optional<View> eigenVectors;
	const bool orientEigenVectors(cloud.descriptorExists("eigVectors"));

	if (orientEigenVectors)
	{
		if (!cloud.descriptorExists("eigValues"))
			throw InvalidField("OrientNormalsDataPointsFilter: Error, eigVectors is present, but cannot find eigValues in descriptors.");

		eigenValues = cloud.getDescriptorViewByName("eigValues");
		eigenVectors = cloud.getDescriptorViewByName("eigVectors");
	}

	for (int i = 0; i < nbPoints; ++i)
	{
		// Check normal orientation
		const Vector observationDirection(observationDirections.col(i));
		const Vector normal(normals.col(i));
		const double scalar(observationDirection.dot(normal));

		// Swap normal
		if (towardCenter)
		{
			if (scalar < 0)
			{
				normals.col(i) = -normal;

				if (orientEigenVectors)
				{
					unsigned index;
					eigenValues->col(i).minCoeff(&index);
					unsigned descDim(cloud.getDescriptorDimension("eigValues"));
					eigenVectors->block(index * descDim, i, descDim, 1) *= -1;
				}
			}
		}
		else
		{
			if (scalar > 0)
			{
				normals.col(i) = -normal;

				if (orientEigenVectors)
				{
					unsigned index;
					eigenValues->col(i).minCoeff(&index);
					unsigned descDim(cloud.getDescriptorDimension("eigValues"));
					eigenVectors->block(index * descDim, i, descDim, 1) *= -1;
				}
			}
		}
	}
}

template struct OrientNormalsDataPointsFilter<float>;
template struct OrientNormalsDataPointsFilter<double>;


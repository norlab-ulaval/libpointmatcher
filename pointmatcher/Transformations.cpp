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

#include "Core.h"
#include <iostream>

// TransformFeatures
template<typename T>
typename PointMatcher<T>::DataPoints PointMatcher<T>::TransformFeatures::compute(
	const DataPoints& input,
	const TransformationParameters& parameters) const
{
	typedef typename DataPoints::Features Features;
	
	assert(input.features.rows() == parameters.rows());
	assert(parameters.rows() == parameters.cols());
	
	DataPoints transformedDataPoints(
		Features(input.features.rows(), input.features.cols()),
		input.featureLabels,
		input.descriptors,
		input.descriptorLabels
	);
	Features& transformedFeatures(transformedDataPoints.features);
	
	for (int i = 0; i < transformedFeatures.cols(); ++i)
	{
		transformedFeatures.col(i) = parameters * input.features.col(i);
	}
	
	return transformedDataPoints;
}

template struct PointMatcher<float>::TransformFeatures;
template struct PointMatcher<double>::TransformFeatures;


// TransformNormals
template<typename T>
typename PointMatcher<T>::DataPoints PointMatcher<T>::TransformNormals::compute(
	const DataPoints& input,
	const TransformationParameters& parameters) const
{
	typedef typename DataPoints::Descriptors Descriptors;
	assert(parameters.rows() == parameters.cols());
	
	DataPoints transformedDataPoints(input);
	
	if (!input.descriptorLabels.contains("normals"))
		return transformedDataPoints;

	const unsigned ptCount(input.descriptors.cols());

	//NOTE: Only need rotation for descriptors (up to now...)
	const Matrix R(parameters.topLeftCorner(parameters.rows()-1, parameters.cols()-1));

	unsigned descRow(0);
	for(unsigned i = 0; i < transformedDataPoints.descriptorLabels.size(); i++)
	{
		const unsigned span(input.descriptorLabels[i].span);
		if(transformedDataPoints.descriptorLabels[i].text.compare("normals") == 0)
		{
			assert(span == parameters.rows()-1);
			transformedDataPoints.descriptors.block(descRow, 0, span, ptCount) = 
				R * input.descriptors.block(descRow, 0, span, ptCount);
		}
		descRow += input.descriptorLabels[i].span;
	}
	
	return transformedDataPoints;
}

template struct PointMatcher<float>::TransformNormals;
template struct PointMatcher<double>::TransformNormals;


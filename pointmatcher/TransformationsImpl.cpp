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

#include "TransformationsImpl.h"

#include <iostream>

// TransformFeatures
template<typename T>
typename PointMatcher<T>::DataPoints TransformationsImpl<T>::TransformFeatures::compute(
	const DataPoints& input,
	const TransformationParameters& parameters) const
{
	assert(input.features.rows() == parameters.rows());
	assert(parameters.rows() == parameters.cols());
	
	DataPoints transformedDataPoints = input;
		
	// Apply the transformation
	transformedDataPoints.features = parameters * input.features;

	return transformedDataPoints;
}

template struct TransformationsImpl<float>::TransformFeatures;
template struct TransformationsImpl<double>::TransformFeatures;


// TransformNormals
template<typename T>
typename PointMatcher<T>::DataPoints TransformationsImpl<T>::TransformNormals::compute(
	const DataPoints& input,
	const TransformationParameters& parameters) const
{
	typedef typename DataPoints::Descriptors Descriptors;
	typedef typename PointMatcher<T>::Matrix Matrix;
	
	assert(parameters.rows() == parameters.cols());
	
	DataPoints transformedDataPoints(input);
	
	if (!input.isDescriptorExist("normals"))
		return transformedDataPoints;

	const Matrix R(parameters.topLeftCorner(parameters.rows()-1, parameters.cols()-1));

	const Descriptors normals = input.getDescriptorByName("normals");
	transformedDataPoints.addDescriptor("normals", R*normals);
	
	return transformedDataPoints;
}

template struct TransformationsImpl<float>::TransformNormals;
template struct TransformationsImpl<double>::TransformNormals;


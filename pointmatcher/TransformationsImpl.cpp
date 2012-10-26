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

#include "TransformationsImpl.h"

#include <iostream>

// RigidTransformation
template<typename T>
typename PointMatcher<T>::DataPoints TransformationsImpl<T>::RigidTransformation::compute(
	const DataPoints& input,
	const TransformationParameters& parameters) const
{
	typedef typename PointMatcher<T>::Matrix Matrix;
	
	assert(input.features.rows() == parameters.rows());
	assert(parameters.rows() == parameters.cols());
	
	DataPoints transformedCloud(input.featureLabels, input.descriptorLabels, input.features.cols());
	
	// Apply the transformation to features
	transformedCloud.features = parameters * input.features;
	
	// Apply the transformation to descriptors
	const Matrix R(parameters.topLeftCorner(parameters.rows()-1, parameters.cols()-1));
	int row(0);
	const int descCols(input.descriptors.cols());
	for (size_t i = 0; i < input.descriptorLabels.size(); ++i)
	{
		const int span(input.descriptorLabels[i].span);
		const std::string& name(input.descriptorLabels[i].text);
		const auto inputDesc(input.descriptors.block(row, 0, span, descCols));
		auto outputDesc(transformedCloud.descriptors.block(row, 0, span, descCols));
		if (name == "normals" || name == "observationDirections")
			outputDesc = R * inputDesc;
		else
			outputDesc = inputDesc;
		row += span;
	}
	
	return transformedCloud;
}

template struct TransformationsImpl<float>::RigidTransformation;
template struct TransformationsImpl<double>::RigidTransformation;


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
#include "Functions.h"

#include <iostream>

using namespace PointMatcherSupport;

//! Construct a transformation exception
TransformationError::TransformationError(const std::string& reason):
	runtime_error(reason)
{}

//! RigidTransformation
template<typename T>
typename PointMatcher<T>::DataPoints TransformationsImpl<T>::RigidTransformation::compute(
	const DataPoints& input,
	const TransformationParameters& parameters) const
{
	//typedef typename PointMatcher<T>::Matrix Matrix;
	
	assert(input.features.rows() == parameters.rows());
	assert(parameters.rows() == parameters.cols());

	const TransformationParameters R(parameters.topLeftCorner(parameters.rows()-1, parameters.cols()-1));

	if(this->checkParameters(parameters) == false)	
		throw TransformationError("RigidTransformation: Error, rotation matrix is not orthogonal.");	
	
	DataPoints transformedCloud(input.featureLabels, input.descriptorLabels, input.features.cols());
	
	// Apply the transformation to features
	transformedCloud.features = parameters * input.features;
	
	// Apply the transformation to descriptors
	int row(0);
	const int descCols(input.descriptors.cols());
	for (size_t i = 0; i < input.descriptorLabels.size(); ++i)
	{
		const int span(input.descriptorLabels[i].span);
		const std::string& name(input.descriptorLabels[i].text);
		const BOOST_AUTO(inputDesc, input.descriptors.block(row, 0, span, descCols));
		BOOST_AUTO(outputDesc, transformedCloud.descriptors.block(row, 0, span, descCols));
		if (name == "normals" || name == "observationDirections")
			outputDesc = R * inputDesc;
		else
			outputDesc = inputDesc;
		row += span;
	}
	
	return transformedCloud;
}

//! Ensure orthogonality of the rotation matrix
template<typename T>
bool TransformationsImpl<T>::RigidTransformation::checkParameters(const TransformationParameters& parameters) const
{
	//FIXME: FP - should we put that as function argument?
	const T epsilon = 0.001;

	const TransformationParameters R(parameters.topLeftCorner(parameters.rows()-1, parameters.cols()-1));
	
	if(anyabs(1 - R.determinant()) > epsilon)
		return false;
	else
		return true;
}

//! Force orthogonality of the rotation matrix
template<typename T>
typename PointMatcher<T>::TransformationParameters TransformationsImpl<T>::RigidTransformation::correctParameters(const TransformationParameters& parameters) const
{
	TransformationParameters ortho = parameters;
	if(ortho.cols() == 4)
	{
		const Eigen::Matrix<T, 3, 1> col0 = parameters.block(0, 0, 3, 1).normalized();
		const Eigen::Matrix<T, 3, 1> col1 = parameters.block(0, 1, 3, 1).normalized();
		const Eigen::Matrix<T, 3, 1> col2 = parameters.block(0, 2, 3, 1).normalized();


		ortho.block(0, 0, 3, 1) = col1.cross(col2);
		ortho.block(0, 1, 3, 1) = col2.cross(col0);
		ortho.block(0, 2, 3, 1) = col2;
	}
	else if(ortho.cols() == 3)
	{
		// R = [ a b]
		//     [-b a]
		
		// mean of a and b
		T a = (parameters(0,0) + parameters(1,1))/2; 	
		T b = (-parameters(1,0) + parameters(0,1))/2;
		T sum = sqrt(pow(a,2) + pow(b,2));

		a = a/sum;
		b = b/sum;

		ortho(0,0) =  a; ortho(0,1) = b;
		ortho(1,0) = -b; ortho(1,1) = a;
	}


	return ortho;
}

template struct TransformationsImpl<float>::RigidTransformation;
template struct TransformationsImpl<double>::RigidTransformation;


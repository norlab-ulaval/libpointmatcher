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
#pragma once

#include "PointMatcher.h"

/*
 The SphericalityDataPointFilter estimates a heuristic value which indicates how much local geometry around
 a given point resemble a plane or a sphere (uniform distribution). The value can lie in <-1,1> interval, -1 for
 perfect plane, 1 for perfectly uniform distribution. The estimation is based on three eigenvalues coming from another
 data points filter (this filter can operate only on 3D data). In the case the largest eigenvalue is zero, the filter
 outputs NaNs. If the middle eigenvalue is zero and the largest one is non-zero, the filter outputs zeros.

 Implemented by Vladimir Kubelka (kubelvla@gmail.com), NORLAB, Universite Laval, 2020
*/

template<typename T>
struct SphericalityDataPointsFilter: public PointMatcher<T>::DataPointsFilter
{
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	typedef Parametrizable::InvalidParameter InvalidParameter;
	
	typedef typename PointMatcher<T>::Vector Vector;
	typedef typename PointMatcher<T>::Matrix Matrix;	
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;

	inline static const std::string description()
	{
		return "This filter computes the level of ’sphericality’ for each point. It describes the shape of local geometry, whether the surrounding points resemble a plane (the sphericality value goes towards -1) or a uniform distribution (the value towards +1). It is intended for 3D point clouds only. Sphericality is computed by subtracting the intermediate values of ‘unstructureness’ and ‘structureness’.\n\n"
		       "Required descriptors: eigValues (must be three, otherwise exception).\n"
		       "Produced descritors:  sphericality, unstructureness(optional), structureness(optional).\n"
			   "Altered descriptors:  none.\n"
			   "Altered features:     none.";
	}
	inline static const ParametersDoc availableParameters()
	{
		return {
			{"keepUnstructureness", "whether the value of the unstructureness should be added to the pointcloud", "0"},
			{"keepStructureness", "whether the value of the structureness should be added to the pointcloud", "0"},
			};
	}
	
	const bool keepUnstructureness;
	const bool keepStructureness;

    SphericalityDataPointsFilter(const Parameters& params = Parameters());
	virtual ~SphericalityDataPointsFilter() {};
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};

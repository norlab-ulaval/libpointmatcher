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
#include "utils/sparsetv.h"

/**
 * Spectral Decomposition Filter (SpDF) is a sampling algorithm based on spectral decomposition analysis to derive local density measures for each geometric primitive.
 * First, we identify the geometric primitives along with their saliencies using the tensor voting framework. 
 * Then, we derive density measures from saliencies: if the density for each geometric primitive is less than the desired density, we stop; else we sub-sample each over-represented geometric primitive, and re-iterate.
 * As output, we have a uniform sampled point cloud enhanced with geometric information.
 *
 * Implemented by Mathieu Labussiere <mathieu dot labu at gmail dot com>, Institut Pascal, Université Clermont Auvergne, 2020
 **/
template<typename T>
struct SpectralDecompositionDataPointsFilter : public PointMatcher<T>::DataPointsFilter
{
  	// Type definitions
	typedef PointMatcher<T> PM;
	typedef typename PM::DataPoints DataPoints;
	typedef typename PM::DataPoints DP;
	typedef typename PM::DataPointsFilter DataPointsFilter;

	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	typedef Parametrizable::InvalidParameter InvalidParameter;
	
	typedef typename DataPoints::Index Index;

	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;
	
	typedef typename PM::Matrix Matrix;
	typedef typename PM::Vector Vector;

	inline static const std::string description()
	{
		return "Point cloud sampling and enhancement: compute geometric features saliencies throught Tensor Voting framework and use them to sample the point cloud. \\cite{Labussiere2020}";
	}

	inline static const ParametersDoc availableParameters()
	{
		return {
			{"k", "Number of neighbors to consider", "50", "6", "4294967295", &P::Comp<std::size_t>},
			{"sigma", "Scale of the vote in TensorVoting.", "0.2", "0.", "+inf", &P::Comp<T>},
			{"radius", "Radius to control the scale of the uniform distribution.", "0.4", "0.", "+inf", &P::Comp<T>},
			{"itMax", "Number max of iterations to do", "10", "1", "4294967295", &P::Comp<std::size_t>},
			{"keepNormals", "Flag to keep normals computed by TV.", "1", "0", "1", P::Comp<bool>},
			{"keepLabels", "Flag to keep labels computed by TV.", "1", "0", "1", P::Comp<bool>},
			{"keepLambdas", "Flag to keep lambdas computed by TV.", "1", "0", "1", P::Comp<bool>},
			{"keepTensors", "Flag to keep elements Tensors computed by TV.", "1", "0", "1", P::Comp<bool>}
		};
	}

public:
	const std::size_t k;
	const T sigma;
	const T radius;
	const std::size_t itMax;
	const bool keepNormals;
	const bool keepLabels;
	const bool keepLambdas;
	const bool keepTensors;
	
	//Ctor, uses parameter interface
	SpectralDecompositionDataPointsFilter(const Parameters& params = Parameters());
	//SpectralDecompositionDataPointsFilter();
	
	//Dtor
	virtual ~SpectralDecompositionDataPointsFilter() {};

	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);

private:
	static T xi_expectation(const std::size_t D, const T sigma_, const T radius_)
	{
		return (D == 1) ? //on a curve
			(std::sqrt(M_PI * sigma_) *  std::erf(radius_ / std::sqrt(sigma_))) / (2. * radius_)
		: (D == 2) ? //on a surface
			(sigma_ - sigma_ * std::exp(- radius_ * radius_ / sigma_)) / (radius_ * radius_)
		: (D == 3) ?//on a sphere
			3. * sigma_ * (std::sqrt(M_PI * sigma_) *  std::erf(radius_ / std::sqrt(sigma_)) - 2. * radius_ * std::exp(- radius_ * radius_ / sigma_)) / (4. * radius_ * radius_ * radius_)
		: T(1.); //otherwise
	}
	
	void addDescriptor(DataPoints& pts, const TensorVoting<T> &tv, bool keepNormals_, bool keepLabels_, bool keepLambdas_, bool keepTensors_) const;
	
	void removeOutlier(DataPoints& pts, const TensorVoting<T> &tv) const;
	
	void filterSurfaceness(DataPoints& pts, T xi, std::size_t k) const;
	void filterCurveness(DataPoints& pts, T xi, std::size_t k) const;
	void filterPointness(DataPoints& pts, T xi, std::size_t k) const;
};
	


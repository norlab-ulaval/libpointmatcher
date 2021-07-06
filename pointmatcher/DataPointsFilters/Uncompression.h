#pragma once

#include "PointMatcher.h"

template<typename T>
struct UncompressionDataPointsFilter : public PointMatcher<T>::DataPointsFilter
{
	typedef PointMatcher<T> PM;
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParametersDoc ParametersDoc;

	inline static const std::string description()
	{
		return "Lossy point cloud uncompression using descriptive statistics."
			   "Required descriptors: covariance, nbPoints.\n"
			   "Produced descritors:  none.\n"
			   "Altered descriptors:  all.\n"
			   "Altered features:     points coordinates and number of points.";
	}

	inline static const ParametersDoc availableParameters()
	{
		return {
				{"seed", "srand seed", "1", "0", "2147483647", &P::Comp < size_t > }
		};
	}

	size_t seed;

	UncompressionDataPointsFilter(const Parameters& params = Parameters());
	virtual typename PM::DataPoints filter(const typename PM::DataPoints& input);
	virtual void inPlaceFilter(typename PM::DataPoints& cloud);
};

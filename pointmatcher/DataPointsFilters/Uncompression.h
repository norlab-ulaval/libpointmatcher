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

	typedef typename PM::DataPoints DataPoints;
	typedef typename PM::DataPoints::InvalidField InvalidField;
	typedef typename PM::Matrix Matrix;
	typedef typename PM::Vector Vector;

	inline static const std::string description()
	{
		return "Lossy point cloud uncompression using descriptive statistics."
			   "Required descriptors: mean, covariance, nbPoints.\n"
			   "Produced descritors:  none.\n"
			   "Altered descriptors:  all.\n"
			   "Altered features:     points coordinates and number of points.";
	}

	inline static const ParametersDoc availableParameters()
	{
		return {
				{"seed",       "srand seed",                                                          "1",    "0",    "2147483647", &P::Comp < size_t > },
				{"maxDensity", "Maximum density of points to target. Unit: number of points per m³.", "1000", "1000", "inf",        &P::Comp < T > }
		};
	}

	const T maxDensity;
	size_t seed;

	UncompressionDataPointsFilter(const Parameters& params = Parameters());
	virtual typename PM::DataPoints filter(const typename PM::DataPoints& input);
	virtual void inPlaceFilter(typename PM::DataPoints& cloud);
};

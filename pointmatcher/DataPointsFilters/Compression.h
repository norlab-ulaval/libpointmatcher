#pragma once

#include "PointMatcher.h"

template<typename T>
struct CompressionDataPointsFilter : public PointMatcher<T>::DataPointsFilter
{
	typedef PointMatcher<T> PM;
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParametersDoc ParametersDoc;

	inline static const std::string description()
	{
		return "Lossy point cloud compression using descriptive statistics."
			   "Required descriptors: none.\n"
			   "Produced descritors:  eigVectors, weightSum.\n"
			   "Altered descriptors:  all.\n"
			   "Altered features:     points coordinates and number of points.";
	}

	inline static const ParametersDoc availableParameters()
	{
		return {
				{"knn",          "number of nearest neighbors to consider in the reference",                "1",                         "1",                         "2147483647",
						&P::Comp < unsigned > },
				{"maxDist",      "maximum distance to consider for neighbors",                              "inf",                       "0",                         "inf",
						&P::Comp < T > },
				{"epsilon",      "Step of discretization for the angle spaces",                             "0.09817477042" /* PI/32 */, "0.04908738521" /* PI/64 */, "3.14159265359" /* PI */,
						&P::Comp < T > },
				{"maxDeviation", "Maximum distance from the mean for a point to represent a distribution.", "0.3",                       "0.0",                       "inf",
						&P::Comp < T > }
		};
	}

	const int knn;
	const T maxDist;
	const T epsilon;
	const T maxDeviation;

	CompressionDataPointsFilter(const Parameters& params = Parameters());
	virtual typename PM::DataPoints filter(const typename PM::DataPoints& input);
	virtual void inPlaceFilter(typename PM::DataPoints& cloud);
};

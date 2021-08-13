#pragma once

#include "PointMatcher.h"

template<typename T>
struct CompressionDataPointsFilter : public PointMatcher<T>::DataPointsFilter
{
	typedef PointMatcher<T> PM;
	typedef typename PM::Matrix Matrix;
	typedef typename PM::Vector Vector;
	typedef typename PM::DataPoints DataPoints;
	typedef typename PM::DataPoints::Label Label;
	typedef typename PM::DataPoints::Labels Labels;
	typedef typename PM::DataPoints::InvalidField InvalidField;
	typedef typename PM::DataPoints::View View;

	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParametersDoc ParametersDoc;

	inline static const std::string description()
	{
		return "Lossy point cloud compression using descriptive statistics."
			   "Required descriptors: none.\n"
			   "Produced descritors:  initialPosition, omega, weightSum, nbPoints, normals(optional), eigValues(optional), eigVectors(optional).\n"
			   "Altered descriptors:  all.\n"
			   "Altered features:     points coordinates and number of points.";
	}

	inline static const ParametersDoc availableParameters()
	{
		return {
				{"knn",               "number of nearest neighbors to consider in the reference",                                                                 "10",            "1",    "2147483647",
						&P::Comp < unsigned > },
				{"maxDist",           "maximum distance to consider for neighbors",                                                                               "inf",           "0",    "inf",
						&P::Comp < T > },
				{"epsilon",           "Step of discretization for the angle spaces",                                                                              "0.09817477042", "0",    "3.14159265359",
						&P::Comp < T > },
				{"maxIterationCount", "Maximum number of iterations",                                                                                             "5",             "0",    "2147483647",
						&P::Comp < unsigned > },
				{"initialVariance",   "Variance on individual point positions (isotropic)",                                                                       "9e-4",          "1e-6", "inf",
						&P::Comp < T > },
				{"maxDeviation",      "Maximum distance from the mean for a point to represent a distribution.",                                                  "0.3",           "0.0",  "inf",
						&P::Comp < T > },
				{"maxVolumeRatio",    "Maximum ratio between the volume of a newly created distribution and the sum of volumes of the distribution it contains.", "2.0",           "0.0",  "inf",
						&P::Comp < T > },
				{"keepNormals",       "whether the normals should be added as descriptors to the resulting cloud",                                                "0"},
				{"keepEigenValues",   "whether the eigen values should be added as descriptors to the resulting cloud",                                           "0"},
				{"keepEigenVectors",  "whether the eigen vectors should be added as descriptors to the resulting cloud",                                          "0"},
				{"sortEigen",         "whether the eigenvalues and eigenvectors should be sorted (ascending) based on the eigenvalues",                           "0"}
		};
	}

	const unsigned knn;
	const T maxDist;
	const T epsilon;
	const unsigned maxIterationCount;
	const T initialVariance;
	const T maxDeviation;
	const T maxVolumeRatio;
	const bool keepNormals;
	const bool keepEigenValues;
	const bool keepEigenVectors;
	const bool sortEigen;

	CompressionDataPointsFilter(const Parameters& params = Parameters());
	virtual typename PM::DataPoints filter(const typename PM::DataPoints& input);
	virtual void inPlaceFilter(typename PM::DataPoints& cloud);
};

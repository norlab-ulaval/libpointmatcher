//
// Created by Matěj Boxan on 2023-10-09.
//

#ifndef LIBPOINTMATCHER_SYMMETRY_H
#define LIBPOINTMATCHER_SYMMETRY_H

#pragma once

#include <utility>

#include "Eigen/Eigenvalues"
#include "PointMatcher.h"
#include "vector"
#include "utils/distribution.h"

// TODO add optional produced descriptors: normals, eigenvalues, eigenvectors
template<typename T>
struct SymmetryDataPointsFilter : public PointMatcher<T>::DataPointsFilter
{
    typedef PointMatcher<T> PM;
    typedef PointMatcherSupport::Parametrizable Parametrizable;
    typedef PointMatcherSupport::Parametrizable P;
    typedef Parametrizable::Parameters Parameters;
    typedef Parametrizable::ParameterDoc ParameterDoc;
    typedef Parametrizable::ParametersDoc ParametersDoc;
    typedef Parametrizable::InvalidParameter InvalidParameter;

	typedef typename PointMatcher<T>::Vector Vector;
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	typedef typename PointMatcher<T>::Matrix Matrix;

	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;

	inline static const std::string description()
	{
        return "Lossy point cloud compression using incremental statistics.\n"
               "Required descriptors: none.\n"
               "Produced descriptors:  omega, deviation.\n"
               "Altered descriptors:  all.\n"
               "Altered features:     points coordinates and number of points.";
	}
	inline static const ParametersDoc availableParameters()
	{
		return {
			{"vrs", "volume ratio for symmetry sampling", "5", "0", "inf", &P::Comp<T>},
			{"vro", "volume ratio for overlap sampling", "1.025", "0", "inf", &P::Comp<T>},
			{"dr", "distance ration for symmetry sampling", "0.1", "0", "inf", &P::Comp<T>},
			{"ct", "compressions tolerance [%]", "0.95", "0", "1", &P::Comp<T>},
			{"knn", "number of nearest neighbors to consider, including the point itself", "5", "3", "2147483647", &P::Comp<unsigned>},
			{"initialVariance", "Variance on individual point positions (isotropic)", "0.0009", "0", "inf", &P::Comp<T>},
			};
	}

	const T vrs;
	const T vro;
	const T dr;
	const T ct;
    const T initialVariance;
	const unsigned knn;

	//! Constructor, uses parameter interface
	explicit SymmetryDataPointsFilter(const Parameters& params = Parameters());
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
    void symmetrySampling(std::vector<std::shared_ptr<Distribution<T>>>& distributions);
    void overlapSampling(std::vector<std::shared_ptr<Distribution<T>>>& distributions);
    std::vector<std::shared_ptr<Distribution<T>>> getDistributionsFromCloud(DataPoints& cloud);
    DataPoints getCloudFromDistributions(const DataPoints& in_cloud, std::vector<std::shared_ptr<Distribution<T>>>& distributions);
    DataPoints getPointsFromDistributions(std::vector<std::shared_ptr<Distribution<T>>>& distributions);
    void mergeTimesDescriptors(std::vector<std::shared_ptr<Distribution<T>>>& distributions, const std::vector<unsigned>& indexesToMerge);
};

#endif //LIBPOINTMATCHER_SYMMETRY_H

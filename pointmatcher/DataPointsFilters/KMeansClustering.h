// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab

#pragma once

#include "PointMatcher.h"

//! Add new descriptor to an existing point cloud
template<typename T>
struct KMeansClusteringDataPointsFilter : public PointMatcher<T>::DataPointsFilter
{
    // Type definitions
    typedef PointMatcher<T> PM;
    typedef typename PM::DataPoints DataPoints;
    typedef typename PM::DataPointsFilter DataPointsFilter;

    typedef PointMatcherSupport::Parametrizable Parametrizable;
    typedef PointMatcherSupport::Parametrizable P;
    typedef Parametrizable::Parameters Parameters;
    typedef Parametrizable::ParameterDoc ParameterDoc;
    typedef Parametrizable::ParametersDoc ParametersDoc;
    typedef Parametrizable::InvalidParameter InvalidParameter;

    typedef typename PointMatcher<T>::Matrix Matrix;
    typedef typename PointMatcher<T>::Vector Vector;
    typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;

    const std::size_t k;
    const std::size_t iter;
    const T epsilon;
    const int seed;

    inline static const std::string description()
    {
        return "Adds a new descriptor to an existing point cloud or overwrites existing descriptor with the same name.\n\n"
			   "Required descriptors: none.\n"
		       "Produced descriptors:  User defined.\n"
			   "Altered descriptors:  none.\n"
			   "Altered features:     none.";
    }

    inline static const ParametersDoc availableParameters()
    {
        return {
                {"k",      "Number of clusters.", "2", "2", "4294967295", &P::Comp < std::size_t > },
                {"iter", "Maximum number of iterations.", "1",            "1", "4294967295", &P::Comp < std::size_t > },
                {"epsilon", "Tolerance value, the clustering is stopped if the centroids moved less than epsilon since the previous iteration.", "-1", "-1", "inf", &P::Comp<T>},
			    {"seed", "Seed for random sampling (-1 means no seed is used)", "-1", "-1", "2147483647", &P::Comp<int>}
        };
    }

	//Constructor, uses parameter interface
	explicit KMeansClusteringDataPointsFilter(const Parameters& params = Parameters());

    virtual DataPoints filter(const DataPoints& input);
    virtual void inPlaceFilter(DataPoints& cloud);

private:
    DataPoints getSeedPoints(const DataPoints& cloud);
    void assignClusters(const DataPoints& cloud, const DataPoints& seeds, Vector& cluster_index);
};

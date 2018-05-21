#pragma once

#include "PointMatcher.h"

#include <vector>

//! Surface normals estimation. Find the normal for every point using eigen-decomposition of neighbour points
template<typename T>
struct SurfaceNormalDataPointsFilter: public PointMatcher<T>::DataPointsFilter
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
		return "This filter extracts the surface normal vector and other statistics to each point by taking the eigenvector corresponding to the smallest eigenvalue of its nearest neighbors.\n\n"
		       "Required descriptors: none.\n"
		       "Produced descritors:  normals(optional), densities(optional), eigValues(optional), eigVectors(optional), matchedIds (optional), meanDists(optional).\n"
			   "Altered descriptors:  none.\n"
			   "Altered features:     none.";
	}
	inline static const ParametersDoc availableParameters()
	{
		return boost::assign::list_of<ParameterDoc>
			( "knn", "number of nearest neighbors to consider, including the point itself", "5", "3", "2147483647", &P::Comp<unsigned> )
			( "maxDist", "maximum distance to consider for neighbors", "inf", "0", "inf", &P::Comp<T> )
			( "epsilon", "approximation to use for the nearest-neighbor search", "0", "0", "inf", &P::Comp<T> )
			( "keepNormals", "whether the normals should be added as descriptors to the resulting cloud", "1" )
			( "keepDensities", "whether the point densities should be added as descriptors to the resulting cloud", "0" )
			( "keepEigenValues", "whether the eigen values should be added as descriptors to the resulting cloud", "0" )
			( "keepEigenVectors", "whether the eigen vectors should be added as descriptors to the resulting cloud", "0" )
			( "keepMatchedIds" , "whether the identifiers of matches points should be added as descriptors to the resulting cloud", "0" )
			( "keepMeanDist" , "whether the distance to the nearest neighbor mean should be added as descriptors to the resulting cloud", "0" )
			( "sortEigen" , "whether the eigenvalues and eigenvectors should be sorted (ascending) based on the eigenvalues", "0" )
			( "smoothNormals", "whether the normal vector should be average with the nearest neighbors", "0" )
		;
	}
	
	const unsigned knn;
	const T maxDist;
	const T epsilon;
	const bool keepNormals;
	const bool keepDensities;
	const bool keepEigenValues;
	const bool keepEigenVectors;
	const bool keepMatchedIds;
	const bool keepMeanDist;
	const bool sortEigen;
	const bool smoothNormals;

	SurfaceNormalDataPointsFilter(const Parameters& params = Parameters());
	virtual ~SurfaceNormalDataPointsFilter() {};
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};

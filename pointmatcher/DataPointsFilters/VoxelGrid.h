#pragma once

#include "PointMatcher.h"

template<typename T>
struct VoxelGridDataPointsFilter : public PointMatcher<T>::DataPointsFilter
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
	typedef typename Eigen::Matrix<T,2,1> Vector2;
	typedef typename Eigen::Matrix<T,3,1> Vector3;
	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;


	inline static const std::string description()
	{
		return "Construct Voxel grid of the point cloud. Down-sample by taking centroid or center of grid cells.";
	}

	inline static const ParametersDoc availableParameters()
	{
		return boost::assign::list_of<ParameterDoc>
		( "vSizeX", "Dimension of each voxel cell in x direction", "1.0", "0.001", "+inf", &P::Comp<T> )
		( "vSizeY", "Dimension of each voxel cell in y direction", "1.0", "0.001", "+inf", &P::Comp<T> )
		( "vSizeZ", "Dimension of each voxel cell in z direction", "1.0", "0.001", "+inf", &P::Comp<T> )
		( "useCentroid", "If 1 (true), down-sample by using centroid of voxel cell.  If false (0), use center of voxel cell.", "1", "0", "1", P::Comp<bool> )
		( "averageExistingDescriptors", "whether the filter keep the existing point descriptors and average them or should it drop them", "1", "0", "1", P::Comp<bool> )
		;
	}

	const T vSizeX;
	const T vSizeY;
	const T vSizeZ;
	const bool useCentroid;
	const bool averageExistingDescriptors;

	struct Voxel {
		unsigned int    numPoints;
		unsigned int    firstPoint;
		Voxel() : numPoints(0), firstPoint(0) {}
	};

	//Constructor, uses parameter interface
	VoxelGridDataPointsFilter(const Parameters& params = Parameters());

	VoxelGridDataPointsFilter();
  // Destr
	virtual ~VoxelGridDataPointsFilter() {};

	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};

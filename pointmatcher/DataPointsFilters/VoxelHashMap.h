#pragma once

#include "PointMatcher.h"
#include <unordered_map>

// got this hashing of eigen vector from kiss-icp : https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/VoxelUtils.hpp
using Voxel = Eigen::Vector3i;
template <>
struct std::hash<Voxel> {
    std::size_t operator()(const Voxel& voxel) const {
        const uint32_t* vec = reinterpret_cast<const uint32_t*>(voxel.data());
        return (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
    }
};

template <typename T>
struct VoxelHashMapDataPointsFilter : public PointMatcher<T>::DataPointsFilter
{

  	// Type definitions
	typedef PointMatcher<T> PM;
	typedef typename PM::DataPoints DataPoints;
	typedef typename PM::DataPointsFilter DataPointsFilter;
	typedef typename DataPoints::Index Index;

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
       return "Construct Voxel hash map of the point cloud. The first \"pointsPerVoxel\" points to be inserted are kept. Once the voxel is full, further points are discarded.";
   }
   inline static const ParametersDoc availableParameters()
   {

       return {
			{ "voxelSize", "The size of the voxel.", "1.0", "0.0", "inf", &P::Comp<T> },
			{ "pointsPerVoxel", "The amount of points per voxel.", "3", "1", "9999999", &P::Comp<size_t> }
       };
   }

    inline Voxel PointToVoxel(const Eigen::Vector3<T>& point, const double voxel_size)
	{
    	return Voxel(static_cast<int>(std::floor(point.x() / voxel_size)),
					 static_cast<int>(std::floor(point.y() / voxel_size)),
                     static_cast<int>(std::floor(point.z() / voxel_size)));
	}

   VoxelHashMapDataPointsFilter(const Parameters& params = Parameters());
   virtual ~VoxelHashMapDataPointsFilter() {};
   virtual DataPoints filter(const DataPoints& input);
   virtual void inPlaceFilter(DataPoints& cloud);


   const T voxelSize;
   const size_t pointsPerVoxel;
};
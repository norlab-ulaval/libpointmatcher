// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
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

#ifndef __POINTMATCHER_DATAPOINTSFILTERS_H
#define __POINTMATCHER_DATAPOINTSFILTERS_H

#include "PointMatcher.h"

template<typename T>
struct DataPointsFiltersImpl
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
	typedef typename PointMatcher<T>::DataPointsFilter DataPointsFilter;
	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;
	
	//! Identity, does nothing
	struct IdentityDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Does nothing.";
		}
		
		//inline static const ParametersDoc availableParameters()
		//{
		//	return ParametersDoc({
		//		( "param1", "Description of the parameter", "defaultValue", "minValue", "maxValue", type of the parameter )
		//		) "param2", "Description of the parameter", "defaultValue", "minValue", "maxValue", type of the parameter )
		//	;
		//}
		//! Constructor, uses parameter interface
		//IdentityDataPointsFilter(const Parameters& params = Parameters());
		
		virtual DataPoints filter(const DataPoints& input);
		virtual void inPlaceFilter(DataPoints& cloud);
	};
	
	
	//! Remove points having NaN as coordinate
	struct RemoveNaNDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Remove points having NaN as coordinate.";
		}
		
		virtual DataPoints filter(const DataPoints& input);
		virtual void inPlaceFilter(DataPoints& cloud);
	};
	
	//! Subsampling. Filter points beyond a maximum distance measured on a specific axis
	struct MaxDistDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Subsampling. Filter points beyond a maximum distance measured on a specific axis. If dim is set to -1, points are filtered based on a maximum radius.";
		}
		inline static const ParametersDoc availableParameters()
		{
			return boost::assign::list_of<ParameterDoc>
				( "dim", "dimension on which the filter will be applied. x=0, y=1, z=2, radius=-1", "-1", "-1", "2", &P::Comp<int> )
				( "maxDist", "maximum distance authorized. If dim is set to -1 (radius), the absolute value of minDist will be used. All points beyond that will be filtered.", "1", "-inf", "inf", &P::Comp<T> )
			;
		}

		const int dim;
		const T maxDist;
		
		//! Constructor, uses parameter interface
		MaxDistDataPointsFilter(const Parameters& params = Parameters());
		virtual DataPoints filter(const DataPoints& input);
		virtual void inPlaceFilter(DataPoints& cloud);
	};

	//! Subsampling. Filter points before a minimum distance measured on a specific axis
	struct MinDistDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Subsampling. Filter points before a minimum distance measured on a specific axis. If dim is set to -1, points are filtered based on a minimum radius.";
		}
		inline static const ParametersDoc availableParameters()
		{
			return boost::assign::list_of<ParameterDoc>
				( "dim", "dimension on which the filter will be applied. x=0, y=1, z=2, radius=-1", "-1", "-1", "2", &P::Comp<int> )
				( "minDist", "minimum value authorized. If dim is set to -1 (radius), the absolute value of minDist will be used. All points before that will be filtered.", "1", "-inf", "inf", &P::Comp<T> )
			;
		}
		
		const int dim;
		const T minDist;

		//! Constructor, uses parameter interface
		MinDistDataPointsFilter(const Parameters& params = Parameters());
		virtual DataPoints filter(const DataPoints& input);
		virtual void inPlaceFilter(DataPoints& cloud);
	};
	
	//! Subsampling. Remove point laying in a bounding box
	struct BoundingBoxDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Subsampling. Remove points laying in a bounding box which is axis aligned.";
		}
		inline static const ParametersDoc availableParameters()
		{
			return boost::assign::list_of<ParameterDoc>
				( "xMin", "minimum value on x-axis defining one side of the bounding box", "-1", "-inf", "inf", &P::Comp<T> )
				( "xMax", "maximum value on x-axis defining one side of the bounding box", "1", "-inf", "inf", &P::Comp<T> )
				( "yMin", "minimum value on y-axis defining one side of the bounding box", "-1", "-inf", "inf", &P::Comp<T> )
				( "yMax", "maximum value on y-axis defining one side of the bounding box", "1", "-inf", "inf", &P::Comp<T> )
				( "zMin", "minimum value on z-axis defining one side of the bounding box", "-1", "-inf", "inf", &P::Comp<T> )
				( "zMax", "maximum value on z-axis defining one side of the bounding box", "1", "-inf", "inf", &P::Comp<T> )
				( "removeInside", "If set to true (1), remove points inside the bounding box; else (0), remove points outside the bounding box", "1", "0", "1", P::Comp<bool> )
			;
		}

		const T xMin;
		const T xMax;
		const T yMin;
		const T yMax;
		const T zMin;
		const T zMax;
		const bool removeInside;
		
		//! Constructor, uses parameter interface
		BoundingBoxDataPointsFilter(const Parameters& params = Parameters());
		virtual DataPoints filter(const DataPoints& input);
		virtual void inPlaceFilter(DataPoints& cloud);
	};

	//! Subsampling. Filter points beyond a maximum quantile measured on a specific axis
	struct MaxQuantileOnAxisDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Subsampling. Filter points beyond a maximum quantile measured on a specific axis.";
		}
		inline static const ParametersDoc availableParameters()
		{
			return boost::assign::list_of<ParameterDoc>
				( "dim", "dimension on which the filter will be applied. x=0, y=1, z=2", "0", "0", "2", &P::Comp<unsigned> )
				( "ratio", "maximum quantile authorized. All points beyond that will be filtered.", "0.5", "0.0000001", "0.9999999", &P::Comp<T> )
			;
		}
		
		const unsigned dim;
		const T ratio;
		
		//! Constructor, uses parameter interface
		MaxQuantileOnAxisDataPointsFilter(const Parameters& params = Parameters());
		virtual DataPoints filter(const DataPoints& input);
		virtual void inPlaceFilter(DataPoints& cloud);
	};

	//! Subsampling. Reduce the points number by randomly removing points with a dentsity higher than a treshold.
	struct MaxDensityDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Subsampling. Reduce the points number by randomly removing points with a density highler than a treshold.";
		}
		inline static const ParametersDoc availableParameters()
		{
			return boost::assign::list_of<ParameterDoc>
				( "maxDensity", "Maximum density of points to target. Unit: number of points per m^3.", "10", "0.0000001", "inf", &P::Comp<T> )
			;
		}
		
		const T maxDensity;
		
		//! Constructor, uses parameter interface
		MaxDensityDataPointsFilter(const Parameters& params = Parameters());
		virtual DataPoints filter(const DataPoints& input);
		virtual void inPlaceFilter(DataPoints& cloud);
	};

	//! Surface normals estimation. Find the normal for every point using eigen-decomposition of neighbour points
	struct SurfaceNormalDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Normals. This filter extracts the normal to each point by taking the eigenvector corresponding to the smallest eigenvalue of its nearest neighbors.";
		}
		inline static const ParametersDoc availableParameters()
		{
			return boost::assign::list_of<ParameterDoc>
				( "knn", "number of nearest neighbors to consider, including the point itself", "5", "3", "2147483647", &P::Comp<unsigned> )
				( "epsilon", "approximation to use for the nearest-neighbor search", "0", "0", "inf", &P::Comp<T> )
				( "keepNormals", "whether the normals should be added as descriptors to the resulting cloud", "1" )
				( "keepDensities", "whether the point densities should be added as descriptors to the resulting cloud", "0" )
				( "keepEigenValues", "whether the eigen values should be added as descriptors to the resulting cloud", "0" )
				( "keepEigenVectors", "whether the eigen vectors should be added as descriptors to the resulting cloud", "0" )
				( "keepMatchedIds" , "whethen the identifiers of matches points should be added as descriptors to the resulting cloud", "0" )
			;
		}
		
		const unsigned knn;
		const double epsilon;
		const bool keepNormals;
		const bool keepDensities;
		const bool keepEigenValues;
		const bool keepEigenVectors;
		const bool keepMatchedIds;

		SurfaceNormalDataPointsFilter(const Parameters& params = Parameters());
		virtual ~SurfaceNormalDataPointsFilter() {};
		virtual DataPoints filter(const DataPoints& input);
		virtual void inPlaceFilter(DataPoints& cloud);

		static Vector computeNormal(const Vector eigenVa, const Matrix eigenVe);
		static T computeDensity(const Matrix NN);
		static Vector serializeEigVec(const Matrix eigenVe);
	};

	//! Sampling surface normals. First decimate the space until there is at most knn points, then find the center of mass and use the points to estimate nromal using eigen-decomposition
	struct SamplingSurfaceNormalDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Subsampling, Normals. This filter decomposes the point-cloud space in boxes, by recursively splitting the cloud through axis-aligned hyperplanes such as to maximize the evenness of the aspect ratio of the box. When the number of points in a box reaches a value knn or lower, the filter computes the center of mass of these points and its normal by taking the eigenvector corresponding to the smallest eigenvalue of all points in the box.";
		}
		inline static const ParametersDoc availableParameters()
		{
			return boost::assign::list_of<ParameterDoc>
				( "ratio", "ratio of points to keep with random subsampling. Matrix (normal, density, etc.) will be associated to all points in the same bin.", "0.5", "0.0000001", "0.9999999", &P::Comp<T> )
				( "knn", "determined how many points are used to compute the normals. Direct link with the rapidity of the computation (large = fast). Technically, limit over which a box is splitted in two", "7", "3", "2147483647", &P::Comp<unsigned> )
				( "samplingMethod", "if set to 0, random subsampling using the parameter ratio. If set to 1, bin subsampling with the resulting number of points being 1/knn.", "0", "0", "1", &P::Comp<unsigned> )
				( "maxBoxDim", "maximum length of a box above which the box is discarded", "inf" )
				( "averageExistingDescriptors", "whether the filter keep the existing point descriptors and average them or should it drop them", "1" )
				( "keepNormals", "whether the normals should be added as descriptors to the resulting cloud", "1" )
				( "keepDensities", "whether the point densities should be added as descriptors to the resulting cloud", "0" )
				( "keepEigenValues", "whether the eigen values should be added as descriptors to the resulting cloud", "0" )
				( "keepEigenVectors", "whether the eigen vectors should be added as descriptors to the resulting cloud", "0" )
			;
		}
		
		const T ratio;
		const unsigned knn;
		const unsigned samplingMethod; 
		const T maxBoxDim;
		const bool averageExistingDescriptors;
		const bool keepNormals;
		const bool keepDensities;
		const bool keepEigenValues;
		const bool keepEigenVectors;
		
		
	public:
		SamplingSurfaceNormalDataPointsFilter(const Parameters& params = Parameters());
		virtual ~SamplingSurfaceNormalDataPointsFilter() {}
		virtual DataPoints filter(const DataPoints& input);
		virtual void inPlaceFilter(DataPoints& cloud);

	protected:
		struct BuildData
		{
			typedef std::vector<int> Indices;
			typedef typename DataPoints::View View;
			
			Indices indices;
			Indices indicesToKeep;
			Matrix& features;
			Matrix& descriptors;
			boost::optional<View> normals;
			boost::optional<View> densities;
			boost::optional<View> eigenValues;
			boost::optional<View> eigenVectors;
			int outputInsertionPoint;
			int unfitPointsCount;

			BuildData(Matrix& features, Matrix& descriptors):
				features(features),
				descriptors(descriptors),
				unfitPointsCount(0)
			{
				const int pointsCount(features.cols());
				indices.reserve(pointsCount);
				for (int i = 0; i < pointsCount; ++i)
					indices.push_back(i);
			}
		};
		
		struct CompareDim
		{
			const int dim;
			const BuildData& buildData;
			CompareDim(const int dim, const BuildData& buildData):dim(dim),buildData(buildData){}
			bool operator() (const int& p0, const int& p1)
			{
				return buildData.features(dim, p0) <
						buildData.features(dim, p1);
			}
		};
		
	protected:
		void buildNew(BuildData& data, const int first, const int last, const Vector minValues, const Vector maxValues) const;
		void fuseRange(BuildData& data, const int first, const int last) const;
	};

	//! Reorientation of normals
	struct OrientNormalsDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Normals. Reorient normals so that they all point in the same direction, with respect to the observation points.";
		}
		
		inline static const ParametersDoc availableParameters()
		{
			return boost::assign::list_of<ParameterDoc>
				( "towardCenter", "If set to true(1), all the normals will point inside the surface (i.e. toward the observation points).", "1", "0", "1", &P::Comp<bool> )
			;
		}

		OrientNormalsDataPointsFilter(const Parameters& params = Parameters());
		virtual ~OrientNormalsDataPointsFilter() {};
		virtual DataPoints filter(const DataPoints& input);
		virtual void inPlaceFilter(DataPoints& cloud);

		const bool towardCenter;
	};

	//! Random sampling
	struct RandomSamplingDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Subsampling. This filter reduces the size of the point cloud by randomly dropping points. Based on \\cite{Masuda1996Random}";
		}
		inline static const ParametersDoc availableParameters()
		{
			return boost::assign::list_of<ParameterDoc>
				( "prob", "probability to keep a point, one over decimation factor ", "0.75", "0", "1", &P::Comp<T> )
			;
		}
		
		const double prob;
		
		RandomSamplingDataPointsFilter(const Parameters& params = Parameters());
		virtual ~RandomSamplingDataPointsFilter() {};
		virtual DataPoints filter(const DataPoints& input);
		virtual void inPlaceFilter(DataPoints& cloud);

	protected:
		RandomSamplingDataPointsFilter(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);

	};
	
	//! Maximum number of points
	struct MaxPointCountDataPointsFilter: public RandomSamplingDataPointsFilter
	{
		inline static const std::string description()
		{
			return "Conditional subsampling. This filter reduces the size of the point cloud by randomly dropping points if their number is above maxCount. Based on \\cite{Masuda1996Random}";
		}
		inline static const ParametersDoc availableParameters()
		{
			return boost::assign::list_of<ParameterDoc>
				( "prob", "probability to keep a point, one over decimation factor ", "0.75", "0", "1", &P::Comp<T> )
				( "maxCount", "maximum number of points", "1000", "0", "2147483647", &P::Comp<unsigned> )
			;
		}
		
		const unsigned maxCount;
		
		MaxPointCountDataPointsFilter(const Parameters& params = Parameters());
		virtual ~MaxPointCountDataPointsFilter() {};
		virtual DataPoints filter(const DataPoints& input);
		virtual void inPlaceFilter(DataPoints& cloud);
	};

	//! Systematic sampling, with variation over time
	struct FixStepSamplingDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Subsampling. This filter reduces the size of the point cloud by only keeping one point over step ones; with step varying in time from startStep to endStep, each iteration getting multiplied by stepMult. If use as prefilter (i.e. before the iterations), only startStep is used.";
		}
		inline static const ParametersDoc availableParameters()
		{
			return boost::assign::list_of<ParameterDoc>
				( "startStep", "initial number of point to skip (initial decimation factor)", "10", "1", "2147483647", &P::Comp<unsigned> )
				( "endStep", "maximal or minimal number of points to skip (final decimation factor)", "10", "1", "2147483647", &P::Comp<unsigned> )
				( "stepMult", "multiplication factor to compute the new decimation factor for each iteration", "1", "0.0000001", "inf", &P::Comp<double> )
			;
		}
		
		// number of steps to skip
		const unsigned startStep;
		const unsigned endStep;
		const double stepMult;

	protected:
		double step;
		
	public:
		FixStepSamplingDataPointsFilter(const Parameters& params = Parameters());
		virtual ~FixStepSamplingDataPointsFilter() {};
		virtual void init();
		virtual DataPoints filter(const DataPoints& input);
		virtual void inPlaceFilter(DataPoints& cloud);
	};

	//! Shadow filter, remove ghost points appearing on edges
	struct ShadowDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Remove ghost points appearing on edge discontinuties. Assume that the origine of the point cloud is close to where the laser center was. Requires surface normal for every points";
		}
		
		inline static const ParametersDoc availableParameters()
		{
			return boost::assign::list_of<ParameterDoc>
				( "eps", "Small angle (in rad) around which a normal shoudn't be observable", "0.1", "0.0", "3.1416", &P::Comp<T> )
			;
		}

		const T eps;

		//! Constructor, uses parameter interface
		ShadowDataPointsFilter(const Parameters& params = Parameters());
		
		virtual DataPoints filter(const DataPoints& input);
		virtual void inPlaceFilter(DataPoints& cloud);
	};

	//! Sick LMS-xxx noise model
	struct SimpleSensorNoiseDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Add a 1D descriptor named <sensorNoise> that would represent the noise radius expressed in meter based on SICK LMS specifications \\cite{Pomerleau2012Noise}.";
		}
		
		inline static const ParametersDoc availableParameters()
		{
			return boost::assign::list_of<ParameterDoc>
				( "sensorType", "Type of the sensor used. Choices: 0=Sick LMS-1xx, 1=Hokuyo URG-04LX, 2=Hokuyo UTM-30LX, 3=Kinect/Xtion", "0", "0", "2147483647", &P::Comp<unsigned> )
				( "gain", "If the point cloud is coming from an untrusty source, you can use the gain to augment the uncertainty", "1", "1", "inf", &P::Comp<T> )
			;
		}
	
		const unsigned sensorType;
		const T gain;
		
		//! Constructor, uses parameter interface
		SimpleSensorNoiseDataPointsFilter(const Parameters& params = Parameters());
		
		virtual DataPoints filter(const DataPoints& input);
		virtual void inPlaceFilter(DataPoints& cloud);

	private:
		/// @param minRadius in meter, noise level of depth measurements
		/// @param beamAngle in rad, half of the total laser beam
		/// @param beamConst in meter, minimum size of the laser beam
		/// @param features points from the sensor
		Matrix computeLaserNoise(const T minRadius, const T beamAngle, const T beamConst, const Matrix features);

	};
	
	//! Extract observation direction
	struct ObservationDirectionDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Observation direction. This filter extracts observation directions (vector from point to sensor), considering a sensor at position (x,y,z).";
		}
		
		inline static const ParametersDoc availableParameters()
		{
			return boost::assign::list_of<ParameterDoc>
				( "x", "x-coordinate of sensor", "0" )
				( "y", "y-coordinate of sensor", "0" )
				( "z", "z-coordinate of sensor", "0" )
			;
		}
	
		const T centerX;
		const T centerY;
		const T centerZ;
	
		//! Constructor, uses parameter interface
		ObservationDirectionDataPointsFilter(const Parameters& params = Parameters());
		virtual DataPoints filter(const DataPoints& input);
		virtual void inPlaceFilter(DataPoints& cloud);
	};

	struct VoxelGridDataPointsFilter : public DataPointsFilter
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

    	// Destr
		virtual ~VoxelGridDataPointsFilter() {};

		inline static const std::string description()
		{
			return "Construct Voxel grid of the point cloud. Down-sample by taking centroid or center of grid cells./n";
		}

		inline static const ParametersDoc availableParameters()
		{
			return boost::assign::list_of<ParameterDoc>
			( "vSizeX", "Dimension of each voxel cell in x direction", "1.0", "-inf", "inf", &P::Comp<T> )
			( "vSizeY", "Dimension of each voxel cell in y direction", "1.0", "-inf", "inf", &P::Comp<T> )
			( "vSizeZ", "Dimension of each voxel cell in z direction", "1.0", "-inf", "inf", &P::Comp<T> )
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

		virtual DataPoints filter(const DataPoints& input);
		virtual void inPlaceFilter(DataPoints& cloud);

	};	

}; // DataPointsFiltersImpl

#endif // __POINTMATCHER_DATAPOINTSFILTERS_H

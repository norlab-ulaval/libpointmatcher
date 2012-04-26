// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2011,
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
	typedef typename PointMatcher<T>::DataPoints::Descriptors Descriptors;
	typedef typename PointMatcher<T>::DataPointsFilter DataPointsFilter;
	
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
		//		{ "param1", "Description of the parameter", "defaultValue", "minValue", "maxValue", type of the parameter },
		//		{ "param2", "Description of the parameter", "defaultValue", "minValue", "maxValue", type of the parameter }
		//	});
		//}
		//! Constructor, uses parameter interface
		//IdentityDataPointsFilter(const Parameters& params = Parameters());
		
		virtual DataPoints filter(const DataPoints& input);
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
			return ParametersDoc({
				{ "dim", "dimension on which the filter will be applied. x=0, y=1, z=2, radius=-1", "-1", "-1", "2", &P::Comp<int> },
				{ "maxDist", "maximum distance authorized. All points beyond that will be filtered.", "1", "0", "inf", P::Comp<T> }
			});
		}

		const int dim;
		const T maxDist;
		
		//! Constructor, uses parameter interface
		MaxDistDataPointsFilter(const Parameters& params = Parameters());
		virtual DataPoints filter(const DataPoints& input);
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
			return ParametersDoc({
				{ "dim", "dimension on which the filter will be applied. x=0, y=1, z=2, radius=-1", "-1", "-1", "2", &P::Comp<int> },
				{ "minDist", "minimum distance authorized. All points before that will be filtered.", "1", "0", "inf", &P::Comp<T> }
			});
		}
		
		const int dim;
		const T minDist;
		
		//! Constructor, uses parameter interface
		MinDistDataPointsFilter(const Parameters& params = Parameters());
		virtual DataPoints filter(const DataPoints& input);
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
			return ParametersDoc({
				{ "dim", "dimension on which the filter will be applied. x=0, y=1, z=2", "0", "0", "2", &P::Comp<unsigned> },
				{ "ratio", "maximum quantile authorized. All points beyond that will be filtered.", "0.5", "0.0000001", "0.9999999", &P::Comp<T>}
			});
		}
		
		const unsigned dim;
		const T ratio;
		
		//! Constructor, uses parameter interface
		MaxQuantileOnAxisDataPointsFilter(const Parameters& params = Parameters());
		virtual DataPoints filter(const DataPoints& input);
	};

	//! Subsampling. Reduce the points number of a certain ratio while trying to uniformize the density of the point cloud.
	struct UniformizeDensityDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Subsampling. Reduce the points number while trying to uniformize the density of the point cloud.";
		}
		inline static const ParametersDoc availableParameters()
		{
			return ParametersDoc({
				{"aggressivity", "how aggressively high density points should be subsampled.", "0.5", "0.0000001", "0.9999999", &P::Comp<T>},
				{ "nbBin", "number of bin used to estimate the probability distribution of the density.", "15", "1", "2147483647", &P::Comp<unsigned> }
			});
		}
		
		const T aggressivity;
		const unsigned nbBin;
		
		//! Constructor, uses parameter interface
		UniformizeDensityDataPointsFilter(const Parameters& params = Parameters());
		virtual DataPoints filter(const DataPoints& input);
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
			return ParametersDoc({
				{ "knn", "number of nearest neighbors to consider, including the point itself", "5", "3", "2147483647", &P::Comp<unsigned> },
				{ "epsilon", "approximation to use for the nearest-neighbor search", "0", "0", "inf", &P::Comp<T> },
				{ "keepNormals", "whether the normals should be added as descriptors to the resulting cloud", "1"},
				{ "keepDensities", "whether the point densities should be added as descriptors to the resulting cloud", "0"},
				{ "keepEigenValues", "whether the eigen values should be added as descriptors to the resulting cloud", "0" },
				{ "keepEigenVectors", "whether the eigen vectors should be added as descriptors to the resulting cloud", "0" },
				{ "keepMatchedIds" , "whethen the identifiers of matches points should be added as descriptors to the resulting cloud", "0" }
			});
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

		static Vector computeNormal(const Vector eigenVa, const Matrix eigenVe);
		static T computeDensity(const Matrix NN);
		static Vector serializeEigVec(const Matrix eigenVe);
	};

	//! Sampling surface normals. First decimate the space until there is at most binSize points, then find the center of mass and use the points to estimate nromal using eigen-decomposition
	//FIXME: the name of the normals field is "triangle_normals"
	struct SamplingSurfaceNormalDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Subsampling, Normals. This filter decomposes the point-cloud space in boxes, by recursively splitting the cloud through axis-aligned hyperplanes such as to maximize the evenness of the aspect ratio of the box. When the number of points in a box reaches a value binSize or lower, the filter computes the center of mass of these points and its normal by taking the eigenvector corresponding to the smallest eigenvalue of all points in the box.";
		}
		inline static const ParametersDoc availableParameters()
		{
			return ParametersDoc({
				{ "ratio", "ratio of points to keep with random subsampling. Descriptors (normal, density, etc.) will be associated to all points in the same bin.", "0.5", "0.0000001", "0.9999999", &P::Comp<T> },
				{ "binSize", "determined how many points are used to compute the normals. Direct link with the rapidity of the computation (large = fast). Technically, limit over which a box is splitted in two", "7", "3", "2147483647", &P::Comp<unsigned> },
				{ "samplingMethod", "if set to 0, random subsampling using the parameter ratio. If set to 1, bin subsampling with the resulting number of points being 1/binSize.", "0", "0", "1", &P::Comp<unsigned> },
				{ "averageExistingDescriptors", "whether the filter keep the existing point descriptors and average them or should it drop them", "1" },
				{ "keepNormals", "whether the normals should be added as descriptors to the resulting cloud", "1" },
				{ "keepDensities", "whether the point densities should be added as descriptors to the resulting cloud", "0" },
				{ "keepEigenValues", "whether the eigen values should be added as descriptors to the resulting cloud", "0" },
				{ "keepEigenVectors", "whether the eigen vectors should be added as descriptors to the resulting cloud", "0" }
			});
		}
		
		const T ratio;
		const unsigned binSize;
		const unsigned samplingMethod; 
		const bool averageExistingDescriptors;
		const bool keepNormals;
		const bool keepDensities;
		const bool keepEigenValues;
		const bool keepEigenVectors;
		
		
	public:
		SamplingSurfaceNormalDataPointsFilter(const Parameters& params = Parameters());
		virtual ~SamplingSurfaceNormalDataPointsFilter() {}
		virtual DataPoints filter(const DataPoints& input);
		
	protected:
		struct BuildData
		{
			typedef std::vector<int> Indices;
			
			Indices indices;
			const Matrix& inputFeatures;
			const Matrix& inputDescriptors;
			Matrix outputFeatures;
			Matrix outputDescriptors;
			Descriptors normals;
			Descriptors densities;
			Descriptors eigValues;
			Descriptors eigVectors;
			int outputInsertionPoint;
			int unfitPointsCount;
			
			BuildData(const Matrix& inputFeatures, const Matrix& inputDescriptors):
				inputFeatures(inputFeatures),
				inputDescriptors(inputDescriptors),
				outputFeatures(inputFeatures.rows(), inputFeatures.cols()),
				outputDescriptors(inputDescriptors.rows(), inputDescriptors.cols()),
				outputInsertionPoint(0),
				unfitPointsCount(0)
			{
				const int pointsCount(inputFeatures.cols());
				indices.reserve(pointsCount);
				for (int i = 0; i < pointsCount; ++i)
					indices.push_back(i);
			}

			Matrix getResizedMatrix(const Matrix input) const
			{
				if(input.rows() != 0)
					return input.leftCols(outputInsertionPoint);
				else
					return input;
			}
		};
		
		struct CompareDim
		{
			const int dim;
			const BuildData& buildData;
			CompareDim(const int dim, const BuildData& buildData):dim(dim),buildData(buildData){}
			bool operator() (const int& p0, const int& p1)
			{
				return  buildData.inputFeatures(dim, p0) < 
						buildData.inputFeatures(dim, p1);
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
			return "Normals. Reorient normals so that they all point in the same direction, with respect to the origin of the point cloud.";
		}
		
		inline static const ParametersDoc availableParameters()
		{
			return ParametersDoc({
				{ "towardCenter", "If set to true(1), all the normals will point inside the surface (i.e. toward the center of the point cloud).", "1", "0", "1", &P::Comp<bool>}
			});
		}

		OrientNormalsDataPointsFilter(const Parameters& params = Parameters());
		virtual ~OrientNormalsDataPointsFilter() {};
		virtual DataPoints filter(const DataPoints& input);

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
			return ParametersDoc({
				{ "prob", "probability to keep a point, one over decimation factor ", "0.75", "0", "1", &P::Comp<T> }
			});
		}
		
		const double prob;
		
		RandomSamplingDataPointsFilter(const Parameters& params = Parameters());
		virtual ~RandomSamplingDataPointsFilter() {};
		virtual DataPoints filter(const DataPoints& input);
		
	private:
		DataPoints randomSample(const DataPoints& input) const;
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
			return ParametersDoc({
				{ "startStep", "initial number of point to skip (initial decimation factor)", "10", "1", "2147483647", &P::Comp<unsigned> },
				{ "endStep", "maximal or minimal number of points to skip (final decimation factor)", "10", "1", "2147483647", &P::Comp<unsigned> },
				{ "stepMult", "multiplication factor to compute the new decimation factor for each iteration", "1", "0.0000001", "inf", &P::Comp<double> }
			});
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
	private:
		DataPoints fixstepSample(const DataPoints& input);
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
			return ParametersDoc({
				{ "eps", "Small angle (in rad) around which a normal shoudn't be observable", "0.1", "0.0", "3.1416", &P::Comp<T> }
			});
		}

	protected:
		T eps;

	public:
		//! Constructor, uses parameter interface
		ShadowDataPointsFilter(const Parameters& params = Parameters());
		
		virtual DataPoints filter(const DataPoints& input);
	};

	//! Sick LMS-xxx noise model
	struct SimpleSensorNoiseDataPointsFilter: public DataPointsFilter
	{
		inline static const std::string description()
		{
			return "Add a 1D descriptor named <sensorNoise> that would represent the noise radius expressed in meter based on SICK LMS specifications.";
		}
		
		inline static const ParametersDoc availableParameters()
		{
			return ParametersDoc({
				{ "sensorType", "Type of the sensor used. Choices: 0=SickLMS", "0", "0", "2147483647", &P::Comp<unsigned> },
				{ "gain", "If the point cloud is coming from an untrusty source, you can use the gain to augment the uncertainty", "1", "1", "inf", &P::Comp<T> }
			});
		}
	protected:
		const unsigned sensorType;
		const T gain;
	public:
		//! Constructor, uses parameter interface
		SimpleSensorNoiseDataPointsFilter(const Parameters& params = Parameters());
		
		virtual DataPoints filter(const DataPoints& input);
	};


}; // DataPointsFiltersImpl

#endif // __POINTMATCHER_DATAPOINTSFILTERS_H

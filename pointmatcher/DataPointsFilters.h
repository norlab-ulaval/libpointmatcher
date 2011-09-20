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

#define ZERO_PLUS_EPS (0. + std::numeric_limits<double>::epsilon())
#define ONE_MINUS_EPS (1. - std::numeric_limits<double>::epsilon())

//! Identidy, does nothing
struct IdentityDataPointsFilter: public DataPointsFilter
{
	static const std::string description()
	{
		return "does nothing";
	}
	static const ParametersDoc availableParameters()
	{
		return {};
	}
	
	IdentityDataPointsFilter(const Parameters& params) {}
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
};

//! Subsampling. Filter points beyond a maximum distance measured on a specific axis
struct MaxDistOnAxisDataPointsFilter: public DataPointsFilter
{
	static const std::string description()
	{
		return "Subsampling. Filter points beyond a maximum distance measured on a specific axis.";
	}
	static const ParametersDoc availableParameters()
	{
		return {
			{ "dim", "dimension on which the filter will be applied. x=0, y=1, z=2", 0, 0 },
			{ "maxDist", "maximum distance authorized. All points beyond that will be filtered.", 1.0, 0. }
		};
	}

	const unsigned dim;
	const T maxDist;
	
	//! Constructor, uses parameter interface
	MaxDistOnAxisDataPointsFilter(const Parameters& params);
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
};

//! Subsampling. Filter points before a minimum distance measured on a specific axis
struct MinDistOnAxisDataPointsFilter: public DataPointsFilter
{
	static const std::string description()
	{
		return "Subsampling. Filter points before a minimum distance measured on a specific axis.";
	}
	static const ParametersDoc availableParameters()
	{
		return {
			{ "dim", "dimension on which the filter will be applied. x=0, y=1, z=2", 0, 0},
			{ "minDist", "minimum distance authorized. All points before that will be filtered.", 1.0, 0. }
		};
	}
	
	const unsigned dim;
	const T minDist;
	
	//! Constructor, uses parameter interface
	MinDistOnAxisDataPointsFilter(const Parameters& params);
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
};

//! Subsampling. Filter points beyond a maximum quantile measured on a specific axis
struct MaxQuantileOnAxisDataPointsFilter: public DataPointsFilter
{
	static const std::string description()
	{
		return "Subsampling. Filter points beyond a maximum quantile measured on a specific axis.";
	}
	static const ParametersDoc availableParameters()
	{
		return {
			{ "dim", "dimension on which the filter will be applied. x=0, y=1, z=2", 0, 0},
			{ "ratio", "maximum quantile authorized. All points beyond that will be filtered.", 0.5, ZERO_PLUS_EPS, ONE_MINUS_EPS}
		};
	}
	
	const unsigned dim;
	const T ratio;
	
	//! Constructor, uses parameter interface
	MaxQuantileOnAxisDataPointsFilter(const Parameters& params);
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
};

//! Subsampling. Reduce the points number of a certain ration while trying to uniformize the density of the point cloud.
struct UniformizeDensityDataPointsFilter: public DataPointsFilter
{
	static const std::string description()
	{
		return "Subsampling. Reduce the points number of a certain ration while trying to uniformize the density of the point cloud.";
	}
	static const ParametersDoc availableParameters()
	{
		return {
			{ "ratio", "targeted reduction ratio", 0.5, ZERO_PLUS_EPS, ONE_MINUS_EPS},
			{ "nbBin", "number of bin used to estimate the probability distribution of the density.", 1, 1}
		};
	}
	
	const T ratio;
	const unsigned nbBin;
	
	//! Constructor, uses parameter interface
	UniformizeDensityDataPointsFilter(const Parameters& params);
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
};

//! Surface normals estimation. Find the normal for every point using eigen-decomposition of neighbour points
struct SurfaceNormalDataPointsFilter: public DataPointsFilter
{
	static const std::string description()
	{
		return "Normals. This filter extracts the normal to each point by taking the eigenvector corresponding to the smallest eigenvalue of its nearest neighbors.";
	}
	static const ParametersDoc availableParameters()
	{
		return {
			{ "knn", "number of nearest neighbors to consider, including the point itself", 5, 3 },
			{ "epsilon", "approximation to use for the nearest-neighbor search", 0., 0. },
			{ "keepNormals", "whether the normals should be added as descriptors to the resulting cloud", true },
			{ "keepDensities", "whether the point densities should be added as descriptors to the resulting cloud", false },
			{ "keepEigenValues", "whether the eigen values should be added as descriptors to the resulting cloud", false },
			{ "keepEigenVectors", "whether the eigen vectors should be added as descriptors to the resulting cloud", false },
			{ "keepMatchedIds" , "whethen the identifiers of matches points should be added as descriptors to the resulting cloud", false }
		};
	}
	
	const int knn; // FIXME: shouldn't we put unsigned?
	const double epsilon;
	const bool keepNormals;
	const bool keepDensities;
	const bool keepEigenValues;
	const bool keepEigenVectors;
	const bool keepMatchedIds;

	SurfaceNormalDataPointsFilter(const Parameters& params);
	virtual ~SurfaceNormalDataPointsFilter() {};
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
};

//! Sampling surface normals. First decimate the space until there is at most binSize points, then find the center of mass and use the points to estimate nromal using eigen-decomposition
struct SamplingSurfaceNormalDataPointsFilter: public DataPointsFilter
{
	static const std::string description()
	{
		return "Subsampling, Normals. This filter decomposes the point-cloud space in boxes, by recursively splitting the cloud through axis-aligned hyperplanes such as to maximize the evenness of the aspect ratio of the box. When the number of points in a box reaches a value binSize or lower, the filter computes the center of mass of these points and its normal by taking the eigenvector corresponding to the smallest eigenvalue of all points in the box.";
	}
	static const ParametersDoc availableParameters()
	{
		return {
			{ "binSize", "limit over which a box is splitted in two", 10, 3 },
			{ "averageExistingDescriptors", "whether the filter keep the existing point descriptors and average them or should it drop them", true },
			{ "keepNormals", "whether the normals should be added as descriptors to the resulting cloud", true },
			{ "keepDensities", "whether the point densities should be added as descriptors to the resulting cloud", false },
			{ "keepEigenValues", "whether the eigen values should be added as descriptors to the resulting cloud", false },
			{ "keepEigenVectors", "whether the eigen vectors should be added as descriptors to the resulting cloud", false }
		};
	}
	
	const int binSize; // FIXME: shouldn't we put unsigned?
	const bool averageExistingDescriptors;
	const bool keepNormals;
	const bool keepDensities;
	const bool keepEigenValues;
	const bool keepEigenVectors;
	
public:
	SamplingSurfaceNormalDataPointsFilter(const Parameters& params);
	virtual ~SamplingSurfaceNormalDataPointsFilter() {}
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
	
protected:
	struct BuildData
	{
		typedef std::vector<int> Indices;
		
		Indices indices;
		const Matrix& inputFeatures;
		const Matrix& inputDescriptors;
		Matrix outputFeatures;
		Matrix outputDescriptors;
		int outputInsertionPoint;
		
		BuildData(const Matrix& inputFeatures, const Matrix& inputDescriptors, const int finalDescDim):
			inputFeatures(inputFeatures),
			inputDescriptors(inputDescriptors),
			outputFeatures(inputFeatures.rows(), inputFeatures.cols()),
			outputDescriptors(finalDescDim, inputFeatures.cols()),
			outputInsertionPoint(0)
		{
			const int pointsCount(inputFeatures.cols());
			indices.reserve(pointsCount);
			for (int i = 0; i < pointsCount; ++i)
				indices[i] = i;
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
	static const std::string description()
	{
		return "Normals. Reorient normals so that they all point in the same direction, with respect to coordinate 0.";
	}
	static const ParametersDoc availableParameters()
	{
		return {};
	}
	
	OrientNormalsDataPointsFilter(const Parameters& params) {}
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
};

//! Random sampling
struct RandomSamplingDataPointsFilter: public DataPointsFilter
{
	static const std::string description()
	{
		return "Subsampling. This filter reduces the size of the point cloud by randomly dropping points.";
	}
	static const ParametersDoc availableParameters()
	{
		return {
			{ "prob", "probability to keep a point, one over decimation factor ", 0.5, 0., 1. }
		};
	}
	
	const double prob;
	
	RandomSamplingDataPointsFilter(const Parameters& params);
	virtual ~RandomSamplingDataPointsFilter() {};
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
	
private:
	DataPoints randomSample(const DataPoints& input) const;
};

//! Systematic sampling, with variation over time
struct FixstepSamplingDataPointsFilter: public DataPointsFilter
{
	static const std::string description()
	{
		return "Subsampling. This filter reduces the size of the point cloud by only keeping one point over step ones; with step varying in time from startStep to endStep, each iteration getting multiplied by stepMult.";
	}
	static const ParametersDoc availableParameters()
	{
		return {
			{ "startStep", "initial number of point to skip (initial decimation factor)", 10., ZERO_PLUS_EPS },
			{ "endStep", "maximal or minimal number of points to skip (final decimation factor)", 10., ZERO_PLUS_EPS },
			{ "stepMult", "multiplication factor to compute the new decimation factor for each iteration", 1., ZERO_PLUS_EPS }
		};
	}
	
	// number of steps to skip
	const double startStep;
	const double endStep;
	const double stepMult;

protected:
	double step;
	
public:
	FixstepSamplingDataPointsFilter(const Parameters& params);
	virtual ~FixstepSamplingDataPointsFilter() {};
	virtual void init();
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
private:
	DataPoints fixstepSample(const DataPoints& input);
};

#endif // __POINTMATCHER_DATAPOINTSFILTERS_H

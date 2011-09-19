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

// Identidy
struct IdentityDataPointsFilter: public DataPointsFilter
{
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
};

//! Subsampling. Filter points beyond a maximum distance measured on a specific axis
struct MaxDistOnAxisDataPointsFilter: public DataPointsFilter
{
	const unsigned dim;
	const T maxDist;
	
	//! Constructor
	/*
		\param dim dimension on which the filter will be applied. x=0, y=1, z=2
		\param maxDist maximum distance authorized. All points beyond that will be filtered. Expecting value within [0;inf[
	*/
	MaxDistOnAxisDataPointsFilter(const unsigned dim, const T maxDist);
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
};

//! Subsampling. Filter points before a minimum distance measured on a specific axis
struct MinDistOnAxisDataPointsFilter: public DataPointsFilter
{
	const unsigned dim;
	const T minDist;
	
	//! Constructor
	/*
		\param dim dimension on which the filter will be applied. x=0, y=1, z=2
		\param minDist minimum distance authorized. All points before that will be filtered. Expecting value within [0;inf[
	*/
	MinDistOnAxisDataPointsFilter(const unsigned dim, const T minDist);
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
};

//! Subsampling. Filter points beyond a maximum quantile measured on a specific axis
struct MaxQuantileOnAxisDataPointsFilter: public DataPointsFilter
{
	const unsigned dim;
	const T ratio;
	
	//! Constructor
	/*
		\param dim dimension on which the filter will be applied. x=0, y=1, z=2
		\param ratio maximum quantile authorized. All points beyond that will be filtered. Expecting value within ]0;1[
	*/
	MaxQuantileOnAxisDataPointsFilter(const unsigned dim, const T ratio);
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
};

//! Subsampling. Reduce the points number of a certain ration while trying to uniformize the density of the point cloud.
struct UniformizeDensityDataPointsFilter: public DataPointsFilter
{
	const T ratio;
	const int nbBin;
	
	//! Constructor
	/*
		\param ratio targeted reduction ratio. Expecting value within ]0;1[
		\param nbBin number of bin used to estimate the probability distribution of the density. Expecting value within ]0;inf]
	*/
	UniformizeDensityDataPointsFilter(const T ratio, const int nbBin);
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
};

// Surface normals
class SurfaceNormalDataPointsFilter: public DataPointsFilter
{
	const int knn;
	const double epsilon;
	const bool keepNormals;
	const bool keepDensities;
	const bool keepEigenValues;
	const bool keepEigenVectors;
	const bool keepMatchedIds;
	
public:
	SurfaceNormalDataPointsFilter(const int knn = 5, 
		const double epsilon = 0,
		const bool keepNormals = true,
		const bool keepDensities = false,
		const bool keepEigenValues = false, 
		const bool keepEigenVectors = false,
		const bool keepMatchedIds = false);
	virtual ~SurfaceNormalDataPointsFilter() {};
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
};

// Sampling surface normals
class SamplingSurfaceNormalDataPointsFilter: public DataPointsFilter
{
	const int binSize;
	const bool averageExistingDescriptors;
	const bool keepNormals;
	const bool keepDensities;
	const bool keepEigenValues;
	const bool keepEigenVectors;
	
public:
	SamplingSurfaceNormalDataPointsFilter(const int binSize = 10,
		const bool averageExistingDescriptors = true,
		const bool keepNormals = true,
		const bool keepDensities = false,
		const bool keepEigenValues = false, 
		const bool keepEigenVectors = false);
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

// Reorientation of normals
class OrientNormalsDataPointsFilter: public DataPointsFilter
{
public:
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
};

// Random sampling
class RandomSamplingDataPointsFilter: public DataPointsFilter
{
	// Probability to keep points, between 0 and 1
	const double prob;
	
public:
	RandomSamplingDataPointsFilter(const double ratio = 0.5);
	virtual ~RandomSamplingDataPointsFilter() {};
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
private:
	DataPoints randomSample(const DataPoints& input) const;
};

// Systematic sampling
class FixstepSamplingDataPointsFilter: public DataPointsFilter
{
	// number of steps to skip
	const double startStep;
	const double endStep;
	const double stepMult;
	double step;
	
public:
	FixstepSamplingDataPointsFilter(const double startStep = 10, const double endStep = 10, const double stepMult = 1);
	virtual ~FixstepSamplingDataPointsFilter() {};
	virtual void init();
	virtual DataPoints filter(const DataPoints& input, bool& iterate);
private:
	DataPoints fixstepSample(const DataPoints& input);
};

#endif // __POINTMATCHER_DATAPOINTSFILTERS_H

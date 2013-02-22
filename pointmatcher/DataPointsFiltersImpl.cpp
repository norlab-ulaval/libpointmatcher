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

#include "DataPointsFiltersImpl.h"
#include "PointMatcherPrivate.h"
#include "MatchersImpl.h"
#include "Functions.h"

#include <algorithm>
#include <boost/format.hpp>

// Eigenvalues
#include "Eigen/QR"

using namespace std;
using namespace PointMatcherSupport;

// IdentityDataPointsFilter
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::IdentityDataPointsFilter::filter(
	const DataPoints& input)
{
	return input;
}

template struct DataPointsFiltersImpl<float>::IdentityDataPointsFilter;
template struct DataPointsFiltersImpl<double>::IdentityDataPointsFilter;


// RemoveNaNDataPointsFilter
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::RemoveNaNDataPointsFilter::filter(
	const DataPoints& input)
{
	const int nbPointsIn = input.features.cols();
	
	DataPoints output(input.createSimilarEmpty());
	
	int j = 0;
	for (int i = 0; i < nbPointsIn; ++i)
	{
		const auto colArray(input.features.col(i).array());
		const bool hasNaN(!(colArray == colArray).all());
		if (!hasNaN)
		{
			output.setColFrom(j, input, i);
			j++;
		}
	}
	
	output.conservativeResize(j);

	return output;
}

template struct DataPointsFiltersImpl<float>::RemoveNaNDataPointsFilter;
template struct DataPointsFiltersImpl<double>::RemoveNaNDataPointsFilter;


// MaxDistDataPointsFilter
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::MaxDistDataPointsFilter::MaxDistDataPointsFilter(const Parameters& params):
	DataPointsFilter("MaxDistDataPointsFilter", MaxDistDataPointsFilter::availableParameters(), params),
	dim(Parametrizable::get<unsigned>("dim")),
	maxDist(Parametrizable::get<T>("maxDist"))
{
}

template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::MaxDistDataPointsFilter::filter(const DataPoints& input)
{
	if (dim >= input.features.rows() - 1)
	{
		throw InvalidParameter(
			(boost::format("MaxDistDataPointsFilter: Error, filtering on dimension number %1%, larger than authorized axis id %2%") % dim % (input.features.rows() - 2)).str());
	}

	const int nbPointsIn = input.features.cols();
	const int nbRows = input.features.rows();
	
	DataPoints output(input.createSimilarEmpty());
	
	int j = 0;
	if(dim == -1) // Euclidean distance
	{
		for (int i = 0; i < nbPointsIn; i++)
		{
			const T absMaxDist = anyabs(maxDist);
			if (input.features.col(i).head(nbRows-1).norm() < absMaxDist)
			{
				output.setColFrom(j, input, i);
				j++;
			}
		}
	}
	else // Single-axis distance
	{
		for (int i = 0; i < nbPointsIn; i++)
		{
			if ((input.features(dim, i)) < maxDist)
			{
				output.setColFrom(j, input, i);
				j++;
			}
		}
	}
	
	output.conservativeResize(j);
	
	return output;
}

template struct DataPointsFiltersImpl<float>::MaxDistDataPointsFilter;
template struct DataPointsFiltersImpl<double>::MaxDistDataPointsFilter;


// MinDistDataPointsFilter
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::MinDistDataPointsFilter::MinDistDataPointsFilter(const Parameters& params):
	DataPointsFilter("MinDistDataPointsFilter", MinDistDataPointsFilter::availableParameters(), params),
	dim(Parametrizable::get<unsigned>("dim")),
	minDist(Parametrizable::get<T>("minDist"))
{
}

template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::MinDistDataPointsFilter::filter(const DataPoints& input)
{
	if (dim >= input.features.rows() - 1)
		throw InvalidParameter((boost::format("MinDistDataPointsFilter: Error, filtering on dimension number %1%, larger than feature dimensionality %2%") % dim % (input.features.rows() - 2)).str());
	
	const int nbPointsIn = input.features.cols();
	const int nbRows = input.features.rows();
	
	DataPoints output(input.createSimilarEmpty());
	
	int j = 0;
	if(dim == -1) // Euclidean distance
	{
		const T absMinDist = anyabs(minDist);
		for (int i = 0; i < nbPointsIn; i++)
		{
			if (input.features.col(i).head(nbRows-1).norm() > absMinDist)
			{
				output.setColFrom(j, input, i);
				j++;
			}
		}
	}
	else // Single axis distance
	{
		for (int i = 0; i < nbPointsIn; i++)
		{
			if ((input.features(dim, i)) > minDist)
			{
				output.setColFrom(j, input, i);
				j++;
			}
		}
	}
	
	output.conservativeResize(j);
	
	return output;
}

template struct DataPointsFiltersImpl<float>::MinDistDataPointsFilter;
template struct DataPointsFiltersImpl<double>::MinDistDataPointsFilter;


// BoundingBoxDataPointsFilter
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::BoundingBoxDataPointsFilter::BoundingBoxDataPointsFilter(const Parameters& params):
	DataPointsFilter("BoundingBoxDataPointsFilter", BoundingBoxDataPointsFilter::availableParameters(), params),
	xMin(Parametrizable::get<T>("xMin")),
	xMax(Parametrizable::get<T>("xMax")),
	yMin(Parametrizable::get<T>("yMin")),
	yMax(Parametrizable::get<T>("yMax")),
	zMin(Parametrizable::get<T>("zMin")),
	zMax(Parametrizable::get<T>("zMax")),
	removeInside(Parametrizable::get<bool>("removeInside"))
{
}

template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::BoundingBoxDataPointsFilter::filter(const DataPoints& input)
{
	const int nbPointsIn = input.features.cols();
	const int nbRows = input.features.rows();

	DataPoints output(input.createSimilarEmpty());

	int j = 0;
	for (int i = 0; i < nbPointsIn; i++)
	{
		bool keepPt = false;
		const Vector point = input.features.col(i);
		
		// FIXME: improve performance by using Eigen array operations
		const bool x_in = (point(0) > xMin && point(0) < xMax);
		const bool y_in = (point(1) > yMin && point(1) < yMax);
		const bool z_in = (point(2) > zMin && point(2) < zMax) || nbRows == 3;
		const bool in_box = x_in && y_in && z_in;

		if(removeInside)
			keepPt = !in_box;
		else
			keepPt = in_box;

		if(keepPt)
		{
			output.setColFrom(j, input, i);
			j++;
		}
	}
	
	output.conservativeResize(j);
	
	return output;
}

template struct DataPointsFiltersImpl<float>::BoundingBoxDataPointsFilter;
template struct DataPointsFiltersImpl<double>::BoundingBoxDataPointsFilter;


// MaxQuantileOnAxisDataPointsFilter
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::MaxQuantileOnAxisDataPointsFilter::MaxQuantileOnAxisDataPointsFilter(const Parameters& params):
	DataPointsFilter("MaxQuantileOnAxisDataPointsFilter", MaxQuantileOnAxisDataPointsFilter::availableParameters(), params),
	dim(Parametrizable::get<unsigned>("dim")),
	ratio(Parametrizable::get<T>("ratio"))
{
}

template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::MaxQuantileOnAxisDataPointsFilter::filter(const DataPoints& input)
{
	if (int(dim) >= input.features.rows())
		throw InvalidParameter((boost::format("MaxQuantileOnAxisDataPointsFilter: Error, filtering on dimension number %1%, larger than feature dimensionality %2%") % dim % input.features.rows()).str());
	
	const int nbPointsIn = input.features.cols();
	const int nbPointsOut = nbPointsIn * ratio;
	
	// build array
	vector<T> values;
	values.reserve(input.features.cols());
	for (int x = 0; x < input.features.cols(); ++x)
		values.push_back(input.features(dim, x));
	
	// get quartiles value
	nth_element(values.begin(), values.begin() + (values.size() * ratio), values.end());
	const T limit = values[nbPointsOut];
	
	// build output values
	DataPoints output(input.createSimilarEmpty(nbPointsOut));
	
	// fill output values
	int j = 0;
	for (int i = 0; i < nbPointsIn; i++)
	{
		if (input.features(dim, i) < limit)
		{
			output.setColFrom(j, input, i);
			j++;
		}
	}
	assert(j <= nbPointsOut);
	
	if (j < nbPointsOut)
		output.conservativeResize(j);
	
	return output;
}

template struct DataPointsFiltersImpl<float>::MaxQuantileOnAxisDataPointsFilter;
template struct DataPointsFiltersImpl<double>::MaxQuantileOnAxisDataPointsFilter;


// MaxDensityDataPointsFilter
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::MaxDensityDataPointsFilter::MaxDensityDataPointsFilter(const Parameters& params):
	DataPointsFilter("MaxDensityDataPointsFilter", MaxDensityDataPointsFilter::availableParameters(), params),
	maxDensity(Parametrizable::get<T>("maxDensity"))
{
}

template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::MaxDensityDataPointsFilter::filter(const DataPoints& input)
{
	typedef typename DataPoints::View View;
	typedef typename DataPoints::ConstView ConstView;

	// Force densities to be computed
	if (!input.descriptorExists("densities"))
	{
		throw InvalidField("MaxDensityDataPointsFilter: Error, no densities found in descriptors.");
	}

	DataPoints output(input.createSimilarEmpty());
	
	const int nbPointsIn = output.features.cols();
	ConstView densities = input.getDescriptorViewByName("densities");
	const T lastDensity = densities.maxCoeff();
	const int nbSaturatedPts = (densities.cwise() == lastDensity).count();

	// fill output values
	int j = 0;
	for (int i = 0; i < nbPointsIn; i++)
	{
		const T density(densities(0,i));
		if (density > maxDensity)
		{
			const float r = (float)std::rand()/(float)RAND_MAX;
			float acceptRatio = maxDensity/density;
			
			// Handle saturation value of density
			if (density == lastDensity)
			{
				acceptRatio = acceptRatio * (1-nbSaturatedPts/nbPointsIn);	
			}

			if (r < acceptRatio)
			{
				output.setColFrom(j, input, i);
				j++;
			}
		}
		else
		{
			output.setColFrom(j, input, i);
			j++;
		}
	}

	output.conservativeResize(j);

	return output;
}

template struct DataPointsFiltersImpl<float>::MaxDensityDataPointsFilter;
template struct DataPointsFiltersImpl<double>::MaxDensityDataPointsFilter;



// SurfaceNormalDataPointsFilter
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::SurfaceNormalDataPointsFilter::SurfaceNormalDataPointsFilter(const Parameters& params):
	DataPointsFilter("SurfaceNormalDataPointsFilter", SurfaceNormalDataPointsFilter::availableParameters(), params),
	knn(Parametrizable::get<int>("knn")),
	epsilon(Parametrizable::get<T>("epsilon")),
	keepNormals(Parametrizable::get<bool>("keepNormals")),
	keepDensities(Parametrizable::get<bool>("keepDensities")),
	keepEigenValues(Parametrizable::get<bool>("keepEigenValues")),
	keepEigenVectors(Parametrizable::get<bool>("keepEigenVectors")),
	keepMatchedIds(Parametrizable::get<bool>("keepMatchedIds"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::SurfaceNormalDataPointsFilter::filter(
	const DataPoints& input)
{
	typedef typename DataPoints::View View;
	typedef typename DataPoints::Label Label;
	typedef typename DataPoints::Labels Labels;
	typedef typename MatchersImpl<T>::KDTreeMatcher KDTreeMatcher;
	typedef typename PointMatcher<T>::Matches Matches;
	
	const int pointsCount(input.features.cols());
	const int featDim(input.features.rows());
	const int descDim(input.descriptors.rows());

	// Validate descriptors and labels
	int insertDim(0);
	for(unsigned int i = 0; i < input.descriptorLabels.size(); i++)
		insertDim += input.descriptorLabels[i].span;
	if (insertDim != descDim)
		throw InvalidField("SurfaceNormalDataPointsFilter: Error, descriptor labels do not match descriptor data");
	
	// Reserve memory for new descriptors
	const int dimNormals(featDim-1);
	const int dimDensities(1);
	const int dimEigValues(featDim-1);
	const int dimEigVectors((featDim-1)*(featDim-1));
	//const int dimMatchedIds(knn); 

	boost::optional<View> normals;
	boost::optional<View> densities;
	boost::optional<View> eigenValues;
	boost::optional<View> eigenVectors;
	boost::optional<View> matchedValues;
	
	DataPoints output(input);
	Labels outputLabels;
	if (keepNormals)
		outputLabels.push_back(Label("normals", dimNormals));
	if (keepDensities)
		outputLabels.push_back(Label("densities", dimDensities));
	if (keepEigenValues)
		outputLabels.push_back(Label("eigValues", dimEigValues));
	if (keepEigenVectors)
		outputLabels.push_back(Label("eigVectors", dimEigVectors));
	output.allocateDescriptors(outputLabels);
	
	if (keepNormals)
		normals = output.getDescriptorViewByName("normals");
	if (keepDensities)
		densities = output.getDescriptorViewByName("densities");
	if (keepEigenValues)
		eigenValues = output.getDescriptorViewByName("eigValues");
	if (keepEigenVectors)
		eigenVectors = output.getDescriptorViewByName("eigVectors");
	// TODO: implement keepMatchedIds
// 	if (keepMatchedIds)
// 	{
// 		output.allocateDescriptor("normals", dimMatchedIds);
// 		matchedValues = output.getDescriptorViewByName("normals");
// 	}
	
	// Build kd-tree
	KDTreeMatcher matcher(Parameters({
		{ "knn", toParam(knn) },
		{ "epsilon", toParam(epsilon) }
	}));
	matcher.init(input);

	Matches matches(typename Matches::Dists(knn, pointsCount), typename Matches::Ids(knn, pointsCount));
	matches = matcher.findClosests(input);
	
	// Search for surrounding points and compute descriptors
	int degenerateCount(0);
	for (int i = 0; i < pointsCount; ++i)
	{
		// Mean of nearest neighbors (NN)
		Matrix d(featDim-1, knn);
		for(int j = 0; j < int(knn); j++)
		{
			const int refIndex(matches.ids(j,i));
			d.col(j) = input.features.block(0, refIndex, featDim-1, 1);
		}

		const Vector mean = d.rowwise().sum() / T(knn);
		const Matrix NN = d.colwise() - mean;
		
		const Matrix C(NN * NN.transpose());
		Vector eigenVa = Vector::Identity(featDim-1, 1);
		Matrix eigenVe = Matrix::Identity(featDim-1, featDim-1);
		// Ensure that the matrix is suited for eigenvalues calculation
		if(keepNormals || keepEigenValues || keepEigenVectors)
		{
			if(C.fullPivHouseholderQr().rank()+1 >= featDim-1)
			{
				const Eigen::EigenSolver<Matrix> solver(C);
				eigenVa = solver.eigenvalues().real();
				eigenVe = solver.eigenvectors().real();
			}
			else
			{
				//std::cout << "WARNING: Matrix C needed for eigen decomposition is degenerated. Expected cause: no noise in data" << std::endl;
				++degenerateCount;
			}
		}
		
		if(keepNormals)
			normals->col(i) = computeNormal(eigenVa, eigenVe);
		if(keepDensities)
			(*densities)(0, i) = computeDensity(NN);
		if(keepEigenValues)
			eigenValues->col(i) = eigenVa;
		if(keepEigenVectors)
			eigenVectors->col(i) = serializeEigVec(eigenVe);
	}
	if (degenerateCount)
	{
		LOG_WARNING_STREAM("WARNING: Matrix C needed for eigen decomposition was degenerated in " << degenerateCount << " points over " << pointsCount << " (" << float(degenerateCount)*100.f/float(pointsCount) << " %)");
	}
	
	return output;
}

template<typename T>
typename PointMatcher<T>::Vector DataPointsFiltersImpl<T>::SurfaceNormalDataPointsFilter::computeNormal(const Vector eigenVa, const Matrix eigenVe)
{
	// Keep the smallest eigenvector as surface normal
	int smallestId(0);
	T smallestValue(numeric_limits<T>::max());
	for(int j = 0; j < eigenVe.cols(); j++)
	{
		if (eigenVa(j) < smallestValue)
		{
			smallestId = j;
			smallestValue = eigenVa(j);
		}
	}
	
	return eigenVe.col(smallestId);
}

template<typename T>
typename PointMatcher<T>::Vector DataPointsFiltersImpl<T>::SurfaceNormalDataPointsFilter::serializeEigVec(const Matrix eigenVe)
{
	// serialize row major
	const int eigenVeDim = eigenVe.cols();
	Vector output(eigenVeDim*eigenVeDim);
	for(int k=0; k < eigenVe.cols(); k++)
	{
		output.segment(k*eigenVeDim, eigenVeDim) = 
			eigenVe.row(k).transpose();
	}

	return output;
}

template<typename T>
T DataPointsFiltersImpl<T>::SurfaceNormalDataPointsFilter::computeDensity(const Matrix NN)
{
	//volume in meter
	T volume = (4./3.)*M_PI*std::pow(NN.colwise().norm().maxCoeff(), 3);

	//volume in decimeter
	//T volume = (4./3.)*M_PI*std::pow(NN.colwise().norm().maxCoeff()*10.0, 3);
	//const T minVolume = 4.18e-9; // minimum of volume of one millimeter radius
	//const T minVolume = 0.42; // minimum of volume of one centimeter radius (in dm^3)

	//if(volume < minVolume) 		
	//	volume = minVolume;
		
	return T(NN.cols())/(volume);
}

template struct DataPointsFiltersImpl<float>::SurfaceNormalDataPointsFilter;
template struct DataPointsFiltersImpl<double>::SurfaceNormalDataPointsFilter;


// SamplingSurfaceNormalDataPointsFilter

// Constructor
template<typename T>
DataPointsFiltersImpl<T>::SamplingSurfaceNormalDataPointsFilter::SamplingSurfaceNormalDataPointsFilter(const Parameters& params):
	DataPointsFilter("SamplingSurfaceNormalDataPointsFilter", SamplingSurfaceNormalDataPointsFilter::availableParameters(), params),
	ratio(Parametrizable::get<T>("ratio")),
	knn(Parametrizable::get<int>("knn")),
	samplingMethod(Parametrizable::get<int>("samplingMethod")),
	maxBoxDim(Parametrizable::get<T>("maxBoxDim")),
	averageExistingDescriptors(Parametrizable::get<bool>("averageExistingDescriptors")),
	keepNormals(Parametrizable::get<bool>("keepNormals")),
	keepDensities(Parametrizable::get<bool>("keepDensities")),
	keepEigenValues(Parametrizable::get<bool>("keepEigenValues")),
	keepEigenVectors(Parametrizable::get<bool>("keepEigenVectors"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::SamplingSurfaceNormalDataPointsFilter::filter(
	const DataPoints& input)
{
	typedef Matrix Features;
	typedef typename DataPoints::View View;
	typedef typename DataPoints::Label Label;
	typedef typename DataPoints::Labels Labels;
	
	const int pointsCount(input.features.cols());
	const int featDim(input.features.rows());
	const int descDim(input.descriptors.rows());
	
	int insertDim(0);
	if (averageExistingDescriptors)
	{
		// TODO: this should be in the form of an assert
		// Validate descriptors and labels
		for(unsigned int i = 0; i < input.descriptorLabels.size(); i++)
			insertDim += input.descriptorLabels[i].span;
		if (insertDim != descDim)
			throw InvalidField("SamplingSurfaceNormalDataPointsFilter: Error, descriptor labels do not match descriptor data");
	}
	
	// Compute space requirement for new descriptors
	const int dimNormals(featDim-1);
	const int dimDensities(1);
	const int dimEigValues(featDim-1);
	const int dimEigVectors((featDim-1)*(featDim-1));
	
	// Build output cloud and allocate space for new descriptors
	DataPoints output(input.featureLabels, input.descriptorLabels, input.features.cols());
	Labels outputLabels;
	if (keepNormals)
		outputLabels.push_back(Label("normals", dimNormals));
	if (keepDensities)
		outputLabels.push_back(Label("densities", dimDensities));
	if (keepEigenValues)
		outputLabels.push_back(Label("eigValues", dimEigValues));
	if (keepEigenVectors)
		outputLabels.push_back(Label("eigVectors", dimEigVectors));
	output.allocateDescriptors(outputLabels);
	
	// we keep build data on stack for reentrant behaviour
	View outputExistingDescriptors(output.descriptors.block(0,0,input.descriptors.rows(),output.descriptors.cols()));
	BuildData buildData(input.features, input.descriptors, output.features, outputExistingDescriptors);

	// get views
	if (keepNormals)
		buildData.normals = output.getDescriptorViewByName("normals");
	if (keepDensities)
		buildData.densities = output.getDescriptorViewByName("densities");
	if (keepEigenValues)
		buildData.eigenValues = output.getDescriptorViewByName("eigValues");
	if (keepEigenVectors)
		buildData.eigenVectors = output.getDescriptorViewByName("eigVectors");

	// build the new point cloud
	buildNew(
		buildData,
		0,
		pointsCount, 
		input.features.rowwise().minCoeff(),
		input.features.rowwise().maxCoeff()
	);
	
	// resize point cloud to trimmed unused space
	const int ptsOut = buildData.outputInsertionPoint;
	output.features.conservativeResize(Eigen::NoChange, ptsOut);
	output.descriptors.conservativeResize(Eigen::NoChange, ptsOut);

	// warning if some points were dropped
	if(buildData.unfitPointsCount != 0)
		LOG_INFO_STREAM("  SamplingSurfaceNormalDataPointsFilter - Could not compute normal for " << buildData.unfitPointsCount << " pts.");
	
	// return the new point cloud
	return output;
}

template<typename T>
size_t argMax(const typename PointMatcher<T>::Vector& v)
{
	T maxVal(0);
	size_t maxIdx(0);
	for (int i = 0; i < v.size(); ++i)
	{
		if (v[i] > maxVal)
		{
			maxVal = v[i];
			maxIdx = i;
		}
	}
	return maxIdx;
}

template<typename T>
void DataPointsFiltersImpl<T>::SamplingSurfaceNormalDataPointsFilter::buildNew(BuildData& data, const int first, const int last, const Vector minValues, const Vector maxValues) const
{
	const int count(last - first);
	if (count <= int(knn))
	{
		// compute for this range
		fuseRange(data, first, last);
		// TODO: make another filter that creates constant-density clouds,
		// typically by stopping recursion after the median of the bounding cuboid
		// is below a threshold, or that the number of points falls under a threshold
		return;
	}
	
	// find the largest dimension of the box
	const int cutDim = argMax<T>(maxValues - minValues);
	
	// compute number of elements
	const int rightCount(count/2);
	const int leftCount(count - rightCount);
	assert(last - rightCount == first + leftCount);
	
	// sort, hack std::nth_element
	std::nth_element(
		data.indices.begin() + first,
		data.indices.begin() + first + leftCount,
		data.indices.begin() + last,
		CompareDim(cutDim, data)
	);
	
	// get value
	const int cutIndex(data.indices[first+leftCount]);
	const T cutVal(data.inputFeatures(cutDim, cutIndex));
	
	// update bounds for left
	Vector leftMaxValues(maxValues);
	leftMaxValues[cutDim] = cutVal;
	// update bounds for right
	Vector rightMinValues(minValues);
	rightMinValues[cutDim] = cutVal;
	
	// recurse
	buildNew(data, first, first + leftCount, minValues, leftMaxValues);
	buildNew(data, first + leftCount, last, rightMinValues, maxValues);
}

template<typename T>
void DataPointsFiltersImpl<T>::SamplingSurfaceNormalDataPointsFilter::fuseRange(BuildData& data, const int first, const int last) const
{
	const int colCount(last-first);
	const int featDim(data.inputFeatures.rows());
	assert(featDim == data.outputFeatures.rows());
	
	// build nearest neighbors list
	Matrix d(featDim-1, colCount);
	for (int i = 0; i < colCount; ++i)
		d.col(i) = data.inputFeatures.block(0,data.indices[first+i],featDim-1, 1);
	const Vector box = d.rowwise().maxCoeff() - d.rowwise().minCoeff();
	const T boxDim(box.maxCoeff());
	// drop box if it is too large
	if (boxDim > maxBoxDim)
	{
		data.unfitPointsCount += colCount;
		return;
	}
	const Vector mean = d.rowwise().sum() / T(colCount);
	const Matrix NN = (d.colwise() - mean);
	
	// compute covariance
	const Matrix C(NN * NN.transpose());
	Vector eigenVa = Vector::Identity(featDim-1, 1);
	Matrix eigenVe = Matrix::Identity(featDim-1, featDim-1);
	// Ensure that the matrix is suited for eigenvalues calculation
	if(keepNormals || keepEigenValues || keepEigenVectors)
	{
		if(C.fullPivHouseholderQr().rank()+1 >= featDim-1)
		{
			const Eigen::EigenSolver<Matrix> solver(C);
			eigenVa = solver.eigenvalues().real();
			eigenVe = solver.eigenvectors().real();
		}
		else
		{
			data.unfitPointsCount += colCount;
			return;
		}
	}

	Vector normal;
	if(keepNormals)
		normal = SurfaceNormalDataPointsFilter::computeNormal(eigenVa, eigenVe);

	T densitie = 0;
	if(keepDensities)
		densitie = SurfaceNormalDataPointsFilter::computeDensity(NN);	
	
	//if(keepEigenValues) nothing to do
	
	Vector serialEigVector;
	if(keepEigenVectors)
		serialEigVector = SurfaceNormalDataPointsFilter::serializeEigVec(eigenVe);
	
	// some safety check
	if(data.inputDescriptors.rows() != 0)
		assert(data.inputDescriptors.cols() != 0);
	
	// Filter points randomly
	if(samplingMethod == 0)
	{
		for(int i=0; i<colCount; i++)
		{
			const float r = (float)std::rand()/(float)RAND_MAX;
			if(r < ratio)
			{
				// Keep points with their descriptors
				data.outputFeatures.col(data.outputInsertionPoint) = 
					data.inputFeatures.col(data.indices[first+i]);
				
				if(data.inputDescriptors.rows() != 0)
				{
					data.outputExistingDescriptors.col(data.outputInsertionPoint) = 
						data.inputDescriptors.col(data.indices[first+i]);
				}

				// Build new descriptors in parallel to be merged at the end
				if(keepNormals)
					data.normals->col(data.outputInsertionPoint) = normal;
				if(keepDensities)
					(*data.densities)(0,data.outputInsertionPoint) = densitie;
				if(keepEigenValues)
					data.eigenValues->col(data.outputInsertionPoint) = eigenVa;
				if(keepEigenVectors)
					data.eigenVectors->col(data.outputInsertionPoint) = serialEigVector;

				++data.outputInsertionPoint;
			}
		}
	}
	else
	{
		data.outputFeatures.col(data.outputInsertionPoint).topRows(featDim-1) = mean;
		data.outputFeatures(featDim-1, data.outputInsertionPoint) = 1;
		
		if(data.inputDescriptors.rows() != 0)
		{
			// average the existing descriptors
			if (averageExistingDescriptors)
			{
				Vector mergedDesc(Vector::Zero(data.inputDescriptors.rows()));
				for (int i = 0; i < colCount; ++i)
					mergedDesc += data.inputDescriptors.col(data.indices[first+i]);
				mergedDesc /= T(colCount);
				data.outputExistingDescriptors.col(data.outputInsertionPoint) = mergedDesc;
			}
			else // just take the first one
				data.outputExistingDescriptors.col(data.outputInsertionPoint) = 
					data.inputDescriptors.col(data.indices[first]);
		}
		
		// Build new descriptor in paralelle to be merge at the end
		if(keepNormals)
			data.normals->col(data.outputInsertionPoint) = normal;
		if(keepDensities)
			(*data.densities)(0,data.outputInsertionPoint) = densitie;
		if(keepEigenValues)
			data.eigenValues->col(data.outputInsertionPoint) = eigenVa;
		if(keepEigenVectors)
			data.eigenVectors->col(data.outputInsertionPoint) = serialEigVector;
		
		++data.outputInsertionPoint;
	}

}

template struct DataPointsFiltersImpl<float>::SamplingSurfaceNormalDataPointsFilter;
template struct DataPointsFiltersImpl<double>::SamplingSurfaceNormalDataPointsFilter;


// OrientNormalsDataPointsFilter
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::OrientNormalsDataPointsFilter::OrientNormalsDataPointsFilter(const Parameters& params):
	DataPointsFilter("OrientNormalsDataPointsFilter", OrientNormalsDataPointsFilter::availableParameters(), params),
	towardCenter(Parametrizable::get<bool>("towardCenter"))
{
}

// OrientNormalsDataPointsFilter
// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::OrientNormalsDataPointsFilter::filter(const DataPoints& input)
{
	if (!input.descriptorExists("normals"))
		throw InvalidField("OrientNormalsDataPointsFilter: Error, cannot find normals in descriptors.");
	if (!input.descriptorExists("observationDirections"))
		throw InvalidField("OrientNormalsDataPointsFilter: Error, cannot find observation directions in descriptors.");
	
	DataPoints output(input);
	auto normals(output.getDescriptorViewByName("normals"));
	const auto observationDirections(output.getDescriptorViewByName("observationDirections"));
	assert(normals.rows() == observationDirections.rows());
	for (int i = 0; i < input.features.cols(); i++)
	{
		// Check normal orientation
		const Vector vecP = observationDirections.col(i);
		const Vector vecN = normals.col(i);
		const double scalar = vecP.dot(vecN);

		// Swap normal
		if(towardCenter)
		{
			if (scalar < 0)
				normals.col(i) = -vecN;
		}
		else
		{
			if (scalar > 0)
				normals.col(i) = -vecN;
		}
	}

	return output;
}

template struct DataPointsFiltersImpl<float>::OrientNormalsDataPointsFilter;
template struct DataPointsFiltersImpl<double>::OrientNormalsDataPointsFilter;


// RandomSamplingDataPointsFilter
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::RandomSamplingDataPointsFilter::RandomSamplingDataPointsFilter(const Parameters& params):
	DataPointsFilter("RandomSamplingDataPointsFilter", RandomSamplingDataPointsFilter::availableParameters(), params),
	prob(Parametrizable::get<double>("prob"))
{
}

// Constructor
template<typename T>
DataPointsFiltersImpl<T>::RandomSamplingDataPointsFilter::RandomSamplingDataPointsFilter(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params):
	DataPointsFilter(className, paramsDoc, params),
	prob(Parametrizable::get<double>("prob"))
{
}

// Sampling
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::RandomSamplingDataPointsFilter::randomSample(const DataPoints& input) const
{
	const int nbPointsIn = input.features.cols();
	
	DataPoints output(input.createSimilarEmpty());
	
	int j = 0;
	for (int i = 0; i < nbPointsIn; i++)
	{
		const float r = (float)std::rand()/(float)RAND_MAX;
		if (r < prob)
		{
			output.setColFrom(j, input, i);
			j++;
		}
	}
	
	output.conservativeResize(j);
	
	return output;
}

// filter
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::RandomSamplingDataPointsFilter::filter(const DataPoints& input)
{
	return randomSample(input);
}

template struct DataPointsFiltersImpl<float>::RandomSamplingDataPointsFilter;
template struct DataPointsFiltersImpl<double>::RandomSamplingDataPointsFilter;


// MaxPointCountDataPointsFilter
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::MaxPointCountDataPointsFilter::MaxPointCountDataPointsFilter(const Parameters& params):
	RandomSamplingDataPointsFilter("MaxPointCountDataPointsFilter", MaxPointCountDataPointsFilter::availableParameters(), params),
	maxCount(Parametrizable::get<unsigned>("maxCount"))
{
}

// filter
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::MaxPointCountDataPointsFilter::filter(const DataPoints& input)
{
	if (unsigned(input.features.cols()) <= maxCount)
		return input;
	return RandomSamplingDataPointsFilter::filter(input);
}

template struct DataPointsFiltersImpl<float>::MaxPointCountDataPointsFilter;
template struct DataPointsFiltersImpl<double>::MaxPointCountDataPointsFilter;


// FixStepSamplingDataPointsFilter
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::FixStepSamplingDataPointsFilter::FixStepSamplingDataPointsFilter(const Parameters& params):
	DataPointsFilter("FixStepSamplingDataPointsFilter", FixStepSamplingDataPointsFilter::availableParameters(), params),
	startStep(Parametrizable::get<unsigned>("startStep")),
	endStep(Parametrizable::get<unsigned>("endStep")),
	stepMult(Parametrizable::get<double>("stepMult")),
	step(startStep)
{
	LOG_INFO_STREAM("Using FixStepSamplingDataPointsFilter with startStep=" << startStep << ", endStep=" << endStep << ", stepMult=" << stepMult); 
}


template<typename T>
void DataPointsFiltersImpl<T>::FixStepSamplingDataPointsFilter::init()
{
	step = startStep;
}

// Sampling
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::FixStepSamplingDataPointsFilter::fixstepSample(const DataPoints& input)
{
	const int iStep(step);
	const int nbPointsIn = input.features.cols();
	const int phase(rand() % iStep);
	
	DataPoints output(input.createSimilarEmpty());
	
	int j = 0;
	for (int i = phase; i < nbPointsIn; i += iStep)
	{
		output.setColFrom(j, input, i);
		j++;
	}
	
	output.conservativeResize(j);
	
	const double deltaStep(startStep * stepMult - startStep);
	step *= stepMult;
	if (deltaStep < 0 && step < endStep)
		step = endStep;
	if (deltaStep > 0 && step > endStep)
		step = endStep;
	
	return output;
}

// Pre filter
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::FixStepSamplingDataPointsFilter::filter(const DataPoints& input)
{
	return fixstepSample(input);
}

template struct DataPointsFiltersImpl<float>::FixStepSamplingDataPointsFilter;
template struct DataPointsFiltersImpl<double>::FixStepSamplingDataPointsFilter;


// ShadowDataPointsFilter
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::ShadowDataPointsFilter::ShadowDataPointsFilter(const Parameters& params):
	DataPointsFilter("ShadowDataPointsFilter", ShadowDataPointsFilter::availableParameters(), params),
	eps(sin(Parametrizable::get<T>("eps")))
{
	//waring: maxAngle is change to sin(maxAngle)!
}


template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::ShadowDataPointsFilter::filter(
	const DataPoints& input)
{
	// Check if normals are present
	if (!input.descriptorExists("normals"))
	{
		throw InvalidField("ShadowDataPointsFilter, Error: cannot find normals in descriptors");
	}
	
	const int dim = input.features.rows();
	DataPoints output(input.createSimilarEmpty());
	
	const auto normals(input.getDescriptorViewByName("normals"));
	int j = 0;

	for(int i=0; i < input.features.cols(); i++)
	{
		const Vector normal = normals.col(i).normalized();
		const Vector point = input.features.block(0, i, dim-1, 1).normalized();
		
		const T value = anyabs(normal.dot(point));

		if(value > eps) // test to keep the points
		{
			output.features.col(j) = input.features.col(i);
			output.descriptors.col(j) = input.descriptors.col(i);
			
			j++;
		}
	}
	
	output.conservativeResize(j);

	return output;
}

template struct DataPointsFiltersImpl<float>::ShadowDataPointsFilter;
template struct DataPointsFiltersImpl<double>::ShadowDataPointsFilter;


// SimpleSensorNoiseDataPointsFilter 
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::SimpleSensorNoiseDataPointsFilter::SimpleSensorNoiseDataPointsFilter(const Parameters& params):
	DataPointsFilter("SimpleSensorNoiseDataPointsFilter", SimpleSensorNoiseDataPointsFilter::availableParameters(), params),
	sensorType(Parametrizable::get<unsigned>("sensorType")),
	gain(Parametrizable::get<T>("gain"))
{
	std::vector<string> sensorNames = {"Sick LMS-1xx", "Hokuyo URG-04LX", "Hokuyo UTM-30LX", "Kinect / Xtion"};
	if (sensorType >= sensorNames.size())
	{
		throw InvalidParameter(
			(boost::format("SimpleSensorNoiseDataPointsFilter: Error, sensorType id %1% does not exist.") % sensorType).str());
	}

	LOG_INFO_STREAM("SimpleSensorNoiseDataPointsFilter - using sensor noise model: " << sensorNames[sensorType]);
}



template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::SimpleSensorNoiseDataPointsFilter::filter(
	const DataPoints& input)
{
	
	DataPoints outputCloud(input);
	outputCloud.allocateDescriptor("simpleSensorNoise", 1);
	auto noise(outputCloud.getDescriptorViewByName("simpleSensorNoise"));

	switch(sensorType)
	{
	case 0: // Sick LMS-1xx
	{
		noise = computeLaserNoise(0.012, 0.0068, 0.0008, input.features);
		break;
	}
	case 1: // Hokuyo URG-04LX
	{
		noise = computeLaserNoise(0.028, 0.0013, 0.0001, input.features);
		break;
	}
	case 2: // Hokuyo UTM-30LX
	{
		noise = computeLaserNoise(0.018, 0.0006, 0.0015, input.features);
		break;
	}
	case 3: // Kinect / Xtion
	{
		const int dim = input.features.rows();
		noise = 0.5*0.00285*input.features.topRows(dim-1).colwise().norm().array().square();
		break;
	}
	default:
		throw InvalidParameter(
			(boost::format("SimpleSensorNoiseDataPointsFilter: Error, cannot compute noise for sensorType id %1% .") % sensorType).str());
	}

	return outputCloud;
}

template<typename T>
typename PointMatcher<T>::Matrix DataPointsFiltersImpl<T>::SimpleSensorNoiseDataPointsFilter::computeLaserNoise(const T minRadius, const T beamAngle, const T beamConst, const Matrix features)
{
	typedef typename Eigen::Array<T, 2, Eigen::Dynamic> Array2rows;
	
	const int nbPoints = features.cols();
	const int dim = features.rows();
	
	Array2rows evalNoise = Array2rows::Constant(2, nbPoints, minRadius);
	evalNoise.row(0) =  beamAngle * features.topRows(dim-1).colwise().norm();
	evalNoise.row(0) += beamConst;

	return evalNoise.colwise().maxCoeff();

}


template struct DataPointsFiltersImpl<float>::SimpleSensorNoiseDataPointsFilter;
template struct DataPointsFiltersImpl<double>::SimpleSensorNoiseDataPointsFilter;


// ObservationDirectionDataPointsFilter
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::ObservationDirectionDataPointsFilter::ObservationDirectionDataPointsFilter(const Parameters& params):
	DataPointsFilter("ObservationDirectionDataPointsFilter", ObservationDirectionDataPointsFilter::availableParameters(), params),
	centerX(Parametrizable::get<T>("x")),
	centerY(Parametrizable::get<T>("y")),
	centerZ(Parametrizable::get<T>("z"))
{
}

template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::ObservationDirectionDataPointsFilter::filter(const DataPoints& input)
{
	const int dim(input.features.rows() - 1);
	if (dim != 2 && dim != 3)
	{
		throw InvalidField(
			(boost::format("ObservationDirectionDataPointsFilter: Error, works only in 2 or 3 dimensions, cloud has %1% dimensions.") % dim).str()
		);
	}
	
	Vector center(dim);
	center[0] = centerX;
	center[1] = centerY;
	if (dim == 3)
		center[2] = centerZ;
	
	DataPoints outputCloud(input);
	outputCloud.allocateDescriptor("observationDirections", dim);
	auto observationDirections(outputCloud.getDescriptorViewByName("observationDirections"));
	
	for (int i = 0; i < input.features.cols(); i++)
	{
		// Check normal orientation
		const Vector p(input.features.block(0, i, dim, 1));
		observationDirections.col(i) = center - p;
	}
	
	return outputCloud;
}

template struct DataPointsFiltersImpl<float>::ObservationDirectionDataPointsFilter;
template struct DataPointsFiltersImpl<double>::ObservationDirectionDataPointsFilter;

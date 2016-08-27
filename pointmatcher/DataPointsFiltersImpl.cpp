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
#include "IO.h"
#include "MatchersImpl.h"
#include "Functions.h"

#include <algorithm>
#include <boost/format.hpp>

// Eigenvalues
#include "Eigen/QR"
#include "Eigen/Eigenvalues"

using namespace std;
using namespace PointMatcherSupport;

// IdentityDataPointsFilter
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::IdentityDataPointsFilter::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::IdentityDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
}

template struct DataPointsFiltersImpl<float>::IdentityDataPointsFilter;
template struct DataPointsFiltersImpl<double>::IdentityDataPointsFilter;


// RemoveNaNDataPointsFilter
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::RemoveNaNDataPointsFilter::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::RemoveNaNDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
	const int nbPointsIn = cloud.features.cols();

	int j = 0;
	for (int i = 0; i < nbPointsIn; ++i)
	{
		const BOOST_AUTO(colArray, cloud.features.col(i).array());
		const BOOST_AUTO(hasNaN, !(colArray == colArray).all());
		if (!hasNaN)
		{
			cloud.setColFrom(j, cloud, i);
			j++;
		}
	}

	cloud.conservativeResize(j);
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

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::MaxDistDataPointsFilter::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::MaxDistDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
	if (dim >= cloud.features.rows() - 1)
	{
		throw InvalidParameter(
			(boost::format("MaxDistDataPointsFilter: Error, filtering on dimension number %1%, larger than authorized axis id %2%") % dim % (cloud.features.rows() - 2)).str());
	}

	const int nbPointsIn = cloud.features.cols();
	const int nbRows = cloud.features.rows();

	int j = 0;
	if(dim == -1) // Euclidean distance
	{
		for (int i = 0; i < nbPointsIn; i++)
		{
			const T absMaxDist = anyabs(maxDist);
			if (cloud.features.col(i).head(nbRows-1).norm() < absMaxDist)
			{
				cloud.setColFrom(j, cloud, i);
				j++;
			}
		}
	}
	else // Single-axis distance
	{
		for (int i = 0; i < nbPointsIn; i++)
		{
			if ((cloud.features(dim, i)) < maxDist)
			{
				cloud.setColFrom(j, cloud, i);
				j++;
			}
		}
	}

	cloud.conservativeResize(j);
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

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::MinDistDataPointsFilter::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::MinDistDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
	if (dim >= cloud.features.rows() - 1)
		throw InvalidParameter((boost::format("MinDistDataPointsFilter: Error, filtering on dimension number %1%, larger than feature dimensionality %2%") % dim % (cloud.features.rows() - 2)).str());

	const int nbPointsIn = cloud.features.cols();
	const int nbRows = cloud.features.rows();

	int j = 0;
	if(dim == -1) // Euclidean distance
	{
		const T absMinDist = anyabs(minDist);
		for (int i = 0; i < nbPointsIn; i++)
		{
			if (cloud.features.col(i).head(nbRows-1).norm() > absMinDist)
			{
				cloud.setColFrom(j, cloud, i);
				j++;
			}
		}
	}
	else // Single axis distance
	{
		for (int i = 0; i < nbPointsIn; i++)
		{
			if ((cloud.features(dim, i)) > minDist)
			{
				cloud.setColFrom(j, cloud, i);
				j++;
			}
		}
	}

	cloud.conservativeResize(j);

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

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::BoundingBoxDataPointsFilter::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::BoundingBoxDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
	const int nbPointsIn = cloud.features.cols();
	const int nbRows = cloud.features.rows();

	int j = 0;
	for (int i = 0; i < nbPointsIn; i++)
	{
		bool keepPt = false;
		const Vector point = cloud.features.col(i);

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
			cloud.setColFrom(j, cloud, i);
			j++;
		}
	}

	cloud.conservativeResize(j);
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

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::MaxQuantileOnAxisDataPointsFilter::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::MaxQuantileOnAxisDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
	if (int(dim) >= cloud.features.rows())
		throw InvalidParameter((boost::format("MaxQuantileOnAxisDataPointsFilter: Error, filtering on dimension number %1%, larger than feature dimensionality %2%") % dim % cloud.features.rows()).str());

	const int nbPointsIn = cloud.features.cols();
	const int nbPointsOut = nbPointsIn * ratio;

	// build array
	vector<T> values;
	values.reserve(cloud.features.cols());
	for (int x = 0; x < cloud.features.cols(); ++x)
		values.push_back(cloud.features(dim, x));

	// get quartiles value
	nth_element(values.begin(), values.begin() + (values.size() * ratio), values.end());
	const T limit = values[nbPointsOut];

	// copy towards beginning the elements we keep
	int j = 0;
	for (int i = 0; i < nbPointsIn; i++)
	{
		if (cloud.features(dim, i) < limit)
		{
			assert(j <= i);
			cloud.setColFrom(j, cloud, i);
			j++;
		}
	}
	assert(j <= nbPointsOut);

	cloud.conservativeResize(j);

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

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::MaxDensityDataPointsFilter::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::MaxDensityDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
	typedef typename DataPoints::View View;

	// Force densities to be computed
	if (!cloud.descriptorExists("densities"))
	{
		throw InvalidField("MaxDensityDataPointsFilter: Error, no densities found in descriptors.");
	}

	const int nbPointsIn = cloud.features.cols();
	View densities = cloud.getDescriptorViewByName("densities");
	const T lastDensity = densities.maxCoeff();
	const int nbSaturatedPts = (densities.array() == lastDensity).count();

	// fill cloud values
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
				cloud.setColFrom(j, cloud, i);
				j++;
			}
		}
		else
		{
			cloud.setColFrom(j, cloud, i);
			j++;
		}
	}

	cloud.conservativeResize(j);
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
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::SurfaceNormalDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
	typedef typename DataPoints::View View;
	typedef typename DataPoints::Label Label;
	typedef typename DataPoints::Labels Labels;
	typedef typename MatchersImpl<T>::KDTreeMatcher KDTreeMatcher;
	typedef typename PointMatcher<T>::Matches Matches;

	const int pointsCount(cloud.features.cols());
	const int featDim(cloud.features.rows());
	const int descDim(cloud.descriptors.rows());

	// Validate descriptors and labels
	int insertDim(0);
	for(unsigned int i = 0; i < cloud.descriptorLabels.size(); i++)
		insertDim += cloud.descriptorLabels[i].span;
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

	Labels cloudLabels;
	if (keepNormals)
		cloudLabels.push_back(Label("normals", dimNormals));
	if (keepDensities)
		cloudLabels.push_back(Label("densities", dimDensities));
	if (keepEigenValues)
		cloudLabels.push_back(Label("eigValues", dimEigValues));
	if (keepEigenVectors)
		cloudLabels.push_back(Label("eigVectors", dimEigVectors));
	cloud.allocateDescriptors(cloudLabels);

	if (keepNormals)
		normals = cloud.getDescriptorViewByName("normals");
	if (keepDensities)
		densities = cloud.getDescriptorViewByName("densities");
	if (keepEigenValues)
		eigenValues = cloud.getDescriptorViewByName("eigValues");
	if (keepEigenVectors)
		eigenVectors = cloud.getDescriptorViewByName("eigVectors");
	// TODO: implement keepMatchedIds
//	if (keepMatchedIds)
//	{
//		cloud.allocateDescriptor("normals", dimMatchedIds);
//		matchedValues = cloud.getDescriptorViewByName("normals");
//	}

	// Build kd-tree
	Parametrizable::Parameters param;
	boost::assign::insert(param) ( "knn", toParam(knn) );
	boost::assign::insert(param) ( "epsilon", toParam(epsilon) );
	KDTreeMatcher matcher(param);
	matcher.init(cloud);

	Matches matches(typename Matches::Dists(knn, pointsCount), typename Matches::Ids(knn, pointsCount));
	matches = matcher.findClosests(cloud);

	// Search for surrounding points and compute descriptors
	int degenerateCount(0);
	for (int i = 0; i < pointsCount; ++i)
	{
		// Mean of nearest neighbors (NN)
		Matrix d(featDim-1, knn);
		for(int j = 0; j < int(knn); j++)
		{
			const int refIndex(matches.ids(j,i));
			d.col(j) = cloud.features.block(0, refIndex, featDim-1, 1);
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
typename PointMatcher<T>::Vector DataPointsFiltersImpl<T>::SurfaceNormalDataPointsFilter::sortEigenValues(const Vector& eigenVa)
{
	// sort the eigenvalues in ascending order
	Vector eigenVaSort = eigenVa;
	std::sort(eigenVaSort.data(), eigenVaSort.data() + eigenVaSort.size());
	return eigenVaSort;
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
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::SamplingSurfaceNormalDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
	typedef typename DataPoints::View View;
	typedef typename DataPoints::Label Label;
	typedef typename DataPoints::Labels Labels;

	const int pointsCount(cloud.features.cols());
	const int featDim(cloud.features.rows());
	const int descDim(cloud.descriptors.rows());

	int insertDim(0);
	if (averageExistingDescriptors)
	{
		// TODO: this should be in the form of an assert
		// Validate descriptors and labels
		for(unsigned int i = 0; i < cloud.descriptorLabels.size(); i++)
			insertDim += cloud.descriptorLabels[i].span;
		if (insertDim != descDim)
			throw InvalidField("SamplingSurfaceNormalDataPointsFilter: Error, descriptor labels do not match descriptor data");
	}

	// Compute space requirement for new descriptors
	const int dimNormals(featDim-1);
	const int dimDensities(1);
	const int dimEigValues(featDim-1);
	const int dimEigVectors((featDim-1)*(featDim-1));

	// Allocate space for new descriptors
	Labels cloudLabels;
	if (keepNormals)
		cloudLabels.push_back(Label("normals", dimNormals));
	if (keepDensities)
		cloudLabels.push_back(Label("densities", dimDensities));
	if (keepEigenValues)
		cloudLabels.push_back(Label("eigValues", dimEigValues));
	if (keepEigenVectors)
		cloudLabels.push_back(Label("eigVectors", dimEigVectors));
	cloud.allocateDescriptors(cloudLabels);

	// we keep build data on stack for reentrant behaviour
	View cloudExistingDescriptors(cloud.descriptors.block(0,0,cloud.descriptors.rows(),cloud.descriptors.cols()));
	BuildData buildData(cloud.features, cloud.descriptors);

	// get views
	if (keepNormals)
		buildData.normals = cloud.getDescriptorViewByName("normals");
	if (keepDensities)
		buildData.densities = cloud.getDescriptorViewByName("densities");
	if (keepEigenValues)
		buildData.eigenValues = cloud.getDescriptorViewByName("eigValues");
	if (keepEigenVectors)
		buildData.eigenVectors = cloud.getDescriptorViewByName("eigVectors");
	// build the new point cloud
	buildNew(
		buildData,
		0,
		pointsCount,
		cloud.features.rowwise().minCoeff(),
		cloud.features.rowwise().maxCoeff()
	);

	// Bring the data we keep to the front of the arrays then
	// wipe the leftover unused space.
	std::sort(buildData.indicesToKeep.begin(), buildData.indicesToKeep.end());
	int ptsOut = buildData.indicesToKeep.size();
	for (int i = 0; i < ptsOut; i++){
		int k = buildData.indicesToKeep[i];
		assert(i <= k);
		cloud.features.col(i) = cloud.features.col(k);
		if (cloud.descriptors.rows() != 0)
			cloud.descriptors.col(i) = cloud.descriptors.col(k);
		if(keepNormals)
			buildData.normals->col(i) = buildData.normals->col(k);
		if(keepDensities)
			(*buildData.densities)(0,i) = (*buildData.densities)(0,k);
		if(keepEigenValues)
			buildData.eigenValues->col(i) = buildData.eigenValues->col(k);
		if(keepEigenVectors)
			buildData.eigenVectors->col(i) = buildData.eigenVectors->col(k);
	}
	cloud.features.conservativeResize(Eigen::NoChange, ptsOut);
	cloud.descriptors.conservativeResize(Eigen::NoChange, ptsOut);

	// warning if some points were dropped
	if(buildData.unfitPointsCount != 0)
		LOG_INFO_STREAM("  SamplingSurfaceNormalDataPointsFilter - Could not compute normal for " << buildData.unfitPointsCount << " pts.");
}

template<typename T>
size_t argMax(const typename PointMatcher<T>::Vector& v)
{
	//FIXME: Change that to use the new API. the new Eigen API (3.2.8) allows this with the call maxCoeff. See the section Visitors in https://eigen.tuxfamily.org/dox/group__TutorialReductionsVisitorsBroadcasting.html
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
	const T cutVal(data.features(cutDim, cutIndex));

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
	const int featDim(data.features.rows());

	// build nearest neighbors list
	Matrix d(featDim-1, colCount);
	for (int i = 0; i < colCount; ++i)
		d.col(i) = data.features.block(0,data.indices[first+i],featDim-1, 1);
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
	if(data.descriptors.rows() != 0)
		assert(data.descriptors.cols() != 0);

	// Filter points randomly
	if(samplingMethod == 0)
	{
		for(int i=0; i<colCount; i++)
		{
			const float r = (float)std::rand()/(float)RAND_MAX;
			if(r < ratio)
			{
				// Keep points with their descriptors
				int k = data.indices[first+i];
				// Mark the indices which will be part of the final data
				data.indicesToKeep.push_back(k);

				// Build new descriptors
				if(keepNormals)
					data.normals->col(k) = normal;
				if(keepDensities)
					(*data.densities)(0,k) = densitie;
				if(keepEigenValues)
					data.eigenValues->col(k) = eigenVa;
				if(keepEigenVectors)
					data.eigenVectors->col(k) = serialEigVector;

			}
		}
	}
	else
	{

		int k = data.indices[first];
		// Mark the indices which will be part of the final data
		data.indicesToKeep.push_back(k);
		data.features.col(k).topRows(featDim-1) = mean;
		data.features(featDim-1, k) = 1;

		if(data.descriptors.rows() != 0)
		{
			// average the existing descriptors
			if (averageExistingDescriptors)
			{
				Vector mergedDesc(Vector::Zero(data.descriptors.rows()));
				for (int i = 0; i < colCount; ++i)
					mergedDesc += data.descriptors.col(data.indices[first+i]);
				mergedDesc /= T(colCount);
				data.descriptors.col(k) = mergedDesc;
			}
			// else just keep the first one
		}

		// Build new descriptors
		if(keepNormals)
			data.normals->col(k) = normal;
		if(keepDensities)
			(*data.densities)(0,k) = densitie;
		if(keepEigenValues)
			data.eigenValues->col(k) = eigenVa;
		if(keepEigenVectors)
			data.eigenVectors->col(k) = serialEigVector;

	}

}

template struct DataPointsFiltersImpl<float>::SamplingSurfaceNormalDataPointsFilter;
template struct DataPointsFiltersImpl<double>::SamplingSurfaceNormalDataPointsFilter;

//////////////////////////////////////////////////////////////////////////////////////

// ElipsoidsDataPointsFilter

// Constructor
template<typename T>
DataPointsFiltersImpl<T>::ElipsoidsDataPointsFilter::ElipsoidsDataPointsFilter(const Parameters& params):
DataPointsFilter("ElipsoidsDataPointsFilter", ElipsoidsDataPointsFilter::availableParameters(), params),
ratio(Parametrizable::get<T>("ratio")),
knn(Parametrizable::get<int>("knn")),
samplingMethod(Parametrizable::get<int>("samplingMethod")),
maxBoxDim(Parametrizable::get<T>("maxBoxDim")),
maxTimeWindow(Parametrizable::get<T>("maxTimeWindow")),
minPlanarity(Parametrizable::get<T>("minPlanarity")),
averageExistingDescriptors(Parametrizable::get<bool>("averageExistingDescriptors")),
keepNormals(Parametrizable::get<bool>("keepNormals")),
keepDensities(Parametrizable::get<bool>("keepDensities")),
keepEigenValues(Parametrizable::get<bool>("keepEigenValues")),
keepEigenVectors(Parametrizable::get<bool>("keepEigenVectors")),
keepCovariances(Parametrizable::get<bool>("keepCovariances")),
keepWeights(Parametrizable::get<bool>("keepWeights")),
keepMeans(Parametrizable::get<bool>("keepMeans")),
keepShapes(Parametrizable::get<bool>("keepShapes")),
keepIndices(Parametrizable::get<bool>("keepIndices"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::ElipsoidsDataPointsFilter::filter(
    const DataPoints& input)
{
  DataPoints output(input);
  inPlaceFilter(output);
  return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::ElipsoidsDataPointsFilter::inPlaceFilter(
    DataPoints& cloud)
{
  typedef typename DataPoints::View View;
  typedef typename DataPoints::Label Label;
  typedef typename DataPoints::Labels Labels;
  typedef typename DataPoints::TimeView TimeView;

  const int pointsCount(cloud.features.cols());
  const int featDim(cloud.features.rows());
  const int descDim(cloud.descriptors.rows());

  int insertDim(0);
  if (averageExistingDescriptors)
  {
    // TODO: this should be in the form of an assert
    // Validate descriptors and labels
    for(unsigned int i = 0; i < cloud.descriptorLabels.size(); i++)
      insertDim += cloud.descriptorLabels[i].span;
    if (insertDim != descDim)
      throw InvalidField("ElipsoidsDataPointsFilter: Error, descriptor labels do not match descriptor data");
  }

  // Compute space requirement for new descriptors
  const int dimNormals(featDim-1);
  const int dimDensities(1);
  const int dimEigValues(featDim-1);
  const int dimEigVectors((featDim-1)*(featDim-1));
  const int dimWeights(1);
  const int dimMeans(featDim-1);
  const int dimCovariances((featDim-1)*(featDim-1));
  const int dimShapes(featDim-1);
  const int dimPointIds(knn);

  // Allocate space for new descriptors
  Labels cloudLabels, timeLabels;
  if (keepIndices) {
    cloudLabels.push_back(Label("pointIds", dimPointIds));
    cloudLabels.push_back(Label("pointX", dimPointIds));
    cloudLabels.push_back(Label("pointY", dimPointIds));
    cloudLabels.push_back(Label("pointZ", dimPointIds));
    cloudLabels.push_back(Label("numOfNN", 1));
  }
  if (keepNormals)
    cloudLabels.push_back(Label("normals", dimNormals));
  if (keepDensities)
    cloudLabels.push_back(Label("densities", dimDensities));
  if (keepEigenValues)
    cloudLabels.push_back(Label("eigValues", dimEigValues));
  if (keepEigenVectors)
    cloudLabels.push_back(Label("eigVectors", dimEigVectors));
  if (keepCovariances)
    cloudLabels.push_back(Label("covariance", dimCovariances));
  if (keepWeights)
    cloudLabels.push_back(Label("weights", dimWeights));
  if (keepMeans)
    cloudLabels.push_back(Label("means", dimMeans));
  if (keepShapes) {
    assert(featDim == 3);
    cloudLabels.push_back(Label("shapes", dimShapes)); // Planarity, Cylindricality, Sphericality
  }
  timeLabels.push_back(Label("time", 2));

  cloud.allocateDescriptors(cloudLabels);
  cloud.allocateTimes(timeLabels);

  // we keep build data on stack for reentrant behaviour
  View cloudExistingDescriptors(cloud.descriptors.block(0,0,cloud.descriptors.rows(),cloud.descriptors.cols()));
  TimeView cloudExistingTimes(cloud.times.block(0,0,cloud.times.rows(),cloud.times.cols()));
  BuildData buildData(cloud.features, cloud.descriptors, cloud.times);

  // get views
  if (keepIndices) {
    buildData.pointIds = cloud.getDescriptorViewByName("pointIds");
    buildData.pointX = cloud.getDescriptorViewByName("pointX");
    buildData.pointY = cloud.getDescriptorViewByName("pointY");
    buildData.pointZ = cloud.getDescriptorViewByName("pointZ");
    buildData.numOfNN = cloud.getDescriptorViewByName("numOfNN");
  }
  if (keepNormals)
    buildData.normals = cloud.getDescriptorViewByName("normals");
  if (keepDensities)
    buildData.densities = cloud.getDescriptorViewByName("densities");
  if (keepEigenValues)
    buildData.eigenValues = cloud.getDescriptorViewByName("eigValues");
  if (keepEigenVectors)
    buildData.eigenVectors = cloud.getDescriptorViewByName("eigVectors");
  if (keepCovariances)
    buildData.covariance = cloud.getDescriptorViewByName("covariance");
  if (keepWeights)
    buildData.weights = cloud.getDescriptorViewByName("weights");
  if (keepMeans)
    buildData.means = cloud.getDescriptorViewByName("means");
  if (keepShapes)
    buildData.shapes = cloud.getDescriptorViewByName("shapes");

  // build the new point cloud
  buildNew(
      buildData,
      0,
      pointsCount,
      cloud.features.rowwise().minCoeff(),
      cloud.features.rowwise().maxCoeff()
  );

  // Bring the data we keep to the front of the arrays then
  // wipe the leftover unused space.
  std::sort(buildData.indicesToKeep.begin(), buildData.indicesToKeep.end());
  int ptsOut = buildData.indicesToKeep.size();
  for (int i = 0; i < ptsOut; i++){
    int k = buildData.indicesToKeep[i];
    assert(i <= k);
    cloud.features.col(i) = cloud.features.col(k);
    cloud.times.col(i) = cloud.times.col(k);
    if (cloud.descriptors.rows() != 0)
      cloud.descriptors.col(i) = cloud.descriptors.col(k);
    if(keepIndices) {
      buildData.pointIds->col(i) = buildData.pointIds->col(k);
      buildData.pointX->col(i) = buildData.pointX->col(k);
      buildData.pointY->col(i) = buildData.pointY->col(k);
      buildData.pointZ->col(i) = buildData.pointZ->col(k);
      buildData.numOfNN->col(i) = buildData.numOfNN->col(k);
    }
    if(keepNormals)
      buildData.normals->col(i) = buildData.normals->col(k);
    if(keepDensities)
      (*buildData.densities)(0,i) = (*buildData.densities)(0,k);
    if(keepEigenValues)
      buildData.eigenValues->col(i) = buildData.eigenValues->col(k);
    if(keepEigenVectors)
      buildData.eigenVectors->col(i) = buildData.eigenVectors->col(k);
    if(keepWeights)
      buildData.weights->col(i) = buildData.weights->col(k);
    if(keepCovariances)
      buildData.covariance->col(i) = buildData.covariance->col(k);
    if(keepMeans)
      buildData.means->col(i) = buildData.means->col(k);
    if(keepShapes)
      buildData.shapes->col(i) = buildData.shapes->col(k);
  }
  cloud.features.conservativeResize(Eigen::NoChange, ptsOut);
  cloud.descriptors.conservativeResize(Eigen::NoChange, ptsOut);
  cloud.times.conservativeResize(Eigen::NoChange, ptsOut);

  // warning if some points were dropped
  if(buildData.unfitPointsCount != 0)
    LOG_INFO_STREAM("  ElipsoidsDataPointsFilter - Could not compute normal for " << buildData.unfitPointsCount << " pts.");
}

template<typename T>
void DataPointsFiltersImpl<T>::ElipsoidsDataPointsFilter::buildNew(BuildData& data, const int first, const int last, const Vector minValues, const Vector maxValues) const
{
  const int count(last - first);
  if (count <= int(knn))
  {
    // compute for this range
    fuseRange(data, first, last);
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
  const T cutVal(data.features(cutDim, cutIndex));

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
void DataPointsFiltersImpl<T>::ElipsoidsDataPointsFilter::fuseRange(BuildData& data, const int first, const int last) const
{
  typedef typename Eigen::Matrix<boost::int64_t, Eigen::Dynamic, Eigen::Dynamic> Int64Matrix;

  const int colCount(last-first);
  const int featDim(data.features.rows());

  // build nearest neighbors list
  Matrix d(featDim-1, colCount);
  Int64Matrix t(1, colCount);
  for (int i = 0; i < colCount; ++i) {
    d.col(i) = data.features.block(0,data.indices[first+i],featDim-1, 1);
    t.col(i) = data.times.col(data.indices[first + i]); //, 0);
  }
  const Vector box = d.rowwise().maxCoeff() - d.rowwise().minCoeff();
  const boost::int64_t timeBox = t.maxCoeff() - t.minCoeff();

  const T boxDim(box.maxCoeff());
  // drop box if it is too large or max timeframe is exceeded
  if (boxDim > maxBoxDim || timeBox > maxTimeWindow)
  {
    data.unfitPointsCount += colCount;
    return;
  }
  const Vector mean = d.rowwise().sum() / T(colCount);
  const Matrix NN = (d.colwise() - mean);

  boost::int64_t minTime = t.minCoeff();
  boost::int64_t maxTime = t.maxCoeff();
  boost::int64_t meanTime = t.sum() / T(colCount);

  // compute covariance
  const Matrix C(NN * NN.transpose());
  Vector eigenVa = Vector::Identity(featDim-1, 1);
  Matrix eigenVe = Matrix::Identity(featDim-1, featDim-1);
  // Ensure that the matrix is suited for eigenvalues calculation
  if(keepNormals || keepEigenValues || keepEigenVectors || keepCovariances || keepShapes || minPlanarity > 0)
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
    if(minPlanarity > 0 ) {
      Eigen::Matrix<T, 3, 1> vals;
      (vals << eigenVa(0),eigenVa(1),eigenVa(2));
      vals = vals/eigenVa.sum();
      T planarity = 2 * vals(1)-2*vals(2);
      // throw out surfel if it does not meet planarity criteria
      if (planarity < minPlanarity)
      {
        data.unfitPointsCount += colCount;
        return;
      }
    }
  }

  // keep the indices of each ellipsoid
  Vector pointIds(1,colCount);
  Matrix points(3,colCount);

  if(keepIndices) {
    for (int i = 0; i < colCount; ++i) {
      pointIds(i) = data.indices[first+i];
      points.col(i) = data.features.block(0,data.indices[first+i],2, 1);
    }
  }

  Vector normal;
  if(keepNormals)
    normal = SurfaceNormalDataPointsFilter::computeNormal(eigenVa, eigenVe);

  T density = 0;
  if(keepDensities)
    density = SurfaceNormalDataPointsFilter::computeDensity(NN);
  Vector serialEigVector;
  if(keepEigenVectors)
    serialEigVector = SurfaceNormalDataPointsFilter::serializeEigVec(eigenVe);
  Vector serialCovVector;
  if(keepCovariances)
    serialCovVector = SurfaceNormalDataPointsFilter::serializeEigVec(C);

  // some safety check
  if(data.descriptors.rows() != 0)
    assert(data.descriptors.cols() != 0);

  // Filter points randomly
  if(samplingMethod == 0)
  {

    for(int i=0; i<colCount; i++)
    {
      const float r = (float)std::rand()/(float)RAND_MAX;
      if(r < ratio)
      {
        // Keep points with their descriptors
        int k = data.indices[first+i];
        // Mark the indices which will be part of the final data
        data.indicesToKeep.push_back(k);

        // write the updated times: min, max, mean
        data.times(0, k) = minTime;
        data.times(1, k) = maxTime;
        data.times(2, k) = meanTime;

        // Build new descriptors
        if(keepIndices) {
          data.pointIds->col(k) = pointIds;
          data.pointX->col(k) = points.row(0);
          data.pointY->col(k) = points.row(1);
          data.pointZ->col(k) = points.row(2);
          (*data.numOfNN)(0,k) = NN.cols();
        }
        if(keepNormals)
          data.normals->col(k) = normal;
        if(keepDensities)
          (*data.densities)(0,k) = density;
        if(keepEigenValues)
          data.eigenValues->col(k) = eigenVa;
        if(keepEigenVectors)
          data.eigenVectors->col(k) = serialEigVector;
        if(keepCovariances)
          data.covariance->col(k) = serialCovVector;
        if(keepMeans)
          data.means->col(k) = mean;
        // a 3d vecetor of shape parameters: planarity (P), cylindricality (C), sphericality (S)
        if(keepShapes) {
          Eigen::Matrix<T, 3, 3> shapeMat;
          (shapeMat << 0, 2, -2, 1, -1, 0, 0, 0, 3);
          Eigen::Matrix<T, 3, 1> vals;
          (vals << eigenVa(0),eigenVa(1),eigenVa(2));
          vals = vals/eigenVa.sum();
          data.shapes->col(k) = shapeMat * vals;

        }
        if(keepWeights) {
          (*data.weights)(0,k) = colCount;
        }
      }
    }
  }
  else
  {

    int k = data.indices[first];
    // Mark the indices which will be part of the final data
    data.indicesToKeep.push_back(k);
    data.features.col(k).topRows(featDim-1) = mean;
    // write the updated times: min, max, mean
    data.times(0, k) = minTime;
    data.times(1, k) = maxTime;
    data.times(2, k) = meanTime;

    data.features(featDim-1, k) = 1;

    if(data.descriptors.rows() != 0)
    {
      // average the existing descriptors
      if (averageExistingDescriptors)
      {
        Vector mergedDesc(Vector::Zero(data.descriptors.rows()));
        for (int i = 0; i < colCount; ++i)
          mergedDesc += data.descriptors.col(data.indices[first+i]);
        mergedDesc /= T(colCount);
        data.descriptors.col(k) = mergedDesc;
      }
      // else just keep the first one
    }

    // Build new descriptors
    if(keepIndices) {
      data.pointIds->col(k) = pointIds;
      data.pointX->col(k) = points.row(0);
      data.pointY->col(k) = points.row(1);
      data.pointZ->col(k) = points.row(2);
    }
    if(keepNormals)
      data.normals->col(k) = normal;
    if(keepDensities)
      (*data.densities)(0,k) = density;
    if(keepEigenValues)
      data.eigenValues->col(k) = eigenVa;
    if(keepEigenVectors)
      data.eigenVectors->col(k) = serialEigVector;
    if(keepCovariances)
      data.covariance->col(k) = serialCovVector;
    if(keepMeans)
      data.means->col(k) = mean;
    if(keepShapes) {
      Eigen::Matrix<T, 3, 3> shapeMat;
      (shapeMat << 0, 2, -2, 1, -1, 0, 0, 0, 3);
      Eigen::Matrix<T, 3, 1> vals;
      (vals << eigenVa(0),eigenVa(1),eigenVa(2));
      vals = vals/eigenVa.sum();
      data.shapes->col(k) = shapeMat * vals; //eigenVa;
    }
    if(keepWeights)
      (*data.weights)(0,k) = colCount;
  }

}

template struct DataPointsFiltersImpl<float>::ElipsoidsDataPointsFilter;
template struct DataPointsFiltersImpl<double>::ElipsoidsDataPointsFilter;

//////////////////////////////////////////////////////////////////////////////////////

// GestaltDataPointsFilter

// Constructor
template<typename T>
DataPointsFiltersImpl<T>::GestaltDataPointsFilter::GestaltDataPointsFilter(const Parameters& params):
DataPointsFilter("GestaltDataPointsFilter", GestaltDataPointsFilter::availableParameters(), params),
ratio(Parametrizable::get<T>("ratio")),
radius(Parametrizable::get<T>("radius")),
knn(Parametrizable::get<int>("knn")),
vSizeX(Parametrizable::get<T>("vSizeX")),
vSizeY(Parametrizable::get<T>("vSizeY")),
vSizeZ(Parametrizable::get<T>("vSizeZ")),
maxBoxDim(Parametrizable::get<T>("maxBoxDim")),
maxTimeWindow(Parametrizable::get<T>("maxTimeWindow")),
keepMeans(Parametrizable::get<bool>("keepMeans")),
averageExistingDescriptors(Parametrizable::get<bool>("averageExistingDescriptors")),
keepNormals(Parametrizable::get<bool>("keepNormals")),
keepEigenValues(Parametrizable::get<bool>("keepEigenValues")),
keepEigenVectors(Parametrizable::get<bool>("keepEigenVectors")),
keepCovariances(Parametrizable::get<bool>("keepCovariances")),
keepGestaltFeatures(Parametrizable::get<bool>("keepGestaltFeatures"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::GestaltDataPointsFilter::filter(
    const DataPoints& input)
{
  DataPoints output(input);
  inPlaceFilter(output);
  return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::GestaltDataPointsFilter::inPlaceFilter(
    DataPoints& cloud)
{
  typedef typename DataPoints::View View;
  typedef typename DataPoints::Label Label;
  typedef typename DataPoints::Labels Labels;
  typedef typename DataPoints::TimeView TimeView;

  const int pointsCount(cloud.features.cols());
  const int featDim(cloud.features.rows());
  const int descDim(cloud.descriptors.rows());

  int insertDim(0);
  if (averageExistingDescriptors)
  {
    // TODO: this should be in the form of an assert
    // Validate descriptors and labels
    for(unsigned int i = 0; i < cloud.descriptorLabels.size(); i++)
      insertDim += cloud.descriptorLabels[i].span;
    if (insertDim != descDim)
      throw InvalidField("GestaltDataPointsFilter: Error, descriptor labels do not match descriptor data");
  }

  // Compute space requirement for new descriptors
  const int dimNormals(featDim-1);
  const int dimMeans(featDim-1);
  const int dimEigValues(featDim-1);
  const int dimEigVectors((featDim-1)*(featDim-1));
  const int dimCovariances((featDim-1)*(featDim-1));
  const int dimGestalt = 32;

  // Allocate space for new descriptors
  Labels cloudLabels, timeLabels;

  if (keepNormals)
    cloudLabels.push_back(Label("normals", dimNormals));
  if (keepMeans)
    cloudLabels.push_back(Label("means", dimMeans));
  if (keepEigenValues)
    cloudLabels.push_back(Label("eigValues", dimEigValues));
  if (keepEigenVectors)
    cloudLabels.push_back(Label("eigVectors", dimEigVectors));
  if (keepCovariances)
    cloudLabels.push_back(Label("covariance", dimCovariances));
  if (keepGestaltFeatures) {
    cloudLabels.push_back(Label("gestaltMeans", dimGestalt));
    cloudLabels.push_back(Label("gestaltVariances", dimGestalt));
    cloudLabels.push_back(Label("warpedXYZ", 3));
    cloudLabels.push_back(Label("gestaltShapes", 2));
  }
  timeLabels.push_back(Label("time", 3));

  cloud.allocateDescriptors(cloudLabels);
  cloud.allocateTimes(timeLabels);

  // we keep build data on stack for reentrant behaviour
  View cloudExistingDescriptors(cloud.descriptors.block(0,0,cloud.descriptors.rows(),cloud.descriptors.cols()));
  TimeView cloudExistingTimes(cloud.times.block(0,0,cloud.times.rows(),cloud.times.cols()));
  BuildData buildData(cloud.features, cloud.descriptors, cloud.times);

  // get views
  if (keepNormals)
    buildData.normals = cloud.getDescriptorViewByName("normals");
  if(keepMeans)
    buildData.means = cloud.getDescriptorViewByName("means");
  if (keepEigenValues)
    buildData.eigenValues = cloud.getDescriptorViewByName("eigValues");
  if (keepEigenVectors)
    buildData.eigenVectors = cloud.getDescriptorViewByName("eigVectors");
  if (keepCovariances)
    buildData.covariance = cloud.getDescriptorViewByName("covariance");
  if (keepGestaltFeatures) {
    buildData.gestaltMeans = cloud.getDescriptorViewByName("gestaltMeans");
    buildData.gestaltVariances = cloud.getDescriptorViewByName("gestaltVariances");
    buildData.warpedXYZ = cloud.getDescriptorViewByName("warpedXYZ");
    buildData.gestaltShapes = cloud.getDescriptorViewByName("gestaltShapes");
  }
  // build the new point cloud
  buildNew(
      buildData,
      0,
      pointsCount,
      cloud.features.rowwise().minCoeff(),
      cloud.features.rowwise().maxCoeff()
  );

  // buildData.indicesToKeep contains all the indices where we want Gestalt features at
  fuseRange(buildData, cloud, 0, pointsCount);

  // Bring the data we keep to the front of the arrays then
  // wipe the leftover unused space.
  std::sort(buildData.indicesToKeep.begin(), buildData.indicesToKeep.end());
  int ptsOut = buildData.indicesToKeep.size();
  for (int i = 0; i < ptsOut; i++){
    int k = buildData.indicesToKeep[i];
    assert(i <= k);
    cloud.features.col(i) = cloud.features.col(k);
    cloud.times.col(i) = cloud.times.col(k);
    if (cloud.descriptors.rows() != 0)
      cloud.descriptors.col(i) = cloud.descriptors.col(k);
    if(keepNormals)
      buildData.normals->col(i) = buildData.normals->col(k);
    if(keepMeans)
      buildData.means->col(i) = buildData.means->col(k);
    if(keepEigenValues)
      buildData.eigenValues->col(i) = buildData.eigenValues->col(k);
    if(keepEigenVectors)
      buildData.eigenVectors->col(i) = buildData.eigenVectors->col(k);
    if(keepCovariances)
      buildData.covariance->col(i) = buildData.covariance->col(k);
    if(keepGestaltFeatures) {
      buildData.gestaltMeans->col(i) = buildData.gestaltMeans->col(k);
      buildData.gestaltVariances->col(i) = buildData.gestaltVariances->col(k);
      buildData.warpedXYZ->col(i) = buildData.warpedXYZ->col(k);
      buildData.gestaltShapes->col(i) = buildData.gestaltShapes->col(k);
    }
  }
  cloud.features.conservativeResize(Eigen::NoChange, ptsOut);
  cloud.descriptors.conservativeResize(Eigen::NoChange, ptsOut);
  cloud.times.conservativeResize(Eigen::NoChange, ptsOut);
  // warning if some points were dropped
  if(buildData.unfitPointsCount != 0)
    LOG_INFO_STREAM("  GestaltDataPointsFilter - Could not compute normal for " << buildData.unfitPointsCount << " pts.");
}

template<typename T>
void DataPointsFiltersImpl<T>::GestaltDataPointsFilter::buildNew(BuildData& data, const int first, const int last, const Vector minValues, const Vector maxValues) const
{

  T minBoundX = minValues.x() / vSizeX;
  T maxBoundX = maxValues.x() / vSizeX;
  T minBoundY = minValues.y() / vSizeY;
  T maxBoundY = maxValues.y() / vSizeY;
  T minBoundZ = minValues.z() / vSizeZ;
  T maxBoundZ = maxValues.z() / vSizeZ;

  // number of divisions is total size / voxel size voxels of equal length + 1
  // with remaining space
  unsigned int numDivX = 1 + maxBoundX - minBoundX;
  unsigned int numDivY = 1 + maxBoundY - minBoundY;;
  unsigned int numDivZ = 1 + maxBoundZ - minBoundZ;
  unsigned int numVox = numDivX * numDivY * numDivZ;

  // Assume point cloud is randomly ordered
  // compute a linear index of the following type
  // i, j, k are the component indices
  // nx, ny number of divisions in x and y components
  // idx = i + j * nx + k * nx * ny
  int numPoints = last - first;
  std::vector<unsigned int> indices(numPoints);

  // vector to hold the first point in a voxel
  // this point will be ovewritten in the input cloud with
  // the output value
  std::vector<typename VoxelGridDataPointsFilter::Voxel>* voxels;

  // try allocating vector. If too big return error
  try {
    voxels = new std::vector<typename VoxelGridDataPointsFilter::Voxel>(numVox);
  } catch (std::bad_alloc&) {
    throw InvalidParameter((boost::format("GestaltDataPointsFilter: Memory allocation error with %1% voxels.  Try increasing the voxel dimensions.") % numVox).str());
  }

  const int featDim(data.features.rows());

  for (int p = 0; p < numPoints; p++ )
  {
    unsigned int i = floor(data.features(0,p)/vSizeX - minBoundX);
    unsigned int j = floor(data.features(1,p)/vSizeY- minBoundY);
    unsigned int k = 0;
    unsigned int idx;
    if ( featDim == 4 )
    {
      k = floor(data.features(2,p)/vSizeZ - minBoundZ);
      idx = i + j * numDivX + k * numDivX * numDivY;
    }
    else
    {
      idx = i + j * numDivX;
    }

    unsigned int pointsInVox = (*voxels)[idx].numPoints + 1;

    if (pointsInVox == 1)
    {
      (*voxels)[idx].firstPoint = p;
    }

    (*voxels)[idx].numPoints = pointsInVox;

    indices[p] = idx;

  }

  // store which points contain voxel position
  std::vector<unsigned int> pointsToKeep;

  // take centers of voxels for now
  // Todo revert to random point selection within cell
  for (int p = 0; p < numPoints ; p++)
  {
    unsigned int idx = indices[p];
    unsigned int firstPoint = (*voxels)[idx].firstPoint;

    // Choose random point in voxel
    int randomIndex = std::rand() % numPoints;
    for (int f = 0; f < (featDim - 1); f++ )
    {
      data.features(f,firstPoint) = data.features(f,randomIndex);
    }
  }

  for (int idx = 0; idx < numVox; idx++)
  {
    unsigned int numPoints = (*voxels)[idx].numPoints;
    unsigned int firstPoint = (*voxels)[idx].firstPoint;

    if (numPoints > 0)
    {
      // get back voxel indices in grid format
      // If we are in the last division, the voxel is smaller in size
      // We adjust the center as from the end of the last voxel to the bounding area

      pointsToKeep.push_back(firstPoint);
    }
  }

  delete voxels;

  // now the keypoints are in pointsToKeep
  // downsample with ratio
  for(int i=0; i<pointsToKeep.size(); i++)
  {
    const float r = (float)std::rand()/(float)RAND_MAX;
    if(r < ratio)
    {
      // Keep points with their descriptors
      int k = pointsToKeep[i];
      // Mark the indices which will be part of the final data
      data.indicesToKeep.push_back(k);
    }
  }
}

template<typename T>
void DataPointsFiltersImpl<T>::GestaltDataPointsFilter::fuseRange(BuildData& data, DataPoints& input, const int first, const int last) const
{
  typedef typename Eigen::Matrix<boost::int64_t, Eigen::Dynamic, Eigen::Dynamic> Int64Matrix;

  const int featDim(data.features.rows());
  std::vector<int> indicesToKeepStrict;
  for (int i = 0; i< data.indicesToKeep.size(); ++i) {
    Eigen::Matrix<T,3,1> keyPoint;
    keyPoint = input.features.col(data.indicesToKeep[i]);

    // Define a search box around each keypoint to search for nearest neighbours.
    T minBoundX = keyPoint(0,0) - radius;
    T maxBoundX = keyPoint(0,0) + radius;
    T minBoundY = keyPoint(1,0) - radius;
    T maxBoundY = keyPoint(1,0) + radius;
    T minBoundZ = keyPoint(2,0) - radius;
    T maxBoundZ = keyPoint(2,0) + radius;
    // iterate over data and find in- / outliers
    Eigen::Matrix<T,3,1> feature;
    std::vector<int> goodIndices;
    for (int j = 0; j < input.features.cols(); ++j) {
      feature = input.features.col(j);
      if(feature(0,0) <= maxBoundX && feature(0,0) >= minBoundX &&
          feature(1,0) <= maxBoundY && feature(1,0) >= minBoundY &&
          feature(2,0) <= maxBoundZ && feature(2,0) >= minBoundZ &&
          keyPoint != feature) {
        goodIndices.push_back(j);
      }
    }
    int colCount = goodIndices.size();
    // if empty neighbourhood unfit the point
    if (colCount == 0) {
      data.unfitPointsCount++;
      continue;
    }
    Matrix d(featDim-1, colCount);
    Int64Matrix t(1, colCount);

    for (int j = 0; j < colCount; ++j) {
      d.col(j) = data.features.block(0,data.indices[goodIndices[j]],featDim-1, 1);
      t.col(j) = data.times.col(data.indices[goodIndices[j]]);
    }

    const int featDim(data.features.rows());

    const Vector mean = d.rowwise().sum() / T(colCount);
    const Matrix NN = d.colwise() - mean;
    boost::int64_t minTime = t.minCoeff();
    boost::int64_t maxTime = t.maxCoeff();
    boost::int64_t meanTime = t.sum() / T(colCount);
    // compute covariance
    const Matrix C(NN * NN.transpose());
    Vector eigenVa = Vector::Identity(featDim-1, 1);
    Matrix eigenVe = Matrix::Identity(featDim-1, featDim-1);
    // Ensure that the matrix is suited for eigenvalues calculation
    if(keepNormals || keepEigenValues || keepEigenVectors || keepCovariances || keepGestaltFeatures)
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
        continue;
      }
    }
    Eigen::Matrix<T,3,1> normal, newX, newY;
    Eigen::Matrix<T,3,3> newBasis;
    double planarity, cylindricality;

    if(keepNormals || keepGestaltFeatures) {
      // calculate orientation of NN
      normal = SurfaceNormalDataPointsFilter::computeNormal(eigenVa, eigenVe);

      if(keepGestaltFeatures) {
        Vector eigenVaSort = SurfaceNormalDataPointsFilter::sortEigenValues(eigenVa);
        planarity = 2 * (eigenVaSort(1) - eigenVaSort(0))/eigenVaSort.sum();
        cylindricality = (eigenVaSort(2) - eigenVaSort(1))/eigenVaSort.sum();
        // project normal on horizontal plane
        Eigen::Matrix<T,3,1> up, base;
        up << 0,0,1;
        base << 1,0,0;
        newX << normal(0), normal(1), 0;
        newX.normalize();
        newY = up.cross(newX);
        newY = newY / newY.norm();
        // form a new basis with world z-axis and projected x & y-axis
        newBasis << newX(0), newY(0), up(0),
            newX(1), newY(1), up(1),
            newX(2), newY(2), up(2);

        // discard keypoints with high planarity
        if(planarity > 0.9) {
          data.unfitPointsCount += colCount;
          continue;
        }
        // discard keypoints with normal too close to vertical
        if(acos(normal.dot(up)) < abs(10 * M_PI/180)) {
          data.unfitPointsCount += colCount;
          continue;
        }

        // define features in new basis that is oriented with the covariance
        for (int j = 0; j < colCount; ++j) {
          data.warpedXYZ->col(j) = ((data.features.block(0,j,3,1) - keyPoint).transpose() * newBasis).transpose();
        }
      }
    }
    Vector angles(colCount), radii(colCount), heights(colCount);
    Matrix gestaltMeans(4, 8), gestaltVariances(4, 8), numOfValues(4, 8);
    if(keepGestaltFeatures) {

      // calculate the polar coordinates of points
      angles = GestaltDataPointsFilter::calculateAngles(*data.warpedXYZ, keyPoint);
      radii = GestaltDataPointsFilter::calculateRadii(*data.warpedXYZ, keyPoint);
      heights = data.warpedXYZ->row(2);

      // sort points into Gestalt bins
      T angularBinWidth = M_PI/4;
      T radialBinWidth = radius/4;
      Matrix indices(2, colCount);
      gestaltMeans = Matrix::Zero(4, 8);
      gestaltVariances = Matrix::Zero(4, 8);
      numOfValues = Matrix::Zero(4, 8);

      for (int it=0; it < colCount; ++it) {
        indices(0,it) = floor(radii(it)/radialBinWidth);
        // if value exceeds borders of bin -> put in outmost bin
        if(indices(0,it) > 3)
          // this case should never happen - just in case
          indices(0,it) = 3;
        indices(1,it) = floor(angles(it)/angularBinWidth);
        if(indices(1,it) > 7)
          indices(1,it) = 7;
        gestaltMeans(indices(0,it), indices(1,it)) += heights(it);
        numOfValues(indices(0,it), indices(1,it))++;
      }

      for (int radial=0; radial < 4; ++radial) {
        for (int angular = 0; angular < 8; ++angular) {
          if (numOfValues(radial, angular) > 0) {
            gestaltMeans(radial, angular) = gestaltMeans(radial, angular)/numOfValues(radial, angular);
          }
        }
      }
      for (int it=0; it < colCount; ++it) {
        gestaltVariances(indices(0,it), indices(1,it)) += (heights(it)-gestaltMeans(indices(0,it), indices(1,it))) * (heights(it)-gestaltMeans(indices(0,it), indices(1,it)));
      }
      for (int radial=0; radial < 4; ++radial) {
        for (int angular = 0; angular < 8; ++angular) {
          // if bins are == 0 -> propagate with value in bin closer to keypoint
          if (gestaltMeans(radial,angular) == 0 && radial > 0) {
            gestaltMeans(radial, angular) = gestaltMeans(radial-1, angular);
            gestaltVariances(radial, angular) = gestaltVariances(radial-1, angular);
          } else if (numOfValues(radial, angular) > 0) {
            gestaltVariances(radial, angular) = gestaltVariances(radial, angular)/numOfValues(radial, angular);
          }
        }
      }
    }
    Vector serialEigVector;
    if(keepEigenVectors)
      serialEigVector = SurfaceNormalDataPointsFilter::serializeEigVec(eigenVe);
    Vector serialCovVector;
    if(keepCovariances)
      serialCovVector = SurfaceNormalDataPointsFilter::serializeEigVec(C);
    Vector serialGestaltMeans;
    Vector serialGestaltVariances;
    if(keepGestaltFeatures) {
      serialGestaltMeans = GestaltDataPointsFilter::serializeGestaltMatrix(gestaltMeans);
      serialGestaltVariances = GestaltDataPointsFilter::serializeGestaltMatrix(gestaltVariances);
    }
    // some safety check
    if(data.descriptors.rows() != 0)
      assert(data.descriptors.cols() != 0);

    // write the updated times: min, max, mean
    data.times(0, data.indicesToKeep[i]) = minTime;
    data.times(1, data.indicesToKeep[i]) = maxTime;
    data.times(2, data.indicesToKeep[i]) = meanTime;

    // Build new descriptors
    if(keepNormals)
      data.normals->col(data.indicesToKeep[i]) = normal;
    if(keepMeans)
      data.means->col(data.indicesToKeep[i]) = mean;
    if(keepEigenValues)
      data.eigenValues->col(data.indicesToKeep[i]) = eigenVa;
    if(keepEigenVectors)
      data.eigenVectors->col(data.indicesToKeep[i]) = serialEigVector;
    if(keepCovariances)
      data.covariance->col(data.indicesToKeep[i]) = serialCovVector;
    if(keepGestaltFeatures) {
      // preserve gestalt features
      data.gestaltMeans->col(data.indicesToKeep[i]) = serialGestaltMeans;
      data.gestaltVariances->col(data.indicesToKeep[i]) = serialGestaltVariances;
      (*data.gestaltShapes)(0,data.indicesToKeep[i]) = planarity;
      (*data.gestaltShapes)(1,data.indicesToKeep[i]) = cylindricality;
    }
    // all went well so far - so keep this keypoint
    indicesToKeepStrict.push_back(data.indicesToKeep[i]);
  }
  data.indicesToKeep = indicesToKeepStrict;
}

template<typename T>
typename PointMatcher<T>::Vector DataPointsFiltersImpl<T>::GestaltDataPointsFilter::serializeGestaltMatrix(const Matrix gestaltFeatures) const
{
  // serialize the gestalt descriptors
  const int dim = gestaltFeatures.rows() * gestaltFeatures.cols();
  Vector output(dim);
  for(int k=0; k < gestaltFeatures.rows(); k++)
  {
    output.segment(k*gestaltFeatures.cols(), gestaltFeatures.cols()) =
        gestaltFeatures.row(k).transpose();
  }
  return output;
}

template<typename T>
typename PointMatcher<T>::Vector DataPointsFiltersImpl<T>::GestaltDataPointsFilter::calculateAngles(const Matrix points, const Eigen::Matrix<T,3,1> keyPoint) const
{
  Vector angles(points.cols());
  for (size_t i = 0; i<points.cols(); ++i) {
    angles(i) = atan2(points(0,i), points(1,i));
    if (angles(i) < 0)
      angles(i) += (2 * M_PI);
  }
  return angles;
}

template<typename T>
typename PointMatcher<T>::Vector DataPointsFiltersImpl<T>::GestaltDataPointsFilter::calculateRadii(const Matrix points, const Eigen::Matrix<T,3,1> keyPoint) const
{
  Vector radii(points.cols());
  for (size_t i = 0; i<points.cols(); ++i) {
    radii(i) = sqrt((points(0,i)) * (points(0,i)) + (points(1,i)) * (points(1,i)));
  }
  return radii;
}

template struct DataPointsFiltersImpl<float>::GestaltDataPointsFilter;
template struct DataPointsFiltersImpl<double>::GestaltDataPointsFilter;

/////////////////////////////////////////////////////////////////////////////////////

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
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::OrientNormalsDataPointsFilter::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::OrientNormalsDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
	if (!cloud.descriptorExists("normals"))
		throw InvalidField("OrientNormalsDataPointsFilter: Error, cannot find normals in descriptors.");
	if (!cloud.descriptorExists("observationDirections"))
		throw InvalidField("OrientNormalsDataPointsFilter: Error, cannot find observation directions in descriptors.");

	BOOST_AUTO(normals, cloud.getDescriptorViewByName("normals"));
	const BOOST_AUTO(observationDirections, cloud.getDescriptorViewByName("observationDirections"));
	assert(normals.rows() == observationDirections.rows());
	for (int i = 0; i < cloud.features.cols(); i++)
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

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::RandomSamplingDataPointsFilter::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::RandomSamplingDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
	const int nbPointsIn = cloud.features.cols();

	int j = 0;
	for (int i = 0; i < nbPointsIn; i++)
	{
		const float r = (float)std::rand()/(float)RAND_MAX;
		if (r < prob)
		{
			cloud.setColFrom(j, cloud, i);
			j++;
		}
	}

	cloud.conservativeResize(j);
}

template struct DataPointsFiltersImpl<float>::RandomSamplingDataPointsFilter;
template struct DataPointsFiltersImpl<double>::RandomSamplingDataPointsFilter;


// MaxPointCountDataPointsFilter
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::MaxPointCountDataPointsFilter::MaxPointCountDataPointsFilter(const Parameters& params):
	DataPointsFilter("MaxPointCountDataPointsFilter", MaxPointCountDataPointsFilter::availableParameters(), params),
	maxCount(Parametrizable::get<unsigned>("maxCount"))
{
	try {
		seed = Parametrizable::get<unsigned>("seed");
	} catch (const InvalidParameter& e) {
		seed = static_cast<unsigned int> (1); // rand default seed number
	}
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::MaxPointCountDataPointsFilter::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::MaxPointCountDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
	unsigned N = static_cast<unsigned> (cloud.features.cols());
	if (maxCount < N) {
		DataPoints cloud_filtered = cloud.createSimilarEmpty(maxCount);
		std::srand(seed);

		unsigned top = N - maxCount;
		unsigned i = 0;
		unsigned index = 0;
		for (size_t n = maxCount; n >= 2; n--)
		{
			float V = static_cast<float>(rand () / double (RAND_MAX));
			unsigned S = 0;
			float quot = static_cast<float> (top) / static_cast<float> (N);
			while (quot > V)
			{
				S++;
				top--;
				N--;
				quot = quot * static_cast<float> (top) / static_cast<float> (N);
			}
			index += S;
			cloud_filtered.setColFrom(i++, cloud, index++);
			N--;
		}

		index += N * static_cast<unsigned> (static_cast<float>(rand () / double (RAND_MAX)));
		cloud_filtered.setColFrom(i++, cloud, index++);
		PointMatcher<T>::swapDataPoints(cloud, cloud_filtered);
		cloud.conservativeResize(i);
	}
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

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::FixStepSamplingDataPointsFilter::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::FixStepSamplingDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
	const int iStep(step);
	const int nbPointsIn = cloud.features.cols();
	const int phase(rand() % iStep);

	int j = 0;
	for (int i = phase; i < nbPointsIn; i += iStep)
	{
		cloud.setColFrom(j, cloud, i);
		j++;
	}

	cloud.conservativeResize(j);

	const double deltaStep(startStep * stepMult - startStep);
	step *= stepMult;
	if (deltaStep < 0 && step < endStep)
		step = endStep;
	if (deltaStep > 0 && step > endStep)
		step = endStep;

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


// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::ShadowDataPointsFilter::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::ShadowDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
	// Check if normals are present
	if (!cloud.descriptorExists("normals"))
	{
		throw InvalidField("ShadowDataPointsFilter, Error: cannot find normals in descriptors");
	}

	const int dim = cloud.features.rows();

	const BOOST_AUTO(normals, cloud.getDescriptorViewByName("normals"));
	int j = 0;

	for(int i=0; i < cloud.features.cols(); i++)
	{
		const Vector normal = normals.col(i).normalized();
		const Vector point = cloud.features.block(0, i, dim-1, 1).normalized();

		const T value = anyabs(normal.dot(point));

		if(value > eps) // test to keep the points
		{
			cloud.features.col(j) = cloud.features.col(i);
			cloud.descriptors.col(j) = cloud.descriptors.col(i);
			j++;
		}
	}

	cloud.conservativeResize(j);
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
  std::vector<string> sensorNames = boost::assign::list_of ("Sick LMS-1xx")("Hokuyo URG-04LX")("Hokuyo UTM-30LX")("Kinect / Xtion")("Sick Tim3xx");
	if (sensorType >= sensorNames.size())
	{
		throw InvalidParameter(
			(boost::format("SimpleSensorNoiseDataPointsFilter: Error, sensorType id %1% does not exist.") % sensorType).str());
	}

	LOG_INFO_STREAM("SimpleSensorNoiseDataPointsFilter - using sensor noise model: " << sensorNames[sensorType]);
}


// SimpleSensorNoiseDataPointsFilter
// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::SimpleSensorNoiseDataPointsFilter::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::SimpleSensorNoiseDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
	cloud.allocateDescriptor("simpleSensorNoise", 1);
	BOOST_AUTO(noise, cloud.getDescriptorViewByName("simpleSensorNoise"));

	switch(sensorType)
	{
	case 0: // Sick LMS-1xx
	{
		noise = computeLaserNoise(0.012, 0.0068, 0.0008, cloud.features);
		break;
	}
	case 1: // Hokuyo URG-04LX
	{
		noise = computeLaserNoise(0.028, 0.0013, 0.0001, cloud.features);
		break;
	}
	case 2: // Hokuyo UTM-30LX
	{
		noise = computeLaserNoise(0.018, 0.0006, 0.0015, cloud.features);
		break;
	}
	case 3: // Kinect / Xtion
	{
		const int dim = cloud.features.rows();
		const Matrix squaredValues(cloud.features.topRows(dim-1).colwise().norm().array().square());
		noise = squaredValues*(0.5*0.00285);
		break;
	}
  case 4: // Sick Tim3xx
  {
    noise = computeLaserNoise(0.004, 0.0053, -0.0092, cloud.features);
    break;
  }
	default:
		throw InvalidParameter(
			(boost::format("SimpleSensorNoiseDataPointsFilter: Error, cannot compute noise for sensorType id %1% .") % sensorType).str());
	}

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

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::ObservationDirectionDataPointsFilter::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::ObservationDirectionDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
	const int dim(cloud.features.rows() - 1);
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

	cloud.allocateDescriptor("observationDirections", dim);
	BOOST_AUTO(observationDirections, cloud.getDescriptorViewByName("observationDirections"));

	for (int i = 0; i < cloud.features.cols(); i++)
	{
		// Check normal orientation
		const Vector p(cloud.features.block(0, i, dim, 1));
		observationDirections.col(i) = center - p;
	}

}

template struct DataPointsFiltersImpl<float>::ObservationDirectionDataPointsFilter;
template struct DataPointsFiltersImpl<double>::ObservationDirectionDataPointsFilter;

// VoxelGridDataPointsFilter
template <typename T>
DataPointsFiltersImpl<T>::VoxelGridDataPointsFilter::VoxelGridDataPointsFilter() :
	vSizeX(1),
	vSizeY(1),
	vSizeZ(1),
	useCentroid(true),
	averageExistingDescriptors(true) {}

template <typename T>
DataPointsFiltersImpl<T>::VoxelGridDataPointsFilter::VoxelGridDataPointsFilter(const Parameters& params) :
DataPointsFilter("VoxelGridDataPointsFilter", VoxelGridDataPointsFilter::availableParameters(), params),
		vSizeX(Parametrizable::get<T>("vSizeX")),
		vSizeY(Parametrizable::get<T>("vSizeY")),
		vSizeZ(Parametrizable::get<T>("vSizeZ")),
		useCentroid(Parametrizable::get<bool>("useCentroid")),
		averageExistingDescriptors(Parametrizable::get<bool>("averageExistingDescriptors"))
{

}

template <typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::VoxelGridDataPointsFilter::filter(const DataPoints& input)
{
    DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

template <typename T>
void DataPointsFiltersImpl<T>::VoxelGridDataPointsFilter::inPlaceFilter(DataPoints& cloud)
{
    const unsigned int numPoints(cloud.features.cols());
	const int featDim(cloud.features.rows());
	const int descDim(cloud.descriptors.rows());

	assert (featDim == 3 || featDim == 4);

	int insertDim(0);
	if (averageExistingDescriptors)
	{
		// TODO: this should be in the form of an assert
		// Validate descriptors and labels
		for(unsigned int i = 0; i < cloud.descriptorLabels.size(); i++)
			insertDim += cloud.descriptorLabels[i].span;
		if (insertDim != descDim)
			throw InvalidField("VoxelGridDataPointsFilter: Error, descriptor labels do not match descriptor data");
	}

	// TODO: Check that the voxel size is not too small, given the size of the data

	// Calculate number of divisions along each axis
	Vector minValues = cloud.features.rowwise().minCoeff();
	Vector maxValues = cloud.features.rowwise().maxCoeff();

    T minBoundX = minValues.x() / vSizeX;
    T maxBoundX = maxValues.x() / vSizeX;
    T minBoundY = minValues.y() / vSizeY;
    T maxBoundY = maxValues.y() / vSizeY;
    T minBoundZ = 0;
    T maxBoundZ = 0;

    if (featDim == 4)
    {
        minBoundZ = minValues.z() / vSizeZ;
        maxBoundZ = maxValues.z() / vSizeZ;
    }

    // number of divisions is total size / voxel size voxels of equal length + 1
    // with remaining space
    unsigned int numDivX = 1 + maxBoundX - minBoundX;
    unsigned int numDivY = 1 + maxBoundY - minBoundY;;
    unsigned int numDivZ = 0;

    // If a 3D point cloud
    if (featDim == 4 )
        numDivZ = 1 + maxBoundZ - minBoundZ;

    unsigned int numVox = numDivX * numDivY;
    if ( featDim == 4)
        numVox *= numDivZ;

    // Assume point cloud is randomly ordered
    // compute a linear index of the following type
    // i, j, k are the component indices
    // nx, ny number of divisions in x and y components
    // idx = i + j * nx + k * nx * ny
    std::vector<unsigned int> indices(numPoints);

    // vector to hold the first point in a voxel
    // this point will be ovewritten in the input cloud with
    // the output value

    std::vector<Voxel>* voxels;

    // try allocating vector. If too big return error
    try {
    	voxels = new std::vector<Voxel>(numVox);
    } catch (std::bad_alloc&) {
    	throw InvalidParameter((boost::format("VoxelGridDataPointsFilter: Memory allocation error with %1% voxels.  Try increasing the voxel dimensions.") % numVox).str());
    }


    for (unsigned int p = 0; p < numPoints; p++ )
    {
        unsigned int i = floor(cloud.features(0,p)/vSizeX - minBoundX);
        unsigned int j = floor(cloud.features(1,p)/vSizeY- minBoundY);
        unsigned int k = 0;
        unsigned int idx;
        if ( featDim == 4 )
        {
            k = floor(cloud.features(2,p)/vSizeZ - minBoundZ);
            idx = i + j * numDivX + k * numDivX * numDivY;
        }
        else
        {
            idx = i + j * numDivX;
        }

        unsigned int pointsInVox = (*voxels)[idx].numPoints + 1;

        if (pointsInVox == 1)
        {
            (*voxels)[idx].firstPoint = p;
        }

        (*voxels)[idx].numPoints = pointsInVox;

        indices[p] = idx;

    }

    // store which points contain voxel position
    std::vector<unsigned int> pointsToKeep;

    // Store voxel centroid in output
    if (useCentroid)
    {
        // Iterate through the indices and sum values to compute centroid
        for (unsigned int p = 0; p < numPoints ; p++)
        {
            unsigned int idx = indices[p];
            unsigned int firstPoint = (*voxels)[idx].firstPoint;

            // If this is the first point in the voxel, leave as is
            // if not sum up this point for centroid calculation
            if (firstPoint != p)
            {
            	// Sum up features and descriptors (if we are also averaging descriptors)

            	for (int f = 0; f < (featDim - 1); f++ )
            	{
            		cloud.features(f,firstPoint) += cloud.features(f,p);
            	}

            	if (averageExistingDescriptors) {
            		for (int d = 0; d < descDim; d++)
            		{
            			cloud.descriptors(d,firstPoint) += cloud.descriptors(d,p);
            		}
            	}
            }
        }

        // Now iterating through the voxels
        // Normalize sums to get centroid (average)
        // Some voxels may be empty and are discarded
        for(unsigned int idx = 0; idx < numVox; idx++)
        {
            unsigned int numPoints = (*voxels)[idx].numPoints;
            unsigned int firstPoint = (*voxels)[idx].firstPoint;
            if(numPoints > 0)
            {
                for ( int f = 0; f < (featDim - 1); f++ )
                    cloud.features(f,firstPoint) /= numPoints;

                if (averageExistingDescriptors) {
                	for ( int d = 0; d < descDim; d++ )
                		cloud.descriptors(d,firstPoint) /= numPoints;
                }

                pointsToKeep.push_back(firstPoint);
            }
        }
    }
    else
    {
    	// Although we don't sum over the features, we may still need to sum the descriptors
    	if (averageExistingDescriptors)
    	{
    		// Iterate through the indices and sum values to compute centroid
    		for (unsigned int p = 0; p < numPoints ; p++)
    		{
    			unsigned int idx = indices[p];
    			unsigned int firstPoint = (*voxels)[idx].firstPoint;

    			// If this is the first point in the voxel, leave as is
    			// if not sum up this point for centroid calculation
    			if (firstPoint != p)
    			{
    				for (int d = 0; d < descDim; d++ )
    				{
    					cloud.descriptors(d,firstPoint) += cloud.descriptors(d,p);
    				}
    			}
    		}
    	}

        for (unsigned int idx = 0; idx < numVox; idx++)
        {
            unsigned int numPoints = (*voxels)[idx].numPoints;
            unsigned int firstPoint = (*voxels)[idx].firstPoint;

            if (numPoints > 0)
            {
                // get back voxel indices in grid format
                // If we are in the last division, the voxel is smaller in size
                // We adjust the center as from the end of the last voxel to the bounding area
                unsigned int i = 0;
                unsigned int j = 0;
                unsigned int k = 0;
                if (featDim == 4)
                {
                    k = idx / (numDivX * numDivY);
                    if (k == numDivZ)
                        cloud.features(3,firstPoint) = maxValues.z() - (k-1) * vSizeZ/2;
                    else
                        cloud.features(3,firstPoint) = k * vSizeZ + vSizeZ/2;
                }

                j = (idx - k * numDivX * numDivY) / numDivX;
                if (j == numDivY)
                    cloud.features(2,firstPoint) = maxValues.y() - (j-1) * vSizeY/2;
                else
                    cloud.features(2,firstPoint) = j * vSizeY + vSizeY / 2;

                i = idx - k * numDivX * numDivY - j * numDivX;
                if (i == numDivX)
                    cloud.features(1,firstPoint) = maxValues.x() - (i-1) * vSizeX/2;
                else
                    cloud.features(1,firstPoint) = i * vSizeX + vSizeX / 2;

                // Descriptors : normalize if we are averaging or keep as is
                if (averageExistingDescriptors) {
                	for ( int d = 0; d < descDim; d++ )
                		cloud.descriptors(d,firstPoint) /= numPoints;
                }

                pointsToKeep.push_back(firstPoint);
            }
        }

    }

    // deallocate memory for voxels information
    delete voxels;

    // Move the points to be kept to the start
    // Bring the data we keep to the front of the arrays then
	// wipe the leftover unused space.
	std::sort(pointsToKeep.begin(), pointsToKeep.end());
	int numPtsOut = pointsToKeep.size();
	for (int i = 0; i < numPtsOut; i++){
		int k = pointsToKeep[i];
		assert(i <= k);
		cloud.features.col(i) = cloud.features.col(k);
		if (cloud.descriptors.rows() != 0)
			cloud.descriptors.col(i) = cloud.descriptors.col(k);
	}
	cloud.features.conservativeResize(Eigen::NoChange, numPtsOut);
	
	if (cloud.descriptors.rows() != 0)
		cloud.descriptors.conservativeResize(Eigen::NoChange, numPtsOut);
}

template struct DataPointsFiltersImpl<float>::VoxelGridDataPointsFilter;
template struct DataPointsFiltersImpl<double>::VoxelGridDataPointsFilter;


// CutAtDescriptorThresholdDataPointsFilter
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::CutAtDescriptorThresholdDataPointsFilter::CutAtDescriptorThresholdDataPointsFilter(const Parameters& params):
	DataPointsFilter("CutAtDescriptorThresholdDataPointsFilter", CutAtDescriptorThresholdDataPointsFilter::availableParameters(), params),
	descName(Parametrizable::get<std::string>("descName")),
	useLargerThan(Parametrizable::get<bool>("useLargerThan")),
	threshold(Parametrizable::get<T>("threshold"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::CutAtDescriptorThresholdDataPointsFilter::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void DataPointsFiltersImpl<T>::CutAtDescriptorThresholdDataPointsFilter::inPlaceFilter(
	DataPoints& cloud)
{
	// Check field exists
	if (!cloud.descriptorExists(descName))
	{
		throw InvalidField("CutAtDescriptorThresholdDataPointsFilter: Error, field not found in descriptors.");
	}

	const int nbPointsIn = cloud.features.cols();
	typename DataPoints::View values = cloud.getDescriptorViewByName(descName);

	// fill cloud values
	int j = 0;
	if (useLargerThan)
	{
		for (int i = 0; i < nbPointsIn; i++)
		{
			const T value(values(0,i));
			if (value <= threshold)
			{
				cloud.setColFrom(j, cloud, i);
				j++;
			}
		}
	}
	else
	{
		for (int i = 0; i < nbPointsIn; i++)
		{
			const T value(values(0,i));
			if (value >= threshold)
			{
				cloud.setColFrom(j, cloud, i);
				j++;
			}
		}
	}
	cloud.conservativeResize(j);
}

template struct DataPointsFiltersImpl<float>::CutAtDescriptorThresholdDataPointsFilter;
template struct DataPointsFiltersImpl<double>::CutAtDescriptorThresholdDataPointsFilter;


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

	//TODO: should we do that in 2 passes or use conservativeResize?
	const int nbPointsIn = input.features.cols();
	const int nbRows = input.features.rows();
	int nbPointsOut = 0;
	if (dim == -1)
	{
		nbPointsOut = (input.features.topRows(nbRows-1).colwise().norm().array() < maxDist).count();
	}
	else
	{
		nbPointsOut = (input.features.row(dim).array().abs() < maxDist).count();
	}

	DataPoints outputCloud(
		typename DataPoints::Features(input.features.rows(), nbPointsOut),
		input.featureLabels
	);
	
	// if there is descriptors, copy the labels
	if (input.descriptors.cols() > 0)
	{
		outputCloud.descriptors = typename DataPoints::Descriptors(input.descriptors.rows(), nbPointsOut);
		outputCloud.descriptorLabels = input.descriptorLabels;
	}
	
	int j = 0;
	if(dim == -1) // Euclidian distance
	{
		for (int i = 0; i < nbPointsIn; i++)
		{
			if (input.features.col(i).head(nbRows-1).norm() < maxDist)
			{
				outputCloud.features.col(j) = input.features.col(i);
				if (outputCloud.descriptors.cols() > 0)
					outputCloud.descriptors.col(j) = input.descriptors.col(i);
				j++;
			}
		}
	}
	else // Single axis distance
	{
		for (int i = 0; i < nbPointsIn; i++)
		{
			if (anyabs(input.features(dim, i)) < maxDist)
			{
				outputCloud.features.col(j) = input.features.col(i);
				if (outputCloud.descriptors.cols() > 0)
					outputCloud.descriptors.col(j) = input.descriptors.col(i);
				j++;
			}
		}
	}
	
	assert(j == nbPointsOut);
	return outputCloud;
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
	int nbPointsOut = 0;
	if (dim == -1)
	{
		nbPointsOut = (input.features.topRows(nbRows-1).colwise().norm().array() > minDist).count();
	}
	else
	{
		nbPointsOut = (input.features.row(dim).array().abs() > minDist).count();
	}

	DataPoints outputCloud(
		typename DataPoints::Features(input.features.rows(), nbPointsOut),
		input.featureLabels
	);
	
	// if there is descriptors, copy the labels
	if (input.descriptors.cols() > 0)
	{
		outputCloud.descriptors = typename DataPoints::Descriptors(input.descriptors.rows(), nbPointsOut);
		outputCloud.descriptorLabels = input.descriptorLabels;
	}
	
	int j = 0;
	if(dim == -1) // Euclidian distance
	{
		for (int i = 0; i < nbPointsIn; i++)
		{
			if (input.features.col(i).head(nbRows-1).norm() > minDist)
			{
				outputCloud.features.col(j) = input.features.col(i);
				if (outputCloud.descriptors.cols() > 0)
					outputCloud.descriptors.col(j) = input.descriptors.col(i);
				j++;
			}
		}
	}
	else // Single axis distance
	{
		for (int i = 0; i < nbPointsIn; i++)
		{
			if (anyabs(input.features(dim, i)) > minDist)
			{
				outputCloud.features.col(j) = input.features.col(i);
				if (outputCloud.descriptors.cols() > 0)
					outputCloud.descriptors.col(j) = input.descriptors.col(i);
				j++;
			}
		}
	}
	assert(j == nbPointsOut);
	
	LOG_INFO_STREAM("MinDistDataPointsFilter- pts in: " << nbPointsIn << " pts out: " << j << " (-" << 100-(j/double(nbPointsIn))*100 << "\%)");
	
	return outputCloud;
}

template struct DataPointsFiltersImpl<float>::MinDistDataPointsFilter;
template struct DataPointsFiltersImpl<double>::MinDistDataPointsFilter;

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
	DataPoints outputCloud(
		typename DataPoints::Features(input.features.rows(), nbPointsOut),
		input.featureLabels
	);
	if (input.descriptors.cols() > 0)
	{
		outputCloud.descriptors = typename DataPoints::Descriptors(input.descriptors.rows(), nbPointsOut);
		outputCloud.descriptorLabels = input.descriptorLabels;
	}
	
	// fill output values
	int j = 0;
	for (int i = 0; i < nbPointsIn; i++)
	{
		if (input.features(dim, i) < limit)
		{
			outputCloud.features.col(j) = input.features.col(i);
			if (outputCloud.descriptors.cols() > 0)
				outputCloud.descriptors.col(j) = input.descriptors.col(i);
			j++;
		}
	}
	assert(j <= nbPointsOut);
	
	if (j < nbPointsOut)
	{
		if (outputCloud.descriptors.cols() > 0)
			return DataPoints(
				outputCloud.features.corner(Eigen::TopLeft,outputCloud.features.rows(),j),
				outputCloud.featureLabels,
				outputCloud.descriptors.corner(Eigen::TopLeft,outputCloud.descriptors.rows(),j),
				outputCloud.descriptorLabels
			);
		else
			return DataPoints(
				outputCloud.features.corner(Eigen::TopLeft,outputCloud.features.rows(),j),
				outputCloud.featureLabels
			);
	}
	
	return outputCloud;
}

template struct DataPointsFiltersImpl<float>::MaxQuantileOnAxisDataPointsFilter;
template struct DataPointsFiltersImpl<double>::MaxQuantileOnAxisDataPointsFilter;


// UniformizeDensityDataPointsFilter
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::UniformizeDensityDataPointsFilter::UniformizeDensityDataPointsFilter(const Parameters& params):
	DataPointsFilter("UniformizeDensityDataPointsFilter", UniformizeDensityDataPointsFilter::availableParameters(), params),
	aggressivity(Parametrizable::get<T>("aggressivity")),
	nbBin(Parametrizable::get<unsigned>("nbBin"))
{
}

// Structure for histogram (created for UniformizeDensityDataPointsFilter)
struct HistElement 
{
	int count;
	int id;
	float ratio;
	HistElement():count(0), id(-1), ratio(1.0){};
	static bool largestCountFirst(const HistElement h1, const HistElement h2)
	{
		return (h1.count > h2.count);	
	};
	static bool smallestIdFirst(const HistElement h1, const HistElement h2)
	{
		return (h1.id < h2.id);	
	};
};

template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::UniformizeDensityDataPointsFilter::filter(const DataPoints& input)
{
	
	
	DataPoints outputCloud;
	
	// Force densities to be computed
	if (input.getDescriptorByName("densities").cols() == 0)
	{
		LOG_INFO_STREAM("UniformizeDensityDataPointsFilter - WARNING: no densities found. Will force computation with default parameters");

		auto normalFilter = new typename DataPointsFiltersImpl<T>::SamplingSurfaceNormalDataPointsFilter(Parameters({{"ratio", "0.001"}, {"binSize", "7"}, {"keepNormals", "0"}, {"keepDensities", "1"}}));
		outputCloud = normalFilter->filter(input);
	}
	else
	{
		outputCloud = input;	
	}

	const int nbPointsIn = outputCloud.features.cols();

	const Matrix densities = outputCloud.getDescriptorByName("densities");
	typename DataPoints::Descriptors origineDistance(1, nbPointsIn);
	origineDistance = input.features.colwise().norm();

	const T minDist = densities.minCoeff();
	const T maxDist = densities.maxCoeff();
	const T delta = (maxDist - minDist)/(T)nbBin;

	typename DataPoints::Descriptors binId(1, nbPointsIn);
	
	std::vector<HistElement> hist;
	hist.resize(nbBin);

	// Initialize ids (useful to backtrack after sorting)
	for(unsigned i=0; i < nbBin; i++)
	{
		hist[i].id = i;	
	}

	// Associate a bin per point and cumulate the histogram
	for (int i=0; i < nbPointsIn; i++)
	{
		unsigned id = (densities(i)-minDist)/delta;

		// validate last interval
		if(id == nbBin)
			id = nbBin-1;

		hist[id].count = hist[id].count + 1;

		binId(i) = id;
		
	}
	
	/*cout << endl << "count:" << endl;
	for(unsigned i=0; i<hist.size(); i++)
	{
		cout << hist[i].count << ", ";	
	}
	*/

	const unsigned startingBin = nbBin*(1-aggressivity);
	
	// Compute the acceptance ratio per bin
	for(unsigned i=0; i<nbBin ; i++)
	{
		if(hist[i].count != 0 && i >= startingBin)
		{
			hist[i].ratio = 1 - double(i-startingBin)/double(nbBin-startingBin);
			hist[i].ratio *= hist[i].ratio; // square
		}
		else
		{
			hist[i].ratio = 1.0;
		}
	}
	
	/*cout << endl << "ratio:" << endl;
	for(unsigned i=0; i<hist.size(); i++)
	{
		cout << hist[i].ratio << ", ";	
	}
	cout << endl;
	*/

	// fill output values
	int j = 0;
	for (int i = 0; i < nbPointsIn; i++)
	{
		const int id = binId(i);
		const float r = (float)std::rand()/(float)RAND_MAX;
		if (r < hist[id].ratio)
		{
			outputCloud.features.col(j) = outputCloud.features.col(i);
			if (outputCloud.descriptors.cols() > 0)
				outputCloud.descriptors.col(j) = outputCloud.descriptors.col(i);
			j++;
		}
		
	}


	// Reduce the point cloud size
	outputCloud.features.conservativeResize(Eigen::NoChange, j);
	if (outputCloud.descriptors.cols() > 0)
			outputCloud.descriptors.conservativeResize(Eigen::NoChange,j);

	LOG_INFO_STREAM("UniformizeDensityDataPointsFilter - pts in: " << nbPointsIn << " pts out: " << j << " (-" << 100 - (j/double(nbPointsIn)*100) << "\%)");
	
	return outputCloud;
}

template struct DataPointsFiltersImpl<float>::UniformizeDensityDataPointsFilter;
template struct DataPointsFiltersImpl<double>::UniformizeDensityDataPointsFilter;


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
	typedef typename DataPoints::Features Features;
	typedef typename DataPoints::Descriptors Descriptors;
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
		throw std::runtime_error("Error, descritor labels do not match descriptor data");
	
	// Reserve memory for new descriptors
	const int dimNormals(featDim-1);
	const int dimDensities(1);
	const int dimEigValues(featDim-1);
	const int dimEigVectors((featDim-1)*(featDim-1));
	const int dimMatchedIds(knn); 

	Descriptors normals;
	Descriptors densities;
	Descriptors eigenValues;
	Descriptors eigenVectors;
	Descriptors matchedValues;
	if (keepNormals)
		normals.resize(dimNormals, pointsCount);

	if (keepDensities)
		densities.resize(dimDensities, pointsCount);

	if (keepEigenValues)
		eigenValues.resize(dimEigValues, pointsCount);

	if (keepEigenVectors)
		eigenVectors.resize(dimEigVectors, pointsCount);
	
	if (keepMatchedIds)
		matchedValues.resize(dimMatchedIds, pointsCount);

	// Build kd-tree
	KDTreeMatcher matcher(Parameters({
		{ "knn", toParam(knn) },
		{ "epsilon", toParam(epsilon) }
	}));
	matcher.init(input);

	Matches matches(typename Matches::Dists(knn, 1), typename Matches::Ids(knn, 1));
	
	// Search for surrounding points and compute descriptors
	int degenerateCount(0);
	for (int i = 0; i < pointsCount; ++i)
	{
		const DataPoints singlePoint(input.features.col(i), input.featureLabels, Matrix(), Labels());
		matches = matcher.findClosests(singlePoint, DataPoints());

		// Mean of nearest neighbors (NN)
		Matrix d(featDim-1, knn);
		for(int j = 0; j < int(knn); j++)
		{
			const int refIndex(matches.ids(j));
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
			if(C.fullPivHouseholderQr().rank() == featDim-1)
			{
				const Eigen::EigenSolver<Matrix> solver(C);
				eigenVa = solver.eigenvalues().real();
				eigenVe = solver.eigenvectors().real();
			}
			else
			{
				//TODO: solve without noise..
				//std::cout << "WARNING: Matrix C needed for eigen decomposition is degenerated. Expected cause: no noise in data" << std::endl;
				++degenerateCount;
			}
		}
		
		if(keepNormals)
			normals.col(i) = computeNormal(eigenVa, eigenVe);

		if(keepDensities)
			densities(i) = computeDensity(NN);

		if(keepEigenValues)
			eigenValues.col(i) = eigenVa;
		
		if(keepEigenVectors)
			eigenVectors.col(i) = serializeEigVec(eigenVe);
		
	}
	if (degenerateCount)
	{
		LOG_WARNING_STREAM("WARNING: Matrix C needed for eigen decomposition was degenerated in " << degenerateCount << " points over " << pointsCount << " (" << float(degenerateCount)*100.f/float(pointsCount) << " %)");
	}

	DataPoints output = input;
	output.addDescriptor("normals", normals);
	output.addDescriptor("densities", densities);
	output.addDescriptor("eigValues", eigenValues);
	output.addDescriptor("eigVectors", eigenVectors);
	
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
	//volume in decimeter
	T volume = (4./3.)*M_PI*std::pow(NN.colwise().norm().maxCoeff()*10.0, 3);
	//const T minVolume = 4.18e-9; // minimum of volume of one millimeter radius
	const T minVolume = 0.42; // minimum of volume of one centimeter radius (in dm^3)

	if(volume < minVolume) 		
		volume = minVolume;
		
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
	binSize(Parametrizable::get<int>("binSize")),
	samplingMethod(Parametrizable::get<int>("samplingMethod")),
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
	typedef typename DataPoints::Features Features;
	typedef typename DataPoints::Label Label;
	typedef typename DataPoints::Labels Labels;
	
	const int pointsCount(input.features.cols());
	const int featDim(input.features.rows());
	const int descDim(input.descriptors.rows());
	
	//if (pointsCount == 0)
	//	throw ConvergenceError("no point to filter");
	
	int insertDim(0);
	if (averageExistingDescriptors)
	{
		// Validate descriptors and labels
		for(unsigned int i = 0; i < input.descriptorLabels.size(); i++)
			insertDim += input.descriptorLabels[i].span;
		if (insertDim != descDim)
			throw std::runtime_error("Error, descriptor labels do not match descriptor data");
	}
	
	// Reserve memory for new descriptors
	const int dimNormals(featDim-1);
	const int dimDensities(1);
	const int dimEigValues(featDim-1);
	const int dimEigVectors((featDim-1)*(featDim-1));
	
	// we keep build data on stack for reentrant behaviour
	BuildData buildData(input.features, input.descriptors);


	if (keepNormals)
		buildData.normals.resize(dimNormals, pointsCount);

	if (keepDensities)
		buildData.densities.resize(dimDensities, pointsCount);

	if (keepEigenValues)
		buildData.eigValues.resize(dimEigValues, pointsCount);

	if (keepEigenVectors)
		buildData.eigVectors.resize(dimEigVectors, pointsCount);

	// build the new point cloud
	buildNew(
		buildData,
		0,
		pointsCount, 
		input.features.rowwise().minCoeff(),
		input.features.rowwise().maxCoeff()
	);
	
	const int ptsOut = buildData.outputInsertionPoint;

	LOG_INFO_STREAM("SamplingSurfaceNormalDataPointsFilter - pts in: " << pointsCount << " pts out: " << ptsOut << " (-" << 100-(ptsOut/double(pointsCount))*100 << "\%)");
	
	if(buildData.unfitPointsCount != 0)
		LOG_INFO_STREAM("SamplingSurfaceNormalDataPointsFilter - Coudn't compute normal for " << buildData.unfitPointsCount << " pts.");
	
	// Build the filtered point cloud
	DataPoints output;
	if(buildData.outputDescriptors.cols() == 0)
	{
		output = DataPoints(
			buildData.outputFeatures.leftCols(ptsOut),
			input.featureLabels
		);
	}
	else
	{
		output = DataPoints(
			buildData.outputFeatures.leftCols(ptsOut),
			input.featureLabels,
			buildData.outputDescriptors.leftCols(ptsOut),
			input.descriptorLabels
		);
	}

	// Add or replace new descriptors
	output.addDescriptor("normals", buildData.getResizedMatrix(buildData.normals));
	output.addDescriptor("densities", buildData.getResizedMatrix(buildData.densities));
	output.addDescriptor("eigValues", buildData.getResizedMatrix(buildData.eigValues));
	output.addDescriptor("eigVectors", buildData.getResizedMatrix(buildData.eigVectors));
	
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
	if (count <= int(binSize))
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
	const Vector mean = d.rowwise().sum() / T(colCount);
	const Matrix NN = (d.colwise() - mean);
	
	
	// compute covariance
	const Matrix C(NN * NN.transpose());
	Vector eigenVa = Vector::Identity(featDim-1, 1);
	Matrix eigenVe = Matrix::Identity(featDim-1, featDim-1);
	// Ensure that the matrix is suited for eigenvalues calculation
	if(keepNormals || keepEigenValues || keepEigenVectors)
	{
		if(C.fullPivHouseholderQr().rank() == featDim-1)
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

	// average the existing descriptors
	Vector mergedDesc;
	if(data.inputDescriptors.cols() != 0)
	{
		mergedDesc.resize(data.inputDescriptors.rows());

		if (averageExistingDescriptors)
		{
			for (int i = 0; i < colCount; ++i)
				mergedDesc += data.inputDescriptors.col(data.indices[first+i]);
			
			mergedDesc = mergedDesc / T(colCount);
		}
		else // just take the first one
			mergedDesc = data.inputDescriptors.col(data.indices[first]);
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
	
	// Filter points randomly
	if(samplingMethod == 0)
	{
		for(int i=0; i<colCount; i++)
		{
			const float r = (float)std::rand()/(float)RAND_MAX;
			if(r > ratio)
			{
				// Keep points with their descriptors
				data.outputFeatures.col(data.outputInsertionPoint) = 
					data.inputFeatures.col(data.indices[first+i]);
				if(data.outputDescriptors.cols() != 0)
				{
					data.outputDescriptors.col(data.outputInsertionPoint) = 
						data.inputDescriptors.col(data.indices[first+i]);
				}

				// Build new descriptor in paralelle to be merge at the end
				if(keepNormals)
					data.normals.col(data.outputInsertionPoint) = normal;
				if(keepDensities)
					data.densities(data.outputInsertionPoint) = densitie;
				if(keepEigenValues)
					data.eigValues.col(data.outputInsertionPoint) = eigenVa;
				if(keepEigenVectors)
					data.eigVectors.col(data.outputInsertionPoint) = serialEigVector;

				++data.outputInsertionPoint;
			}
		}
	}
	else
	{
		data.outputFeatures.col(data.outputInsertionPoint).topRows(featDim-1) = mean;
		data.outputFeatures(featDim-1, data.outputInsertionPoint) = 1;
		
		if(data.inputDescriptors.rows() != 0)
			data.outputDescriptors.col(data.outputInsertionPoint) = mergedDesc;
		
		// Build new descriptor in paralelle to be merge at the end
		if(keepNormals)
			data.normals.col(data.outputInsertionPoint) = normal;
		if(keepDensities)
			data.densities(data.outputInsertionPoint) = densitie;
		if(keepEigenValues)
			data.eigValues.col(data.outputInsertionPoint) = eigenVa;
		if(keepEigenVectors)
			data.eigVectors.col(data.outputInsertionPoint) = serialEigVector;
		
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
	typedef typename Eigen::Matrix<T, Eigen::Dynamic, 1> Vector;
	
	Matrix normals = input.getDescriptorByName("normals");

	if (normals.cols() == 0)
	{
		LOG_INFO_STREAM("Warning: cannot find normals in descriptors");
		return input;
	}
	
	const int nbPoints = input.features.cols();
	const int dim = input.features.rows();
	const int nbNormals = input.descriptors.cols();
	assert(nbPoints == nbNormals);

	double scalar;
	for (int i = 0; i < nbPoints; i++)
	{
		// Check normal orientation
		Vector vecP = -input.features.block(0, i, dim-1, 1);
		Vector vecN = normals.col(i);
		scalar = vecP.dot(vecN);

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

	// Update descriptor
	DataPoints output = input;
	int row(0);
	for (unsigned int j = 0; j < input.descriptorLabels.size(); j++)
	{
		const int span(input.descriptorLabels[j].span);
		if (input.descriptorLabels[j].text.compare("normals") == 0)
		{
			output.descriptors.block(row, 0, span, nbNormals) = normals;
			break;
		}
		row += span;
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

// Sampling
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::RandomSamplingDataPointsFilter::randomSample(const DataPoints& input) const
{
	Eigen::Matrix<double, 1, Eigen::Dynamic> filter;
	filter.setRandom(input.features.cols());
	filter = (filter.cwise() + 1)/2;

	const int nbPoints = (filter.cwise() < prob).count();

	//std::cout << "RandomSampling: size before: " << input.features.cols() << std::endl;

	typename DataPoints::Features filteredFeat(input.features.rows(), nbPoints);


	int j(0);
	for(int i = 0; i < filter.cols(); i++)
	{
		if(filter(i) < prob)
		{
			filteredFeat.col(j) = input.features.col(i);
			j++;
		}
	}

	// To handle no descriptors
	typename DataPoints::Descriptors filteredDesc;
	if(input.descriptors.cols() > 0)
	{
		filteredDesc = typename DataPoints::Descriptors(input.descriptors.rows(),nbPoints);
		
		int k(0);
		for(int i = 0; i < filter.cols(); i++)
		{
			if(filter(i) < prob)
			{
				filteredDesc.col(k) = input.descriptors.col(i);
				k++;
			}
		}
	}
	
	const int nbPointsIn = input.features.cols();
	const int nbPointsOut = filteredFeat.cols();
	LOG_INFO_STREAM("RandomSamplingDataPointsFilter - pts in: " << nbPointsIn << " pts out: " << nbPointsOut << " (-" << 100 - (nbPointsOut/double(nbPointsIn)*100) << "\%)");

	return DataPoints(filteredFeat, input.featureLabels, filteredDesc, input.descriptorLabels);
}

// Pre filter
template<typename T>
typename PointMatcher<T>::DataPoints DataPointsFiltersImpl<T>::RandomSamplingDataPointsFilter::filter(const DataPoints& input)
{
	return randomSample(input);
}

template struct DataPointsFiltersImpl<float>::RandomSamplingDataPointsFilter;
template struct DataPointsFiltersImpl<double>::RandomSamplingDataPointsFilter;


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
	LOG_INFO_STREAM( "Using FixStepSamplingDataPointsFilter with startStep=" << startStep << ", endStep=" << endStep << ", stepMult=" << stepMult); 
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
	const int nbPointsOut = ((nbPointsIn - phase) + iStep - 1) / iStep;
	typename DataPoints::Features filteredFeat(input.features.rows(), nbPointsOut);
	
	int j(0);
	for (int i = 0; i < nbPointsIn-phase; i++)
	{
		if ((i % iStep) == 0)
		{
			filteredFeat.col(j) = input.features.col(i+phase);
			j++;
		}
	}
	assert(j == nbPointsOut);

	// To handle no descriptors
	typename DataPoints::Descriptors filteredDesc;
	if (input.descriptors.cols() > 0)
	{
		filteredDesc = typename DataPoints::Descriptors(input.descriptors.rows(), nbPointsOut);
		
		j = 0;
		for (int i = 0; i < nbPointsIn-phase; i++)
		{
			if ((i % iStep) == 0)
			{
				filteredDesc.col(j) = input.descriptors.col(i+phase);
				j++;
			}
		}
		assert(j == nbPointsOut);
	}
	
	const double deltaStep(startStep * stepMult - startStep);
	step *= stepMult;
	if (deltaStep < 0 && step < endStep)
		step = endStep;
	if (deltaStep > 0 && step > endStep)
		step = endStep;
	
	return DataPoints(filteredFeat, input.featureLabels, filteredDesc, input.descriptorLabels);
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
	const int dim = input.features.rows();
	
	DataPoints outputCloud;

	// Force normals to be computed
	if (input.getDescriptorByName("normals").cols() == 0)
	{
		LOG_INFO_STREAM("ShadowDataPointsFilter - Warning: no normals found. Will force computation with default parameters");

		auto normalFilter = new typename DataPointsFiltersImpl<T>::SamplingSurfaceNormalDataPointsFilter(Parameters({{"ratio", "0.0001"}, {"binSize", "5"}, {"keepNormals", "1"}, {"keepDensities", "1"}}));
		outputCloud = normalFilter->filter(input);
	}
	else
	{
		outputCloud = input;	
	}

	const Matrix normals = outputCloud.getDescriptorByName("normals");
	int j = 0;

	for(int i=0; i < outputCloud.features.cols(); i++)
	{
		const Vector normal = normals.col(i).normalized();
		const Vector point = outputCloud.features.block(0, i, dim-1, 1).normalized();
		
		const T value = anyabs(normal.dot(point));

		if(value > eps) // test to keep the points
		{
			outputCloud.features.col(j) = outputCloud.features.col(i);
			outputCloud.descriptors.col(j) = outputCloud.descriptors.col(i);
			
			j++;
		}
	}

	outputCloud.features.conservativeResize(Eigen::NoChange, j);
	outputCloud.descriptors.conservativeResize(Eigen::NoChange, j);

	const int nbPointsIn = input.features.cols();
	const int nbPointsOut = outputCloud.features.cols();
	LOG_INFO_STREAM("ShadowDataPointsFilter - pts in: " << nbPointsIn << " pts out: " << nbPointsOut << " (-" << 100 - (nbPointsOut/double(nbPointsIn)*100) << "\%)");
	return outputCloud;
}

template struct DataPointsFiltersImpl<float>::ShadowDataPointsFilter;
template struct DataPointsFiltersImpl<double>::ShadowDataPointsFilter;


//SimpleSensorNoiseDataPointsFilter 
// Constructor
template<typename T>
DataPointsFiltersImpl<T>::SimpleSensorNoiseDataPointsFilter::SimpleSensorNoiseDataPointsFilter(const Parameters& params):
	DataPointsFilter("SimpleSensorNoiseDataPointsFilter", SimpleSensorNoiseDataPointsFilter::availableParameters(), params),
	sensorType(Parametrizable::get<unsigned>("sensorType")),
	gain(Parametrizable::get<T>("gain"))
{
	std::vector<string> sensorNames = {"SickLMS"};
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
	typedef typename Eigen::Array<T, 2, Eigen::Dynamic> Array2rows;
	const int nbPoints = input.features.cols();
	const int dim = input.features.rows();
	DataPoints outputCloud = input;
	Descriptors noise;
	noise.resize(1, nbPoints);

	if(sensorType == 0)
	{
		const T minRadius = 0.01; // in meter
		const T beamAngle = 0.01745; // in rad
		Array2rows evalNoise = Array2rows::Constant(2, nbPoints, minRadius);
		evalNoise.row(0) =  sin(beamAngle) * input.features.topRows(dim-1).colwise().norm();

		noise = evalNoise.colwise().maxCoeff();
	}

	outputCloud.addDescriptor("simpleSensorNoise", noise);

	return outputCloud;
}

template struct DataPointsFiltersImpl<float>::SimpleSensorNoiseDataPointsFilter;
template struct DataPointsFiltersImpl<double>::SimpleSensorNoiseDataPointsFilter;



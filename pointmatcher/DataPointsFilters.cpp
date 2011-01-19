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

#include "Core.h"
#include <stdexcept>
#include <algorithm>

// Eigenvalues
#include "Eigen/QR"

using namespace std;

// IdentityDataPointsFilter
template<typename T>
typename MetricSpaceAligner<T>::DataPoints MetricSpaceAligner<T>::IdentityDataPointsFilter::preFilter(
	const DataPoints& input, 
	bool& iterate) const
{
	return input;
}

template struct MetricSpaceAligner<float>::IdentityDataPointsFilter;
template struct MetricSpaceAligner<double>::IdentityDataPointsFilter;


// SurfaceNormalDataPointsFilter
// Constructor
template<typename T>
MetricSpaceAligner<T>::SurfaceNormalDataPointsFilter::SurfaceNormalDataPointsFilter(
	const int knn, 
	const double epsilon, 
	const bool keepNormals, 
	const bool keepDensities, 
	const bool keepEigenValues, 
	const bool keepEigenVectors,
	const bool keepMatchedIds):
	knn(knn),
	epsilon(epsilon),
	keepNormals(keepNormals),
	keepDensities(keepDensities),
	keepEigenValues(keepEigenValues),
	keepEigenVectors(keepEigenVectors),
	keepMatchedIds(keepMatchedIds)
{
}


// Compute
template<typename T>
typename MetricSpaceAligner<T>::DataPoints MetricSpaceAligner<T>::SurfaceNormalDataPointsFilter::preFilter(
	const DataPoints& input, 
	bool& iterate) const
{
	std::cerr << "SurfaceNormalDataPointsFilter::preFilter" << std::endl;
	//typedef typename Types<T>::DataPoints DataPoints;
	typedef typename DataPoints::Features Features;
	typedef typename Eigen::Matrix<T, Eigen::Dynamic, 1> Vector;
	typedef typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Matrix;
	typedef typename DataPoints::Label Label;
	typedef std::vector<Label> Labels;
	
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
	int finalDim(insertDim);
	const int dimNormals(featDim-1);
	const int dimDensities(1);
	const int dimEigValues(featDim-1);
	const int dimEigVectors((featDim-1)*(featDim-1));
	const int dimMatchedIds(knn); 
	Labels newDescriptorLabels(input.descriptorLabels);

	if (keepNormals)
	{
		newDescriptorLabels.push_back(Label("normals", dimNormals));
		finalDim += dimNormals;
	}

	if (keepDensities)
	{
		newDescriptorLabels.push_back(Label("densities", dimDensities));
		finalDim += dimDensities;
	}

	if (keepEigenValues)
	{
		newDescriptorLabels.push_back(Label("eigValues", dimEigValues));
		finalDim += dimEigValues;
	}

	if (keepEigenVectors)
	{
		newDescriptorLabels.push_back(Label("eigVectors", dimEigVectors));
		finalDim += dimEigVectors;
	}
	
	if (keepMatchedIds)
	{
		newDescriptorLabels.push_back(Label("matchedIds", dimMatchedIds));
		finalDim += dimMatchedIds;
	}

	Matrix newDescriptors(finalDim, pointsCount);
	
	KDTreeMatcher matcher(knn, epsilon);
	matcher.init(DataPoints(), input, iterate);

	Matches matches(typename Matches::Dists(knn, 1), typename Matches::Ids(knn, 1));
	// Search for surrounding points
	int degenerateCount(0);
	for (int i = 0; i < pointsCount; ++i)
	{
		Vector mean(Vector::Zero(featDim-1));
		Features NN(featDim-1, knn);
		
		DataPoints singlePoint(input.features.col(i), input.featureLabels, Matrix(), Labels());
		matches = matcher.findClosests(singlePoint, DataPoints(), iterate);

		// Mean of nearest neighbors (NN)
		for(int j = 0; j < knn; j++)
		{
			const int refIndex(matches.ids(j));
			const Vector v(input.features.block(0, refIndex, featDim-1, 1));
			NN.col(j) = v;
			mean += v;
		}

		mean /= knn;
		
		// Covariance of nearest neighbors
		for (int j = 0; j < knn; ++j)
		{
			//std::cout << "NN.col(j):\n" << NN.col(j) << std::endl;
			//std::cout << "mean:\n" << mean << std::endl;
			NN.col(j) -= mean;
		}
		
		const Matrix C(NN * NN.transpose());
		Vector eigenVa = Vector::Identity(featDim-1, 1);
		Matrix eigenVe = Matrix::Identity(featDim-1, featDim-1);
		// Ensure that the matrix is suited for eigenvalues calculation
		if(C.qr().rank() == featDim-1)
		{
			eigenVa = Eigen::EigenSolver<Matrix>(C).eigenvalues().real();
			eigenVe = Eigen::EigenSolver<Matrix>(C).eigenvectors().real();
		}
		else
		{
			//TODO: solve without noise..
			//std::cout << "WARNING: Matrix C needed for eigen decomposition is degenerated. Expected cause: no noise in data" << std::endl;
			++degenerateCount;
		}
		
		int posCount(insertDim);
		if(keepNormals)
		{
			// Keep the smallest eigenvector as surface normal
			int smallestId(0);
			T smallestValue(numeric_limits<T>::max());
			for(int j = 0; j < dimNormals; j++)
			{
				if (eigenVa(j) < smallestValue)
				{
					smallestId = j;
					smallestValue = eigenVa(j);
				}
			}
			
			newDescriptors.block(posCount, i, dimNormals, 1) = 
			//eigenVe.row(smallestId).transpose();
			eigenVe.col(smallestId);
			posCount += dimNormals;
		}

		if(keepDensities)
		{
			//TODO: set lambda to a realistic value (based on sensor)
			//TODO: debug here: volume too low 
			const double epsilon(0.005);

			//T volume(eigenVa(0)+lambda);
			T volume(eigenVa(0));
			for(int j = 1; j < eigenVa.rows(); j++)
			{
				volume *= eigenVa(j);
			}
			newDescriptors(posCount, i) = knn/(volume + epsilon);
			posCount += dimDensities;
		}

		if(keepEigenValues)
		{
			newDescriptors.block(posCount, i, featDim-1, 1) = 
				eigenVa;
			posCount += dimEigValues;
		}
		
		if(keepEigenVectors)
		{
			//TODO: Watch for that serialization
			for(int k=0; k < (featDim-1); k++)
			{
				newDescriptors.block(
					posCount + k*(featDim-1), i, (featDim-1), 1) = 
						(eigenVe.row(k).transpose().cwise() * eigenVa);
			}
			
			posCount += dimEigVectors;
		}
		
		if(keepMatchedIds)
		{
			// BUG: cannot used .cast<T>() on dynamic matrices...
			for(int k=0; k < dimMatchedIds; k++)
			{
				newDescriptors(posCount + k, i) = (T)matches.ids(k);
			}
			
			posCount += dimMatchedIds;
		}
	}
	if (degenerateCount)
	{
		std::cout << "WARNING: Matrix C needed for eigen decomposition was degenerated in " << degenerateCount << " points over " << pointsCount << " (" << float(degenerateCount)*100.f/float(pointsCount) << " %)" << std::endl;
	}
	
	return DataPoints(input.features, input.featureLabels, newDescriptors, newDescriptorLabels);
}

template struct MetricSpaceAligner<float>::SurfaceNormalDataPointsFilter;
template struct MetricSpaceAligner<double>::SurfaceNormalDataPointsFilter;


// SamplingSurfaceNormalDataPointsFilter

// Constructor
template<typename T>
MetricSpaceAligner<T>::SamplingSurfaceNormalDataPointsFilter::SamplingSurfaceNormalDataPointsFilter(
	const int k,
	const bool averageExistingDescriptors,
	const bool keepNormals,
	const bool keepDensities,
	const bool keepEigenValues, 
	const bool keepEigenVectors):
	k(k),
	averageExistingDescriptors(averageExistingDescriptors),
	keepNormals(keepNormals),
	keepDensities(keepDensities),
	keepEigenValues(keepEigenValues),
	keepEigenVectors(keepEigenVectors)
{
}

// Compute
template<typename T>
typename MetricSpaceAligner<T>::DataPoints MetricSpaceAligner<T>::SamplingSurfaceNormalDataPointsFilter::preFilter(
	const DataPoints& input, 
	bool& iterate) const
{
	//std::cerr << "SamplingSurfaceNormalDataPointsFilter::preFilter " << input.features.cols() << std::endl;
	//typedef typename Types<T>::DataPoints DataPoints;
	typedef typename DataPoints::Features Features;
	typedef typename Eigen::Matrix<T, Eigen::Dynamic, 1> Vector;
	typedef typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Matrix;
	typedef typename DataPoints::Label Label;
	typedef std::vector<Label> Labels;
	
	const int pointsCount(input.features.cols());
	const int featDim(input.features.rows());
	const int descDim(input.descriptors.rows());

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
	int finalDescDim(insertDim);
	const int dimNormals(featDim-1);
	const int dimDensities(1);
	const int dimEigValues(featDim-1);
	const int dimEigVectors((featDim-1)*(featDim-1));
	Labels outputDescriptorLabels(input.descriptorLabels);

	if (keepNormals)
	{
		outputDescriptorLabels.push_back(Label("normals", dimNormals));
		finalDescDim += dimNormals;
	}

	if (keepDensities)
	{
		outputDescriptorLabels.push_back(Label("densities", dimDensities));
		finalDescDim += dimDensities;
	}

	if (keepEigenValues)
	{
		outputDescriptorLabels.push_back(Label("eigValues", dimEigValues));
		finalDescDim += dimEigValues;
	}

	if (keepEigenVectors)
	{
		outputDescriptorLabels.push_back(Label("eigVectors", dimEigVectors));
		finalDescDim += dimEigVectors;
	}
	
	// we keep build data on stack for reentrant behaviour
	BuildData buildData(input.features, input.descriptors, finalDescDim);
	
	// build the new point cloud
	buildNew(
		buildData,
		0,
		pointsCount, 
		input.features.rowwise().minCoeff(),
		input.features.rowwise().maxCoeff()
	);
	
	//std::cerr << "SamplingSurfaceNormalDataPointsFilter::preFilter done " << buildData.outputInsertionPoint << std::endl;
	
	// return the new point cloud
	return DataPoints(
		buildData.outputFeatures.corner(Eigen::TopLeft, buildData.outputFeatures.rows(), buildData.outputInsertionPoint),
		input.featureLabels,
		buildData.outputDescriptors.corner(Eigen::TopLeft, buildData.outputDescriptors.rows(), buildData.outputInsertionPoint),
		outputDescriptorLabels
	);
}

// TODO: move into a file Utils.h
template<typename T>
size_t argMax(const typename MetricSpaceAligner<T>::Vector& v)
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
void MetricSpaceAligner<T>::SamplingSurfaceNormalDataPointsFilter::buildNew(BuildData& data, const int first, const int last, const Vector minValues, const Vector maxValues) const
{
	const int count(last - first);
	if (count <= k)
	{
		// compute for this range
		fuseRange(data, first, last);
		return;
	}
	
	// find the largest dimension of the box
	const int cutDim = argMax<T>(maxValues - minValues);
	
	// compute number of elements
	const int rightCount(count/2);
	const int leftCount(count - rightCount);
	assert(last - rightCount == first + leftCount);
	
	//cerr << "cutting on dim " << cutDim << " at " << leftCount << endl;
	
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
void MetricSpaceAligner<T>::SamplingSurfaceNormalDataPointsFilter::fuseRange(BuildData& data, const int first, const int last) const
{
	const int colCount(last-first);
	const int featDim(data.inputFeatures.rows());
	assert(featDim == data.outputFeatures.rows());
	
	// build nearest neighbors list
	Matrix d(featDim-1, colCount);
	for (int i = 0; i < colCount; ++i)
		d.col(i) = data.inputFeatures.block(0,data.indices[first+i],featDim-1, 1);
	const Vector& mean = d.rowwise().sum() / T(colCount);
	// FIXME: with Eigen3, do this:
	//const Matrix& NN = (d.colwise() - mean);
	Matrix NN(d);
	for (int i = 0; i < NN.cols(); ++i)
		NN.col(i) -= mean;
	data.outputFeatures.block(0, data.outputInsertionPoint, featDim-1,1) = mean;
	data.outputFeatures(featDim-1, data.outputInsertionPoint) = 1;
	
	// compute covariance
	const Matrix C(NN * NN.transpose());
	Vector eigenVa = Vector::Identity(featDim-1, 1);
	Matrix eigenVe = Matrix::Identity(featDim-1, featDim-1);
	// Ensure that the matrix is suited for eigenvalues calculation
	if(C.qr().rank() == featDim-1)
	{
		eigenVa = Eigen::EigenSolver<Matrix>(C).eigenvalues().real();
		eigenVe = Eigen::EigenSolver<Matrix>(C).eigenvectors().real();
	}
	else
	{
		return;
		//std::cout << "WARNING: Matrix C needed for eigen decomposition is degenerated. Expected cause: no noise in data" << std::endl;
	}
	
	// average the existing descriptors
	int insertDim(0);
	if (averageExistingDescriptors && (data.inputDescriptors.rows() != 0))
	{
		Vector newDesc(data.inputDescriptors.rows());
		for (int i = 0; i < colCount; ++i)
			newDesc += data.inputDescriptors.block(0,data.indices[first+i],data.inputDescriptors.rows(), 1);
		data.outputDescriptors.block(0, data.outputInsertionPoint, data.inputDescriptors.rows(), 1) =
			newDesc / T(colCount);
		insertDim += data.inputDescriptors.rows();
		/*cerr << "averaging desc:\n";
		cerr << data.inputDescriptors.block(0, first, data.inputDescriptors.rows(), colCount) << endl;
		cerr << data.inputDescriptors.block(0, first, data.inputDescriptors.rows(), colCount).rowwise().sum() << endl;
		cerr << data.outputDescriptors.block(0, data.outputInsertionPoint, data.inputDescriptors.rows(), 1) << endl;*/
	}
	
	if(keepNormals)
	{
		const int dimNormals(featDim-1);
		// Keep the smallest eigenvector as surface normal
		int smallestId(0);
		T smallestValue(numeric_limits<T>::max());
		for(int j = 0; j < dimNormals; j++)
		{
			if (eigenVa(j) < smallestValue)
			{
				smallestId = j;
				smallestValue = eigenVa(j);
			}
		}
		data.outputDescriptors.block(insertDim, data.outputInsertionPoint, dimNormals, 1) =
			eigenVe.col(smallestId);
		insertDim += dimNormals;
	}

	if(keepDensities)
	{
		//TODO: set lambda to a realistic value (based on sensor)
		//TODO: debug here: volume too low 
		const T epsilon(0.005);

		//T volume(eigenVa(0)+lambda);
		T volume(eigenVa(0));
		for(int j = 1; j < eigenVa.rows(); j++)
		{
			volume *= eigenVa(j);
		}
		data.outputDescriptors(insertDim, data.outputInsertionPoint) =
			T(colCount)/(volume + epsilon);
		insertDim += 1;
	}
	
	if(keepEigenValues)
	{
		const int dimEigValues(featDim-1);
		data.outputDescriptors.block(insertDim, data.outputInsertionPoint, dimEigValues, 1) = 
			eigenVa;
		insertDim += dimEigValues;
	}
	
	if(keepEigenVectors)
	{
		const int dimEigVectors((featDim-1)*(featDim-1));
		//TODO: Watch for that serialization
		for(int k=0; k < (featDim-1); k++)
		{
			data.outputDescriptors.block(
				insertDim +  k*(featDim-1), data.outputInsertionPoint, (featDim-1), 1) = 
					(eigenVe.row(k).transpose().cwise() * eigenVa);
		}
		insertDim += dimEigVectors;
	}
	
	++data.outputInsertionPoint;
}

template struct MetricSpaceAligner<float>::SamplingSurfaceNormalDataPointsFilter;
template struct MetricSpaceAligner<double>::SamplingSurfaceNormalDataPointsFilter;


// OrientNormalsDataPointsFilter
// Compute
template<typename T>
typename MetricSpaceAligner<T>::DataPoints MetricSpaceAligner<T>::OrientNormalsDataPointsFilter::preFilter(const DataPoints& input, bool& iterate) const
{
	Matrix normals = input.getDescriptorByName("normals");

	if (normals.cols() == 0)
	{
		cerr << "Warning: cannot find normals in descriptors" << endl;
		return input;
	}
	
	const int nbPoints = input.features.cols();
	const int nbNormals = input.descriptors.cols();
	assert(nbPoints == nbNormals);

	Matrix points = input.features.block(0, 0, 3, nbPoints);
	
	double scalar;
	for (int i = 0; i < nbPoints; i++)
	{
		// Check normal orientation
		Vector3 vecP = -points.col(i);
		Vector3 vecN = normals.col(i);
		scalar = vecP.dot(vecN);
		//if (scalar < 0)
		if (scalar > 0)
		{
			// Swap normal
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

template struct MetricSpaceAligner<float>::OrientNormalsDataPointsFilter;
template struct MetricSpaceAligner<double>::OrientNormalsDataPointsFilter;


// RandomSamplingDataPointsFilter
// Constructor
template<typename T>
MetricSpaceAligner<T>::RandomSamplingDataPointsFilter::RandomSamplingDataPointsFilter(
	const double prob, bool enablePreFilter, bool enableStepFilter):
	prob(prob),
	enableStepFilter(enableStepFilter),
	enablePreFilter(enablePreFilter)
{
	assert(prob > 0 && prob <= 1);
}

// Sampling
template<typename T>
typename MetricSpaceAligner<T>::DataPoints MetricSpaceAligner<T>::RandomSamplingDataPointsFilter::randomSample(const DataPoints& input) const
{
	Eigen::Matrix<double, 1, Eigen::Dynamic> filter;
	filter.setRandom(input.features.cols());
	filter = (filter.cwise() + 1)/2;

	const int nbPoints = (filter.cwise() < prob).count();

	std::cout << "RandomSampling: size before: " << input.features.cols() << std::endl;

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

	std::cout << "RandomSampling: size after: " << filteredFeat.cols() << std::endl;
	
	return DataPoints(filteredFeat, input.featureLabels, filteredDesc, input.descriptorLabels);
}

// Pre filter
template<typename T>
typename MetricSpaceAligner<T>::DataPoints MetricSpaceAligner<T>::RandomSamplingDataPointsFilter::preFilter(const DataPoints& input,	bool& iterate) const
{
	if(enablePreFilter)
		return randomSample(input);
	else
		return input;
}

// Step filter
template<typename T>
typename MetricSpaceAligner<T>::DataPoints MetricSpaceAligner<T>::RandomSamplingDataPointsFilter::stepFilter(const DataPoints& input,	bool& iterate) const
{
	if(enableStepFilter)
		return randomSample(input);
	else
		return input;
}

template struct MetricSpaceAligner<float>::RandomSamplingDataPointsFilter;
template struct MetricSpaceAligner<double>::RandomSamplingDataPointsFilter;


// FixstepSamplingDataPointsFilter
// Constructor
template<typename T>
MetricSpaceAligner<T>::FixstepSamplingDataPointsFilter::FixstepSamplingDataPointsFilter(
	const int step, bool enablePreFilter, bool enableStepFilter):
	step(step),
	enableStepFilter(enableStepFilter),
	enablePreFilter(enablePreFilter)
{
	assert(step > 0);
}

// Sampling
template<typename T>
typename MetricSpaceAligner<T>::DataPoints MetricSpaceAligner<T>::FixstepSamplingDataPointsFilter::fixstepSample(const DataPoints& input) const
{
	const int nbPointsIn = input.features.cols();
	const int nbPointsOut = (nbPointsIn + step - 1) / step;
	typename DataPoints::Features filteredFeat(input.features.rows(), nbPointsOut);
	
	int j(0);
	for (int i = 0; i < nbPointsIn; i++)
	{
		if ((i % step) == 0)
		{
			filteredFeat.col(j) = input.features.col(i);
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
		for (int i = 0; i < nbPointsIn; i++)
		{
			if ((i % step) == 0)
			{
				filteredDesc.col(j) = input.descriptors.col(i);
				j++;
			}
		}
		assert(j == nbPointsOut);
	}
	
	return DataPoints(filteredFeat, input.featureLabels, filteredDesc, input.descriptorLabels);
}

// Pre filter
template<typename T>
typename MetricSpaceAligner<T>::DataPoints MetricSpaceAligner<T>::FixstepSamplingDataPointsFilter::preFilter(const DataPoints& input,	bool& iterate) const
{
	if(enablePreFilter)
		return fixstepSample(input);
	else
		return input;
}

// Step filter
template<typename T>
typename MetricSpaceAligner<T>::DataPoints MetricSpaceAligner<T>::FixstepSamplingDataPointsFilter::stepFilter(const DataPoints& input,	bool& iterate) const
{
	if(enableStepFilter)
		return fixstepSample(input);
	else
		return input;
}

template struct MetricSpaceAligner<float>::FixstepSamplingDataPointsFilter;
template struct MetricSpaceAligner<double>::FixstepSamplingDataPointsFilter;


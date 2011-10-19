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

#include "ErrorMinimizers.h"

#include "Eigen/SVD"
#include <iostream>

using namespace Eigen;
using namespace std;

template<typename T>
PointMatcher<T>::ErrorMinimizer::ErrorElements::ErrorElements(const DataPoints& reading, const DataPoints reference, const OutlierWeights weights, const Matches matches):
	reading(reading),
	reference(reference),
	weights(weights),
	matches(matches)
{
	assert(reading.features.cols() == reference.features.cols());
	assert(reading.features.cols() == weights.cols());
	assert(reading.features.cols() == matches.dists.cols());
	// May have no descriptors... size 0
}

// Identity Error Minimizer
template<typename T>
typename PointMatcher<T>::TransformationParameters ErrorMinimizersImpl<T>::IdentityErrorMinimizer::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const OutlierWeights& outlierWeights,
	const Matches& matches)
{
	return TransformationParameters::Identity(filteredReading.features.rows(), filteredReading.features.rows());
}

template struct ErrorMinimizersImpl<float>::IdentityErrorMinimizer;
template struct ErrorMinimizersImpl<double>::IdentityErrorMinimizer;


// Point To POINT ErrorMinimizer
template<typename T>
typename PointMatcher<T>::TransformationParameters ErrorMinimizersImpl<T>::PointToPointErrorMinimizer::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const OutlierWeights& outlierWeights,
	const Matches& matches)
{
	typedef typename DataPoints::Features Features;
	typedef typename Matches::Ids Ids;
	
	assert(matches.ids.rows() > 0);

	typename ErrorMinimizer::ErrorElements mPts = this->getMatchedPoints(filteredReading, filteredReference, matches, outlierWeights);
	
	// now minimize on kept points
	const int dimCount(mPts.reading.features.rows());
	const int ptsCount(mPts.reading.features.cols()); //But point cloud have now the same number of (matched) point

	// Compute the mean of each point cloud
	const Vector meanReading = mPts.reading.features.rowwise().sum() / ptsCount;
	const Vector meanReference = mPts.reference.features.rowwise().sum() / ptsCount;
	
	// Remove the mean from the point clouds
	mPts.reading.features.colwise() -= meanReading;
	mPts.reference.features.colwise() -= meanReference;

	// Singular Value Decomposition
	const Matrix m(mPts.reference.features.topRows(dimCount-1) * mPts.reading.features.topRows(dimCount-1).transpose());
	const JacobiSVD<Matrix> svd(m, ComputeThinU | ComputeThinV);
	const Matrix rotMatrix(svd.matrixU() * svd.matrixV().transpose());
	const Vector trVector(meanReference.head(dimCount-1)- rotMatrix * meanReading.head(dimCount-1));
	
	Matrix result(Matrix::Identity(dimCount, dimCount));
	result.corner(TopLeft, dimCount-1, dimCount-1) = rotMatrix;
	result.corner(TopRight, dimCount-1, 1) = trVector;
	
	return result;
}

template struct ErrorMinimizersImpl<float>::PointToPointErrorMinimizer;
template struct ErrorMinimizersImpl<double>::PointToPointErrorMinimizer;


// Point To PLANE ErrorMinimizer

template<typename T>
typename PointMatcher<T>::TransformationParameters ErrorMinimizersImpl<T>::PointToPlaneErrorMinimizer::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const OutlierWeights& outlierWeights,
	const Matches& matches)
{
	typedef typename PointMatcher<T>::ConvergenceError ConvergenceError;
	typedef typename DataPoints::Features Features;
	typedef typename Matches::Ids Ids;
	
	assert(matches.ids.rows() > 0);
	
	// if 2D, use homogenious coordinates [x, y] 
	// if 3D, use [x, y, z]

	const typename ErrorMinimizer::ErrorElements mPts = this->getMatchedPoints(filteredReading, filteredReference, matches, outlierWeights);
	
	const Matrix normalRef = mPts.reference.getDescriptorByName("normals");

	// Normal vector must be precalculated to use this error. Use appropriate input filter.
	assert(normalRef.rows() > 0);

	// Compute cross product of cross = cross(reading X normalRef)
	const Matrix cross = this->crossProduct(mPts.reading.features, normalRef);

	// wF = [weights*cross, normals]
	// F  = [cross, normals]
	Matrix wF(normalRef.rows()+ cross.rows(), normalRef.cols());
	Matrix F(normalRef.rows()+ cross.rows(), normalRef.cols());
	
	for(int i=0; i < cross.rows(); i++)
	{
		wF.row(i) = mPts.weights.cwise() * cross.row(i);
		F.row(i) = cross.row(i);
	}
	for(int i=0; i < normalRef.rows(); i++)
	{
		wF.row(i + cross.rows()) = normalRef.row(i);
		F.row(i + cross.rows()) = normalRef.row(i);
	}

	// Unadjust covariance A = wF * F'
	const Matrix A = wF * F.transpose();
	if (A.fullPivHouseholderQr().rank() != A.rows())
	{
		throw ConvergenceError("encountered singular while minimizing point to plane distance");
	}

	const Matrix deltas = mPts.reading.features - mPts.reference.features;

	// dot product of dot = dot(deltas, normals)
	Matrix dotProd = Matrix::Zero(1, normalRef.cols());
	
	for(int i=0; i<normalRef.rows(); i++)
	{
		dotProd += (deltas.row(i).cwise() * normalRef.row(i));
	}

	// b = -(wF' * dot)
	const Vector b = -(wF * dotProd.transpose());

	// Cholesky decomposition
	Vector x(A.rows());
	A.llt().solve(b, &x);
	
	// Transform parameters to matrix
	Matrix mOut;
	if(normalRef.rows() == 3)
	{
		Eigen::Transform<T, 3, Eigen::Affine> transform;
		// Rotation in Eular angles follow roll-pitch-yaw (1-2-3) rule
		transform = Eigen::AngleAxis<T>(x(0), Eigen::Matrix<T,1,3>::UnitX())
				* Eigen::AngleAxis<T>(x(1), Eigen::Matrix<T,1,3>::UnitY())
				* Eigen::AngleAxis<T>(x(2), Eigen::Matrix<T,1,3>::UnitZ());
		// Reverse roll-pitch-yaw conversion, very useful piece of knowledge, keep it with you all time!
		/*const T pitch = -asin(transform(2,0));
		const T roll = atan2(transform(2,1), transform(2,2));
		const T yaw = atan2(transform(1,0) / cos(pitch), transform(0,0) / cos(pitch));
		std::cerr << "d angles" << x(0) - roll << ", " << x(1) - pitch << "," << x(2) - yaw << std::endl;*/
		transform.translation() = x.segment(3, 3);
		mOut = transform.matrix();
	}
	else
	{
		Eigen::Transform<T, 2, Eigen::Affine> transform;
		transform = Eigen::Rotation2D<T> (x(0));
		transform.translation() = x.segment(1, 2);

		mOut = transform.matrix();
	}
	return mOut; 
}

//! Helpper funtion doing the cross product in 3D and a pseudo cross product in 2D
template<typename T>
typename PointMatcher<T>::Matrix PointMatcher<T>::ErrorMinimizer::crossProduct(const Matrix& A, const Matrix& B)
{
	// Expecting matched points
	assert(A.cols() == B.cols());

	// Expecting homogenous coord X eucl. coord
	assert(A.rows() -1 == B.rows());

	// Expecting homogenous coordinates
	assert(A.rows() == 4 || A.rows() == 3);
	
	const unsigned int x = 0;
	const unsigned int y = 1;
	const unsigned int z = 2;

	Matrix cross;
	if(A.rows() == 4)
	{
		cross = Matrix(B.rows(), B.cols());
				
		cross.row(x) = A.row(y).cwise() * B.row(z) - A.row(z).cwise() * B.row(y);
		cross.row(y) = A.row(z).cwise() * B.row(x) - A.row(x).cwise() * B.row(z);
		cross.row(z) = A.row(x).cwise() * B.row(y) - A.row(y).cwise() * B.row(x);
	}
	else
	{
		cross = Vector(B.cols());
		cross = A.row(x).cwise() * B.row(y) - A.row(y).cwise() * B.row(x);
	}

	return cross;
}


//! Helper function outputting pair of points from the reference and 
//! the reading based on the matching matrix
template<typename T>
typename PointMatcher<T>::ErrorMinimizer::ErrorElements PointMatcher<T>::ErrorMinimizer::getMatchedPoints(
		const DataPoints& requestedPts,
		const DataPoints& sourcePts,
		const Matches& matches, 
		const OutlierWeights& outlierWeights)
{
	typedef typename DataPoints::Features Features;
	typedef typename DataPoints::Descriptors Descriptors;
	typedef typename Matches::Ids Ids;
	typedef typename Matches::Dists Dists;
	
	assert(matches.ids.rows() > 0);
	assert(matches.ids.cols() > 0);
	assert(matches.ids.cols() == requestedPts.features.cols()); //nbpts
	assert(outlierWeights.rows() == matches.ids.rows());  // knn
	
	const int knn = outlierWeights.rows();
	const int dimFeat = requestedPts.features.rows();
	const int dimReqDesc = requestedPts.descriptors.rows();

	// Count points with no weights
	const int pointsCount = (outlierWeights.cwise() != 0.0).count();
	if (pointsCount == 0)
		throw ConvergenceError("no point to minimize");

	Features keptFeat(dimFeat, pointsCount);
	
	Descriptors keptDesc;
	if(dimReqDesc > 0)
		keptDesc = Descriptors(dimReqDesc, pointsCount);

	Matches keptMatches (Dists(1,pointsCount), Ids(1, pointsCount));
	OutlierWeights keptWeights(1, pointsCount);

	int j = 0;
	weightedPointUsedRatio = 0;
	for(int k = 0; k < knn; k++) // knn
	{
		for (int i = 0; i < requestedPts.features.cols(); ++i) //nb pts
		{
			if (outlierWeights(k,i) != 0.0)
			{
				if(dimReqDesc > 0)
					keptDesc.col(j) = requestedPts.descriptors.col(i);
				
				keptFeat.col(j) = requestedPts.features.col(i);
				keptMatches.ids(0, j) = matches.ids(k, i);
				keptMatches.dists(0, j) = matches.dists(k, i);
				keptWeights(0,j) = outlierWeights(k,i);
				++j;
				weightedPointUsedRatio += outlierWeights(k,i);
			}
		}
	}

	//FIXME: This is not true with multiple knn
	this->pointUsedRatio = double(j)/double(requestedPts.features.cols());
	this->weightedPointUsedRatio /= double(requestedPts.features.cols());
	
	assert(dimFeat == sourcePts.features.rows());
	const int dimSourDesc = sourcePts.descriptors.rows();
	
	Features associatedFeat(dimFeat, pointsCount);
	Descriptors associatedDesc;
	if(dimSourDesc > 0)
		associatedDesc = Descriptors(dimSourDesc, pointsCount);

	// Fetch matched points
	for (int i = 0; i < pointsCount; ++i)
	{
		const int refIndex(keptMatches.ids(i));
		associatedFeat.col(i) = sourcePts.features.block(0, refIndex, dimFeat, 1);
		
		if(dimSourDesc > 0)
			associatedDesc.col(i) = sourcePts.descriptors.block(0, refIndex, dimSourDesc, 1);
	}

	DataPoints keptPts;
	keptPts.features = keptFeat;
	keptPts.descriptors = keptDesc;
	keptPts.descriptorLabels = requestedPts.descriptorLabels;

	DataPoints associatedPts;
	associatedPts.features = associatedFeat;
	associatedPts.descriptors = associatedDesc;
	associatedPts.descriptorLabels = sourcePts.descriptorLabels;

	return ErrorElements(keptPts, associatedPts, keptWeights, keptMatches);
		
}


template struct ErrorMinimizersImpl<float>::PointToPlaneErrorMinimizer;
template struct ErrorMinimizersImpl<double>::PointToPlaneErrorMinimizer;

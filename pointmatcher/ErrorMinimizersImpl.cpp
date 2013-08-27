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

#include "ErrorMinimizersImpl.h"
#include "PointMatcherPrivate.h"
#include "Functions.h"

#include "Eigen/SVD"
#include <iostream>

using namespace Eigen;
using namespace std;
using namespace PointMatcherSupport;

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
	typedef typename Matches::Ids Ids;
	
	assert(matches.ids.rows() > 0);

	typename ErrorMinimizer::ErrorElements& mPts = this->getMatchedPoints(filteredReading, filteredReference, matches, outlierWeights);
	
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

template<typename T>
T ErrorMinimizersImpl<T>::PointToPointErrorMinimizer::getOverlap() const
{
	const int nbPoints = this->lastErrorElements.reading.features.cols();
	if(nbPoints == 0)
	{
		throw std::runtime_error("Error, last error element empty. Error minimizer needs to be called at least once before using this method.");
	}

	if (!this->lastErrorElements.reading.descriptorExists("simpleSensorNoise"))
	{
		LOG_INFO_STREAM("PointToPointErrorMinimizer - warning, no sensor noise found. Using best estimate given outlier rejection instead.");
		return this->weightedPointUsedRatio;
	}

	const BOOST_AUTO(noises, this->lastErrorElements.reading.getDescriptorViewByName("simpleSensorNoise"));
	int count = 0;
	for(int i=0; i < nbPoints; i++)
	{
		const T dist = (this->lastErrorElements.reading.features.col(i) - this->lastErrorElements.reference.features.col(i)).norm();
		if(dist < noises(0,i))
			count++;
	}

	return count/nbPoints;
}

template struct ErrorMinimizersImpl<float>::PointToPointErrorMinimizer;
template struct ErrorMinimizersImpl<double>::PointToPointErrorMinimizer;


// Point To PLANE ErrorMinimizer
template<typename T>
ErrorMinimizersImpl<T>::PointToPlaneErrorMinimizer::PointToPlaneErrorMinimizer(const Parameters& params):
	ErrorMinimizer("PointToPlaneErrorMinimizer", PointToPlaneErrorMinimizer::availableParameters(), params),
	force2D(Parametrizable::get<T>("force2D"))
{
}


template<typename T>
typename PointMatcher<T>::TransformationParameters ErrorMinimizersImpl<T>::PointToPlaneErrorMinimizer::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const OutlierWeights& outlierWeights,
	const Matches& matches)
{
	typedef typename PointMatcher<T>::ConvergenceError ConvergenceError;
	typedef typename Matches::Ids Ids;

	assert(matches.ids.rows() > 0);

	// Fetch paired points
	typename ErrorMinimizer::ErrorElements& mPts = this->getMatchedPoints(filteredReading, filteredReference, matches, outlierWeights);

	const int dim = mPts.reading.features.rows();
	const int nbPts = mPts.reading.features.cols();

	// Adjust if the user forces 2D minimization on XY-plane
	int forcedDim = dim - 1;
	if(force2D && dim == 4)
	{
		mPts.reading.features.conservativeResize(3, Eigen::NoChange);
		mPts.reading.features.row(2) = Matrix::Ones(1, nbPts);
		mPts.reference.features.conservativeResize(3, Eigen::NoChange);
		mPts.reference.features.row(2) = Matrix::Ones(1, nbPts);
		forcedDim = dim - 2;
	}

	// Fetch normal vectors of the reference point cloud (with adjustment if needed)
	const BOOST_AUTO(normalRef, mPts.reference.getDescriptorViewByName("normals").topRows(forcedDim));

	// Note: Normal vector must be precalculated to use this error. Use appropriate input filter.
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
		// TODO: handle that properly
		//throw ConvergenceError("encountered singular while minimizing point to plane distance");
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
	if(dim == 4 && !force2D)
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

		if(force2D)
		{
			mOut = Matrix::Identity(dim, dim);
			mOut.corner(TopLeft, 2, 2) = transform.matrix().corner(TopLeft, 2, 2);
			mOut.corner(TopRight, 2, 1) = transform.matrix().corner(TopRight, 2, 1);
		}
		else
		{
			mOut = transform.matrix();
		}
	}
	return mOut; 
}

template<typename T>
T ErrorMinimizersImpl<T>::PointToPlaneErrorMinimizer::getOverlap() const
{
	const int nbPoints = this->lastErrorElements.reading.features.cols();
	const int dim = this->lastErrorElements.reading.features.rows();
	if(nbPoints == 0)
	{
		throw std::runtime_error("Error, last error element empty. Error minimizer needs to be called at least once before using this method.");
	}
	
	if (!this->lastErrorElements.reading.descriptorExists("simpleSensorNoise") ||
		!this->lastErrorElements.reading.descriptorExists("normals"))
	{
		LOG_INFO_STREAM("PointToPlaneErrorMinimizer - warning, no sensor noise or normals found. Using best estimate given outlier rejection instead.");
		return this->weightedPointUsedRatio;
	}

	const BOOST_AUTO(noises, this->lastErrorElements.reading.getDescriptorViewByName("simpleSensorNoise"));
	const BOOST_AUTO(normals, this->lastErrorElements.reading.getDescriptorViewByName("normals"));
	int count = 0;
	for(int i=0; i < nbPoints; i++)
	{
		if(this->lastErrorElements.matches.dists(0, i) != numeric_limits<T>::infinity())
		{
			const Vector d = this->lastErrorElements.reading.features.col(i) - this->lastErrorElements.reference.features.col(i);
			const Vector n = normals.col(i);
			const T projectionDist = d.head(dim-1).dot(n.normalized());
			if(anyabs(projectionDist) < noises(0,i))
				count++;
		}
	}

	return (T)count/(T)nbPoints;
}

template struct ErrorMinimizersImpl<float>::PointToPlaneErrorMinimizer;
template struct ErrorMinimizersImpl<double>::PointToPlaneErrorMinimizer;

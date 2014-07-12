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
	//NOTE: computing overlap of 2 point clouds can be complicated due to
	// the sparse nature of the representation. Here is only an estimate 
	// of the true overlap.
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

	const Vector dists = (this->lastErrorElements.reading.features - this->lastErrorElements.reference.features).colwise().norm();
	const T mean = dists.sum()/nbPoints;

	int count = 0;
	for(int i=0; i < nbPoints; i++)
	{
		if(dists(i) < (mean + noises(0,i)))
		{
			count++;
		}
	}

	return (T)count/(T)nbPoints;
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

	// wF = [weights*cross, weights*normals]
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
    	        wF.row(i + cross.rows()) = mPts.weights.cwise() * normalRef.row(i);
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
		// PLEASE DONT USE EULAR ANGLES!!!!
		// Rotation in Eular angles follow roll-pitch-yaw (1-2-3) rule
		/*transform = Eigen::AngleAxis<T>(x(0), Eigen::Matrix<T,1,3>::UnitX())
				* Eigen::AngleAxis<T>(x(1), Eigen::Matrix<T,1,3>::UnitY())
				* Eigen::AngleAxis<T>(x(2), Eigen::Matrix<T,1,3>::UnitZ());*/

		transform = Eigen::AngleAxis<T>(x.head(3).norm(),x.head(3).normalized());

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



// Point To POINT WITH COV ErrorMinimizer
template<typename T>
ErrorMinimizersImpl<T>::PointToPointWithCovErrorMinimizer::PointToPointWithCovErrorMinimizer(const Parameters& params):
	ErrorMinimizer("PointToPointWithCovErrorMinimizer", PointToPointWithCovErrorMinimizer::availableParameters(), params),
	sensorStdDev(Parametrizable::get<T>("sensorStdDev"))
{
}

template<typename T>
typename PointMatcher<T>::TransformationParameters ErrorMinimizersImpl<T>::PointToPointWithCovErrorMinimizer::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const OutlierWeights& outlierWeights,
	const Matches& matches)
{	
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

	this->covMatrix = this->estimateCovariance(filteredReading, filteredReference, matches, outlierWeights, result);

	return result;
}

template<typename T>
typename ErrorMinimizersImpl<T>::Matrix
ErrorMinimizersImpl<T>::PointToPointWithCovErrorMinimizer::estimateCovariance(const DataPoints& reading, const DataPoints& reference, const Matches& matches, const OutlierWeights& outlierWeights, const TransformationParameters& transformation)
{
	int max_nbr_point = outlierWeights.cols();

	Matrix covariance(Matrix::Zero(6,6));
	Matrix J_hessian(Matrix::Zero(6,6));
	Matrix d2J_dReadingdX(Matrix::Zero(6, max_nbr_point));
	Matrix d2J_dReferencedX(Matrix::Zero(6, max_nbr_point));

	Vector reading_point(Vector::Zero(3));
	Vector reference_point(Vector::Zero(3));
	Vector normal(3);
	Vector reading_direction(Vector::Zero(3));
	Vector reference_direction(Vector::Zero(3));

	normal << 1.0, 1.0, 1.0;    // Used for point-to-point computation

	T beta = -asin(transformation(2,0));
	T alpha = atan2(transformation(2,1), transformation(2,2));
	T gamma = atan2(transformation(1,0)/cos(beta), transformation(0,0)/cos(beta));
	T t_x = transformation(0,3);
	T t_y = transformation(1,3);
	T t_z = transformation(2,3);

	Vector tmp_vector_6(Vector::Zero(6));

	int valid_points_count = 0;

	for(int i = 0; i < max_nbr_point; ++i)
	{
		if (outlierWeights(0,i) > 0.0)
		{
			reading_point = reading.features.block(0,i,3,1);
			int reference_idx = matches.ids(0,i);
			reference_point = reference.features.block(0,reference_idx,3,1);

			T reading_range = reading_point.norm();
			reading_direction = reading_point / reading_range;
			T reference_range = reference_point.norm();
			reference_direction = reference_point / reference_range;

			T n_alpha = normal(2)*reading_direction(1) - normal(1)*reading_direction(2);
			T n_beta = normal(0)*reading_direction(2) - normal(2)*reading_direction(0);
			T n_gamma = normal(1)*reading_direction(0) - normal(0)*reading_direction(1);

			T E = normal(0)*(reading_point(0) - gamma*reading_point(1) + beta*reading_point(2) + t_x - reference_point(0));
			E +=  normal(1)*(gamma*reading_point(0) + reading_point(1) - alpha*reading_point(2) + t_y - reference_point(1));
			E +=  normal(2)*(-beta*reading_point(0) + alpha*reading_point(1) + reading_point(2) + t_z - reference_point(2));

			T N_reading = normal(0)*(reading_direction(0) - gamma*reading_direction(1) + beta*reading_direction(2));
			N_reading +=  normal(1)*(gamma*reading_direction(0) + reading_direction(1) - alpha*reading_direction(2));
			N_reading +=  normal(2)*(-beta*reading_direction(0) + alpha*reading_direction(1) + reading_direction(2));

			T N_reference = -(normal(0)*reference_direction(0) + normal(1)*reference_direction(1) + normal(2)*reference_direction(2));

			// update the hessian and d2J/dzdx
			tmp_vector_6 << normal(0), normal(1), normal(2), reading_range * n_alpha, reading_range * n_beta, reading_range * n_gamma;

			J_hessian += tmp_vector_6 * tmp_vector_6.transpose();

			tmp_vector_6 << normal(0) * N_reading, normal(1) * N_reading, normal(2) * N_reading, n_alpha * (E + reading_range * N_reading), n_beta * (E + reading_range * N_reading), n_gamma * (E + reading_range * N_reading);

			d2J_dReadingdX.block(0,valid_points_count,6,1) = tmp_vector_6;

			tmp_vector_6 << normal(0) * N_reference, normal(1) * N_reference, normal(2) * N_reference, reference_range * n_alpha * N_reference, reference_range * n_beta * N_reference, reference_range * n_gamma * N_reference;

			d2J_dReferencedX.block(0,valid_points_count,6,1) = tmp_vector_6;

			valid_points_count++;
		} // if (outlierWeights(0,i) > 0.0)
	}

	Matrix d2J_dZdX(Matrix::Zero(6, 2 * valid_points_count));
	d2J_dZdX.block(0,0,6,valid_points_count) = d2J_dReadingdX.block(0,0,6,valid_points_count);
	d2J_dZdX.block(0,valid_points_count,6,valid_points_count) = d2J_dReferencedX.block(0,0,6,valid_points_count);

	Matrix inv_J_hessian = J_hessian.inverse();

	covariance = d2J_dZdX * d2J_dZdX.transpose();
	covariance = inv_J_hessian * covariance * inv_J_hessian;

	return (sensorStdDev * sensorStdDev) * covariance;
}

template<typename T>
T ErrorMinimizersImpl<T>::PointToPointWithCovErrorMinimizer::getOverlap() const
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

	return (T)count/(T)nbPoints;
}

template<typename T>
typename ErrorMinimizersImpl<T>::Matrix ErrorMinimizersImpl<T>::PointToPointWithCovErrorMinimizer::getCovariance() const
{
  return covMatrix;
}

template struct ErrorMinimizersImpl<float>::PointToPointWithCovErrorMinimizer;
template struct ErrorMinimizersImpl<double>::PointToPointWithCovErrorMinimizer;


// Point To PLANE WITH COV ErrorMinimizer
template<typename T>
ErrorMinimizersImpl<T>::PointToPlaneWithCovErrorMinimizer::PointToPlaneWithCovErrorMinimizer(const Parameters& params):
	ErrorMinimizer("PointToPlaneWithCovErrorMinimizer", PointToPlaneWithCovErrorMinimizer::availableParameters(), params),
	force2D(Parametrizable::get<T>("force2D")),
	sensorStdDev(Parametrizable::get<T>("sensorStdDev"))
{
}


template<typename T>
typename PointMatcher<T>::TransformationParameters ErrorMinimizersImpl<T>::PointToPlaneWithCovErrorMinimizer::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const OutlierWeights& outlierWeights,
	const Matches& matches)
{
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

	// wF = [weights*cross, weight*normals]
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
       	        wF.row(i + cross.rows()) = mPts.weights.cwise() * normalRef.row(i);
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
		// IT IS NOT CORRECT TO USE EULER ANGLES!
		// Rotation in Eular angles follow roll-pitch-yaw (1-2-3) rule
		/*transform = Eigen::AngleAxis<T>(x(0), Eigen::Matrix<T,1,3>::UnitX())
				* Eigen::AngleAxis<T>(x(1), Eigen::Matrix<T,1,3>::UnitY())
				* Eigen::AngleAxis<T>(x(2), Eigen::Matrix<T,1,3>::UnitZ()); */
		// Reverse roll-pitch-yaw conversion, very useful piece of knowledge, keep it with you all time!
		/*const T pitch = -asin(transform(2,0));
		const T roll = atan2(transform(2,1), transform(2,2));
		const T yaw = atan2(transform(1,0) / cos(pitch), transform(0,0) / cos(pitch));
		std::cerr << "d angles" << x(0) - roll << ", " << x(1) - pitch << "," << x(2) - yaw << std::endl;*/

		transform = Eigen::AngleAxis<T>(x.head(3).norm(),x.head(3).normalized());
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

	this->covMatrix = this->estimateCovariance(filteredReading, filteredReference, matches, outlierWeights, mOut);

	return mOut; 
}

template<typename T>
typename ErrorMinimizersImpl<T>::Matrix
ErrorMinimizersImpl<T>::PointToPlaneWithCovErrorMinimizer::estimateCovariance(const DataPoints& reading, const DataPoints& reference, const Matches& matches, const OutlierWeights& outlierWeights, const TransformationParameters& transformation)
{
	int max_nbr_point = outlierWeights.cols();

	Matrix covariance(Matrix::Zero(6,6));
	Matrix J_hessian(Matrix::Zero(6,6));
	Matrix d2J_dReadingdX(Matrix::Zero(6, max_nbr_point));
	Matrix d2J_dReferencedX(Matrix::Zero(6, max_nbr_point));

	Vector reading_point(Vector::Zero(3));
	Vector reference_point(Vector::Zero(3));
	Vector normal(3);
	Vector reading_direction(Vector::Zero(3));
	Vector reference_direction(Vector::Zero(3));

	Matrix normals = reference.getDescriptorViewByName("normals");

	if (normals.rows() < 3)    // Make sure there are normals in DataPoints
		return Matrix::Zero(6,6);

	T beta = -asin(transformation(2,0));
	T alpha = atan2(transformation(2,1), transformation(2,2));
	T gamma = atan2(transformation(1,0)/cos(beta), transformation(0,0)/cos(beta));
	T t_x = transformation(0,3);
	T t_y = transformation(1,3);
	T t_z = transformation(2,3);

	Vector tmp_vector_6(Vector::Zero(6));

	int valid_points_count = 0;

	for(int i = 0; i < max_nbr_point; ++i)
	{
		if (outlierWeights(0,i) > 0.0)
		{
			reading_point = reading.features.block(0,i,3,1);
			int reference_idx = matches.ids(0,i);
			reference_point = reference.features.block(0,reference_idx,3,1);

			normal = normals.block(0,reference_idx,3,1);

			T reading_range = reading_point.norm();
			reading_direction = reading_point / reading_range;
			T reference_range = reference_point.norm();
			reference_direction = reference_point / reference_range;

			T n_alpha = normal(2)*reading_direction(1) - normal(1)*reading_direction(2);
			T n_beta = normal(0)*reading_direction(2) - normal(2)*reading_direction(0);
			T n_gamma = normal(1)*reading_direction(0) - normal(0)*reading_direction(1);

			T E = normal(0)*(reading_point(0) - gamma*reading_point(1) + beta*reading_point(2) + t_x - reference_point(0));
			E +=  normal(1)*(gamma*reading_point(0) + reading_point(1) - alpha*reading_point(2) + t_y - reference_point(1));
			E +=  normal(2)*(-beta*reading_point(0) + alpha*reading_point(1) + reading_point(2) + t_z - reference_point(2));

			T N_reading = normal(0)*(reading_direction(0) - gamma*reading_direction(1) + beta*reading_direction(2));
			N_reading +=  normal(1)*(gamma*reading_direction(0) + reading_direction(1) - alpha*reading_direction(2));
			N_reading +=  normal(2)*(-beta*reading_direction(0) + alpha*reading_direction(1) + reading_direction(2));

			T N_reference = -(normal(0)*reference_direction(0) + normal(1)*reference_direction(1) + normal(2)*reference_direction(2));

			// update the hessian and d2J/dzdx
			tmp_vector_6 << normal(0), normal(1), normal(2), reading_range * n_alpha, reading_range * n_beta, reading_range * n_gamma;

			J_hessian += tmp_vector_6 * tmp_vector_6.transpose();

			tmp_vector_6 << normal(0) * N_reading, normal(1) * N_reading, normal(2) * N_reading, n_alpha * (E + reading_range * N_reading), n_beta * (E + reading_range * N_reading), n_gamma * (E + reading_range * N_reading);

			d2J_dReadingdX.block(0,valid_points_count,6,1) = tmp_vector_6;

			tmp_vector_6 << normal(0) * N_reference, normal(1) * N_reference, normal(2) * N_reference, reference_range * n_alpha * N_reference, reference_range * n_beta * N_reference, reference_range * n_gamma * N_reference;

			d2J_dReferencedX.block(0,valid_points_count,6,1) = tmp_vector_6;

			valid_points_count++;
		} // if (outlierWeights(0,i) > 0.0)
	}

	Matrix d2J_dZdX(Matrix::Zero(6, 2 * valid_points_count));
	d2J_dZdX.block(0,0,6,valid_points_count) = d2J_dReadingdX.block(0,0,6,valid_points_count);
	d2J_dZdX.block(0,valid_points_count,6,valid_points_count) = d2J_dReferencedX.block(0,0,6,valid_points_count);

	Matrix inv_J_hessian = J_hessian.inverse();

	covariance = d2J_dZdX * d2J_dZdX.transpose();
	covariance = inv_J_hessian * covariance * inv_J_hessian;

	return (sensorStdDev * sensorStdDev) * covariance;
}



template<typename T>
T ErrorMinimizersImpl<T>::PointToPlaneWithCovErrorMinimizer::getOverlap() const
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

template<typename T>
typename ErrorMinimizersImpl<T>::Matrix ErrorMinimizersImpl<T>::PointToPlaneWithCovErrorMinimizer::getCovariance() const
{
  return covMatrix;
}

template struct ErrorMinimizersImpl<float>::PointToPlaneWithCovErrorMinimizer;
template struct ErrorMinimizersImpl<double>::PointToPlaneWithCovErrorMinimizer;


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

#include <iostream>
#include <pointmatcher/PointMatcher.h>

#include "Eigen/QR"
#include "Eigen/Eigenvalues"

#include "ErrorMinimizersImpl.h"
#include "PointMatcherPrivate.h"
#include "Functions.h"

using namespace Eigen;
using namespace std;
using namespace PointMatcherSupport;

typedef PointMatcherSupport::Parametrizable Parametrizable;
typedef PointMatcherSupport::Parametrizable P;
typedef Parametrizable::Parameters Parameters;
typedef Parametrizable::ParameterDoc ParameterDoc;
typedef Parametrizable::ParametersDoc ParametersDoc;

template<typename T>
PointToGaussianErrorMinimizer<T>::PointToGaussianErrorMinimizer(const Parameters& params):
	PointToPlaneErrorMinimizer<T>(PointToGaussianErrorMinimizer::availableParameters(), params),
	confidenceInPenalties(Parametrizable::get<T>("confidenceInPenalties")),
	force2D(Parametrizable::get<bool>("force2D"))
{
}

template<typename T>
typename PointMatcher<T>::TransformationParameters PointToGaussianErrorMinimizer<T>::compute(const ErrorElements& mPts_const)
{
	auto errElements = convertCovariancesToNormals(mPts_const);
	typename PointMatcher<T>::TransformationParameters out = PointToPlaneErrorMinimizer<T>::compute_in_place(errElements);

	return out;
}


template<typename T>
typename PointToGaussianErrorMinimizer<T>::ErrorElements PointToGaussianErrorMinimizer<T>::convertCovariancesToNormals(const ErrorElements& mPts_const) const
{
	typedef typename DataPoints::View View;
	typedef typename DataPoints::ConstView ConstView;
	typedef typename PointMatcher<T>::ConvergenceError ConvergenceError;

	ErrorElements mPts = mPts_const;
	const size_t dim(mPts_const.reference.features.rows() - 1);
	const size_t nbPenalty(mPts_const.penalties.size());
	const size_t newSize(dim * (mPts.weights.cols())); // + nbPenalty

	mPts.reference.conservativeResize(newSize);
	mPts.reading.conservativeResize(newSize);
	mPts.weights.conservativeResize(Eigen::NoChange, newSize + dim * nbPenalty);


	std::cout << "mPts.reference:" << std::endl << mPts.reference.features.block(0, 0, 4, 300) << std::endl;
	std::cout << "mPts.reading:" << std::endl << mPts.reading.features.block(0, 0, 4, 300) << std::endl;

	if (!mPts.reference.descriptorExists("normals")) {
		Labels cloudLabels;
		cloudLabels.push_back(Label("normals", dim));
		// Reserve memory
		mPts.reference.allocateDescriptors(cloudLabels);
//		mPts.reference.getDescriptorViewByName("normals").setZero();
	}
	View normals = mPts.reference.getDescriptorViewByName("normals");
	ConstView eigVectors = mPts_const.reference.getDescriptorViewByName("eigVectors");
	ConstView eigValues = mPts_const.reference.getDescriptorViewByName("eigValues");
	std::cout << "normals:" << std::endl << normals.block(0, 0, 3, 300) << std::endl;

	if ((eigValues.array() != 0.0).any()) {
		throw ConvergenceError("PointToGaussian(): Some of the eigen values are negative.");
	}
	for (long i = 0; i < mPts_const.reference.features.cols(); ++i) {
		mPts.reading.features.block(0, dim * i, dim + 1, dim) = mPts_const.reading.features.col(i).replicate(1, dim);
		mPts.reference.features.block(0, dim * i, dim + 1, dim) = mPts_const.reference.features.col(i).replicate(1, dim);

		// Convert a vector into a matrix
		const Matrix matEigVectors = Eigen::Map<const Matrix>(eigVectors.col(i).data(), dim, dim);
		normals.block(0, dim * i, dim, dim) = matEigVectors;

		// A eigen value of zero will have infinite weight.
		assert((eigValues.col(i).array() != 0.0).any());
		for (unsigned k = 0; k < dim; ++k) {
			if (eigValues(k, i) < 0.0)
				std::cout << "Eigval neg: " << eigValues.col(i) << std::endl;
		}
		mPts.weights.block(0, dim * i, 1, dim) = eigValues.col(i).array().inverse().transpose();
	}

	if (mPts_const.penalties.size() > 0) {
		// It's hard to add points that have descriptor to a Datapoints, so we create a new Datapoints for the new points and then concatenate it
		Matrix penaltiesPtsRead(dim + 1, nbPenalty * dim);
		Matrix penaltiesPtsReference(dim + 1, nbPenalty * dim);
		Matrix penaltiesNormals(dim, nbPenalty * dim);

		for (size_t i = 0; i < mPts_const.penalties.size(); ++i) {
			// To minimize both the distances from the point cloud and the penalties at the same time we convert the penalties to fake pairs of point/normal.
			// For each penalty n fake pairs of point/normal will be created, where n is the dimensions of the covariance.
			// The eigen decomposition of the penalty's covariance give us the following:
			// W: Covariance a n by n matrix
			// W = N * L * N^T
			// N = [n1 n2 n3]
			// where L is a diagonal matrix of the eigen value and N is a rotation matrix.
			// n1, n2, n3 are column vectors. The fake pairs will use these vectors as normal.
			// For the fake points of the reference and the reading the translation part of penalty tf matrix and the current transformation matrix will be used respectively.
			const auto &penalty = mPts_const.penalties[i];
			const Eigen::EigenSolver<Matrix> solver(penalty.second);
			const Matrix eigenVec = solver.eigenvectors().real();
			const Vector eigenVal = solver.eigenvalues().real();
	//		std::cout<< "Eigen Vector" << eigenVec << std::endl;
	//		std::cout<< "Eigen Value" << eigenVal << std::endl;

			const Vector transInRef(penalty.first.col(dim));
			const Vector transInRead((mPts_const.T_refMean_iter.col(dim)));

			penaltiesPtsRead.block(0, dim * i, dim + 1, dim) = transInRead.replicate(1, dim);
			penaltiesPtsReference.block(0, dim * i, dim + 1, dim) = transInRef.replicate(1, dim);
			penaltiesNormals.block(0, dim * i, dim, dim) = eigenVec;

			// The eigen value are the variance for each eigen vector.
			mPts.weights.bottomRightCorner(1, dim) = eigenVal.diagonal().array().inverse().transpose();
		}
		const Labels normalLabel({Label("normals", dim)});
		const DataPoints penaltiesReference(penaltiesPtsReference, mPts_const.reference.featureLabels, penaltiesNormals, normalLabel);
		const DataPoints penaltiesRead(penaltiesPtsRead, mPts_const.reading.featureLabels);

		std::cout << "before concat normals:" << std::endl << normals.block(0, 0, 3, 300) << std::endl;
		std::cout << "before concat penaltiesNormals:" << std::endl << penaltiesNormals.block(0, 0, 3, 300) << std::endl;
		mPts.reference.concatenate(penaltiesReference);
		mPts.reading.concatenate(penaltiesRead);
	}
	std::cout << "end not normalize mPts.weights:" << std::endl << mPts.weights.block(0, 0, 1, 300) << std::endl;
//	mPts.weights = mPts.weights.normalized();
	std::cout << "end mPts.weights:" << std::endl << mPts.weights.block(0, 0, 1, 300) << std::endl;
	std::cout << "end mPts.weights max:" << std::endl << mPts.weights.maxCoeff() << std::endl;
	std::cout << "end mPts.weights mean:" << std::endl << mPts.weights.mean() << std::endl;
	std::cout << "end mPts.weights min:" << std::endl << mPts.weights.minCoeff() << std::endl;
	std::cout << "end mPts.reference:" << std::endl << mPts.reference.features.block(0, 0, 4, 300) << std::endl;
	std::cout << "end mPts.reading:" << std::endl << mPts.reading.features.block(0, 0, 4, 300) << std::endl;
	std::cout << "end normals:" << std::endl << normals.block(0, 0, 3, 300)  << std::endl;
	return mPts;
}

template<typename T>
T PointToGaussianErrorMinimizer<T>::getResidualError(
				const DataPoints& filteredReading,
				const DataPoints& filteredReference,
				const OutlierWeights& outlierWeights,
				const Matches& matches,
				const Penalties& penalties,
				const TransformationParameters& T_refMean_iter) const
{
	assert(matches.ids.rows() > 0);

	// Fetch paired points
	const typename ErrorMinimizer::ErrorElements mPts(filteredReading, filteredReference, outlierWeights, matches, penalties, T_refMean_iter);
	auto mPtsNormals = convertCovariancesToNormals(mPts);

	return PointToGaussianErrorMinimizer::computeResidualError(mPtsNormals, force2D);
}

template struct PointToGaussianErrorMinimizer<float>;
template struct PointToGaussianErrorMinimizer<double>;

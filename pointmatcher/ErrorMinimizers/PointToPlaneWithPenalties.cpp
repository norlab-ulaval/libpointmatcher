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
PointToPlaneWithPenaltiesErrorMinimizer<T>::PointToPlaneWithPenaltiesErrorMinimizer(const Parameters& params):
	PointToPlaneErrorMinimizer<T>(PointToPlaneWithPenaltiesErrorMinimizer::availableParameters(), params),
	confidenceInPenalties(Parametrizable::get<T>("confidenceInPenalties"))
{
}

template<typename T>
typename PointMatcher<T>::TransformationParameters PointToPlaneWithPenaltiesErrorMinimizer<T>::compute(const ErrorElements& mPts_const)
{
	ErrorElements mPts = mPts_const;
	const size_t dim(mPts_const.reference.features.rows() - 1);
	const size_t nbPenalty(mPts_const.penalties.size());
	const size_t nbPoints = mPts.weights.cols();

	mPts.weights = mPts.weights / mPts.weights.norm();
	mPts.weights.conservativeResize(Eigen::NoChange, nbPenalty * dim + nbPoints);

	// It's hard to add points with descriptor to a Datapoints, so we create a new Datapoints for the new points and then concatenate it
	Matrix penaltiesPtsRead(dim + 1, nbPenalty * dim);
	Matrix penaltiesPtsReference(dim + 1, nbPenalty * dim);
	Matrix penaltiesNormals(dim, nbPenalty * dim);

	Matrix location, cov, offset;
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
		std::tie(location, cov, offset) = mPts_const.penalties[i];
		const Eigen::EigenSolver<Matrix> solver(cov);
		const Matrix eigenVec = solver.eigenvectors().real();
		const Vector eigenVal = solver.eigenvalues().real();
//		std::cout<< "Eigen Vector" << eigenVec << std::endl;
//		std::cout<< "Eigen Value" << eigenVal << std::endl;

		const Vector transInRef(location.col(dim));
		const Vector transInRead(mPts_const.T_refMean_iter * (offset).col(dim));

		penaltiesPtsRead.block(0, dim * i, dim + 1, dim) = transInRead.replicate(1, dim);
		penaltiesPtsReference.block(0, dim * i, dim + 1, dim) = transInRef.replicate(1, dim);
		penaltiesNormals.block(0, dim * i, dim, dim) = eigenVec;

		// The eigen value are the variance for each eigen vector.
		mPts.weights.block(0, nbPoints + dim * i, 1, dim) = eigenVal.diagonal().array().inverse().transpose();
//		std::cout<< "penaltiesNormals" << std::endl << penaltiesNormals << std::endl;
//		std::cout<< "penaltiesPtsRead" << std::endl << penaltiesPtsRead << std::endl;

	}
	const Labels normalLabel({Label("normals", dim)});
	const DataPoints penaltiesReference(penaltiesPtsReference, mPts_const.reference.featureLabels, penaltiesNormals, normalLabel);
	const DataPoints penaltiesRead(penaltiesPtsRead, mPts_const.reading.featureLabels);

	mPts.reference.concatenate(penaltiesReference);
	mPts.reading.concatenate(penaltiesRead);

//	std::cout<< "mPts.weights" << std::endl << mPts.weights << std::endl;
//	std::cout<< "mPts.reference" << std::endl << mPts.reference.features << std::endl;
//	std::cout<< "mPts.reading" << std::endl << mPts.reading.features << std::endl << std::endl;
//	std::cout<< "mPts.reference.descriptors" << std::endl << mPts.reference.descriptors << std::endl << std::endl;

	typename PointMatcher<T>::TransformationParameters out = PointToPlaneErrorMinimizer<T>::compute_in_place(mPts);

	return out;
}


template<typename T>
T PointToPlaneWithPenaltiesErrorMinimizer<T>::getResidualError(
				const DataPoints& filteredReading,
				const DataPoints& filteredReference,
				const OutlierWeights& outlierWeights,
				const Matches& matches,
				const Penalties& penalties,
				const TransformationParameters& T_refMean_iter) const
{
	assert(matches.ids.rows() > 0);

	// Fetch paired points
	typename ErrorMinimizer::ErrorElements mPts(filteredReading, filteredReference, outlierWeights, matches, penalties, T_refMean_iter);
	mPts.weights.row(0) = mPts.weights.row(0) / mPts.weights.row(0).norm();
	T pointToPlaneErr = PointToPlaneErrorMinimizer<T>::computeResidualError(mPts, false);

	// HACK FSR 2019
	T penalitiesErr = 0.0;
	for (const Penalty& p: penalties) {
		Vector e = T_refMean_iter.topRightCorner(3, 1) - std::get<0>(p).topRightCorner(3, 1);
		penalitiesErr += e.transpose() * std::get<1>(p).transpose() * e;
	}

	return pointToPlaneErr + penalitiesErr;
}

template struct PointToPlaneWithPenaltiesErrorMinimizer<float>;
template struct PointToPlaneWithPenaltiesErrorMinimizer<double>;

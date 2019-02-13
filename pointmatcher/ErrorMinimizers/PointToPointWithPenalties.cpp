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

#include <pointmatcher/PointMatcher.h>
#include "ErrorMinimizersImpl.h"
#include "PointMatcherPrivate.h"
#include "Eigen/SVD"

using namespace Eigen;

template<typename T>
PointToPointWithPenaltiesErrorMinimizer<T>::PointToPointWithPenaltiesErrorMinimizer(const Parameters& params):
	PointToPointErrorMinimizer<T>("PointToPointWithPenaltiesErrorMinimizer", availableParameters(), params),
	confidenceInPenalties(Parametrizable::get<T>("confidenceInPenalties"))
{
}

template<typename T>
typename PointMatcher<T>::TransformationParameters PointToPointWithPenaltiesErrorMinimizer<T>::compute(const ErrorElements& mPts_const)
{
	ErrorElements mPts = mPts_const;
	const size_t dim(mPts.reference.features.rows() - 1);
	const size_t nbPenalty(mPts_const.penalties.size());
	const size_t refSize(mPts_const.reference.features.cols());
	const size_t readSize(mPts_const.reading.features.cols());


	mPts.reference.features.conservativeResize(Eigen::NoChange, nbPenalty * dim + refSize);
	mPts.reading.features.conservativeResize(Eigen::NoChange, nbPenalty * dim + readSize);
	mPts.weights.conservativeResize(Eigen::NoChange, nbPenalty * dim + mPts.weights.cols());

	// FIXME: Add assert for that make sure the covariance is diagonal and penalties only in translation
	if (nbPenalty > 0) {
		const T pointsWeightSum = mPts_const.weights.sum();
		T penaltiesWeightSum = 0;
		for (size_t i = 0; i < mPts_const.penalties.size(); ++i) {
			// To minimize both the distances from the point cloud and the penalties at the same time we convert the penalties to fake points.
			// These fake points will have a weight corresponding to a element in the covariance's diagonal.
			// For instance, imagine a penalty with the following parameters:
			// tf = [ 1 0 0 x]  cov = [ σ0  0  0]
			//      [ 0 1 0 y]        [  0 σ1  0]
			//      [ 0 0 1 z]        [  0  0 σ2]
			//      [ 0 0 0 1]
			// To represent this penalty with a point to point minimization, we add 3 fakes points with corresponding weight:
			// pts_fake = [x 0 0]   W = [1/σ0 1/σ1 1/σ2]
			//            [0 y 0]
			//            [0 0 z]
			// The fake points are added to the reference. Meanwhile the corresponding fake point for the reading corresponds
			// to the origin. At each iteration the reading and its origin are moved, thus the fake points for the reading are:
			// pts_fake = [tx  0  0]
			//            [ 0 ty  0]
			//            [ 0  0 tz]
			// where [tx, ty, tz] is the translation part of the transformation matrix at the current iteration (T_refMean_iter).
			// Note: for this hack to works the covariance matrix must be diagonal and penalties must be only in translation.
			const auto &penalty = mPts_const.penalties[i];
			// Take translation part of the penalty's transformation matrix and create a diagonal matrix with it
			Matrix transInRef(penalty.first.col(dim).asDiagonal());

			// We must move the reading origin [0, 0, 0], we extract the translation part of the reading's tf,
			// since apply a tf on a zero vector is the same as extracting the translation part of the tf.
			Matrix transInRead((mPts_const.T_refMean_iter.col(dim)).asDiagonal());

			transInRef.row(dim).setOnes();
			transInRead.row(dim).setOnes();
			// Convert sigma on the covariance matrix diagonal to 1/sigma
			const Vector penaltiesWeight = penalty.second.diagonal().array().inverse().matrix();
			mPts.reference.features.block(0, refSize + dim * i, dim, dim) = transInRef;
			mPts.reading.features.block(0, readSize + dim * i, dim, dim) = transInRead;
			mPts.weights.block(0, readSize + dim * i, 1, dim) = penaltiesWeight.transpose();
			penaltiesWeightSum += penaltiesWeight.sum();
		}
		// Normalize the weight so the confidenceInPenalties determine the influence of the penalties on the minimization
//		std::cout <<"before mPts.weight:" << std::endl << mPts.weights << " penaltiesWeightSum "<< penaltiesWeightSum << std::endl;
//		mPts.weights.block(0,        0, 1,        readSize) *= (1 - confidenceInPenalties) / pointsWeightSum;
//		mPts.weights.block(0, readSize, 1, dim * nbPenalty) *= confidenceInPenalties / penaltiesWeightSum;
	}

	typename PointMatcher<T>::TransformationParameters result = PointToPointErrorMinimizer<T>::compute_in_place(mPts);
	return result;
}


template struct PointToPointWithPenaltiesErrorMinimizer<float>;
template struct PointToPointWithPenaltiesErrorMinimizer<double>;
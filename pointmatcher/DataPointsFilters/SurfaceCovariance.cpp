// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2018,
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
#include "SurfaceCovariance.h"

// Eigenvalues
#include "Eigen/QR"
#include "Eigen/Eigenvalues"

#include "PointMatcherPrivate.h"
#include "IO.h"
#include "MatchersImpl.h"

#include <boost/format.hpp>

#include "utils.h"

// SurfaceCovarianceDataPointsFilter
// Constructor
template<typename T>
SurfaceCovarianceDataPointsFilter<T>::SurfaceCovarianceDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("SurfaceCovarianceDataPointsFilter",
																		SurfaceCovarianceDataPointsFilter::availableParameters(), params),
	knn(Parametrizable::get<int>("knn")),
	maxDist(Parametrizable::get<T>("maxDist")),
	epsilon(Parametrizable::get<T>("epsilon"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints
SurfaceCovarianceDataPointsFilter<T>::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void SurfaceCovarianceDataPointsFilter<T>::inPlaceFilter(
	DataPoints& cloud)
{
	typedef typename DataPoints::View View;
	typedef typename MatchersImpl<T>::KDTreeMatcher KDTreeMatcher;
	typedef typename PointMatcher<T>::Matches Matches;
	
	using namespace PointMatcherSupport;

	const int pointsCount(cloud.features.cols());
	const int featDim(cloud.features.rows());
	const int descDim(cloud.descriptors.rows());
	const unsigned int labelDim(cloud.descriptorLabels.size());

	// Validate descriptors and labels
	int insertDim(0);
	for(unsigned int i = 0; i < labelDim ; ++i)
		insertDim += cloud.descriptorLabels[i].span;
	if (insertDim != descDim)
		throw InvalidField("SurfaceCovarianceDataPointsFilter: Error, descriptor labels do not match descriptor data");

	// Reserve memory for new descriptors
	const size_t dimCovs((featDim-1) * (featDim-1));


	if (!cloud.descriptorExists("covariances")) {
		// Reserve memory
		cloud.allocateDescriptor("covariances", dimCovs);
		cloud.getDescriptorViewByName("covariances").setZero();
	}

	View covariances = cloud.getDescriptorViewByName("covariances");

	using namespace PointMatcherSupport;
	// Build kd-tree
	Parametrizable::Parameters param;
	boost::assign::insert(param) ( "knn", toParam(knn) );
	boost::assign::insert(param) ( "epsilon", toParam(epsilon) );
	boost::assign::insert(param) ( "maxDist", toParam(maxDist) );
	
	KDTreeMatcher matcher(param);
	matcher.init(cloud);

	Matches matches(typename Matches::Dists(knn, pointsCount), typename Matches::Ids(knn, pointsCount));
	matches = matcher.findClosests(cloud);

	// Search for surrounding points and compute descriptors
	for (int i = 0; i < pointsCount; ++i)
	{
		// Mean of nearest neighbors (NN)
		Matrix d(featDim-1, knn);
		int realKnn = 0;

		for(int j = 0; j < int(knn); ++j)
		{
			if (matches.dists(j,i) != Matches::InvalidDist)
			{
				const int refIndex(matches.ids(j,i));
				d.col(realKnn) = cloud.features.block(0, refIndex, featDim-1, 1);
				++realKnn;
			}
		}
		d.conservativeResize(Eigen::NoChange, realKnn);

		const Vector mean = d.rowwise().sum() / T(realKnn);
		const Matrix NN = d.colwise() - mean;

		Matrix C(NN * NN.transpose());
		const Vector covVec = Eigen::Map<Vector>(C.data(), dimCovs);
		// We add together the previous matrix
		covariances.col(i) += covVec.transpose();

	}

}

template struct SurfaceCovarianceDataPointsFilter<float>;
template struct SurfaceCovarianceDataPointsFilter<double>;


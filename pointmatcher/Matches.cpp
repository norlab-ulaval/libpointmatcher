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

#include "PointMatcher.h"
#include "PointMatcherPrivate.h"

using namespace std;

//! Construct empty matches
template<typename T>
PointMatcher<T>::Matches::Matches() {}

//! Construct matches from distances to and identifiers of closest points
template<typename T>
PointMatcher<T>::Matches::Matches(const Dists& dists, const Ids ids):
	dists(dists),
	ids(ids)
{}

//! Construct uninitialized matches from number of closest points (knn) and number of points (pointsCount)
template<typename T>
PointMatcher<T>::Matches::Matches(const int knn, const int pointsCount):
	dists(Dists(knn, pointsCount)),
	ids(Ids(knn, pointsCount))
{}

//! Get the distance at the T-ratio closest point
template<typename T>
T PointMatcher<T>::Matches::getDistsQuantile(const T quantile) const
{
	// build array
	vector<T> values;
	values.reserve(dists.rows() * dists.cols());
	for (int x = 0; x < dists.cols(); ++x)
		for (int y = 0; y < dists.rows(); ++y)
			if (dists(y, x) != numeric_limits<T>::infinity())
				values.push_back(dists(y, x));
	if (values.size() == 0)
		throw ConvergenceError("no outlier to filter");
	
	// get quantile
	nth_element(values.begin(), values.begin() + (values.size() * quantile), values.end());
	return values[values.size() * quantile];
}

template struct PointMatcher<float>::Matches;
template struct PointMatcher<double>::Matches;

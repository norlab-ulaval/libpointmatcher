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

#include "OutlierFilters.h"

#include <algorithm>
#include <vector>
#include <iostream>
#include <limits>

using namespace std;


// NullFeatureOutlierFilter
template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::NullFeatureOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	return OutlierWeights::Constant(input.ids.rows(), input.ids.cols(), 1);
}

template struct OutlierFiltersImpl<float>::NullFeatureOutlierFilter;
template struct OutlierFiltersImpl<double>::NullFeatureOutlierFilter;


// MaxDistOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::MaxDistOutlierFilter::MaxDistOutlierFilter(const Parameters& params):
	FeatureOutlierFilter("MaxDistOutlierFilter", MaxDistOutlierFilter::availableParameters(), params),
	maxDist(Parametrizable::get<T>("maxDist"))
{
}


template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::MaxDistOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	// select weight from median
	OutlierWeights w(input.dists.rows(), input.dists.cols());
	for (int x = 0; x < w.cols(); ++x)
	{
		for (int y = 0; y < w.rows(); ++y)
		{
			if (input.dists(y, x) > maxDist)
				w(y, x) = 0;
			else
				w(y, x) = 1;
		}
	}
	
	return w;
}

template struct OutlierFiltersImpl<float>::MaxDistOutlierFilter;
template struct OutlierFiltersImpl<double>::MaxDistOutlierFilter;

// MinDistOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::MinDistOutlierFilter::MinDistOutlierFilter(const Parameters& params):
	FeatureOutlierFilter("MinDistOutlierFilter", MinDistOutlierFilter::availableParameters(), params),
	minDist(Parametrizable::get<T>("minDist"))
{
}

template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::MinDistOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	
	OutlierWeights w(input.dists.rows(), input.dists.cols());
	for (int x = 0; x < w.cols(); ++x)
	{
		for (int y = 0; y < w.rows(); ++y)
		{
			if (input.dists(y, x) < minDist)
				w(y, x) = 0;
			else
				w(y, x) = 1;
		}
	}
	
	return w;
}

template struct OutlierFiltersImpl<float>::MinDistOutlierFilter;
template struct OutlierFiltersImpl<double>::MinDistOutlierFilter;



// MedianDistOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::MedianDistOutlierFilter::MedianDistOutlierFilter(const Parameters& params):
	FeatureOutlierFilter("MedianDistOutlierFilter", MedianDistOutlierFilter::availableParameters(), params),
	factor(Parametrizable::get<T>("factor"))
{
}

template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::MedianDistOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	const T median = input.getDistsQuantile(0.5);
	
	// select weight from median
	OutlierWeights w(input.dists.rows(), input.dists.cols());
	for (int x = 0; x < w.cols(); ++x)
	{
		for (int y = 0; y < w.rows(); ++y)
		{
			if (input.dists(y, x) > factor * median)
				w(y, x) = 0;
			else
				w(y, x) = 1;
			//if (w(y, x) == 0)
			//	cout << "rejeting long dist " <<  input.dists(y, x) << " on median " << median << endl;
		}
	}
	
	return w;
}

template struct OutlierFiltersImpl<float>::MedianDistOutlierFilter;
template struct OutlierFiltersImpl<double>::MedianDistOutlierFilter;


// TrimmedDistOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::TrimmedDistOutlierFilter::TrimmedDistOutlierFilter(const Parameters& params):
	FeatureOutlierFilter("TrimmedDistOutlierFilter", TrimmedDistOutlierFilter::availableParameters(), params),
	ratio(Parametrizable::get<T>("ratio"))
{
}

template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::TrimmedDistOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	const T limit = input.getDistsQuantile(ratio);
	
	// select weight from median
	OutlierWeights w(input.dists.rows(), input.dists.cols());
	for (int x = 0; x < w.cols(); ++x)
	{
		for (int y = 0; y < w.rows(); ++y)
		{
			if (input.dists(y, x) > limit)
				w(y, x) = 0;
			else
				w(y, x) = 1;
		}
	}
	
	return w;
}

template struct OutlierFiltersImpl<float>::TrimmedDistOutlierFilter;
template struct OutlierFiltersImpl<double>::TrimmedDistOutlierFilter;


template<typename T>
OutlierFiltersImpl<T>::VarTrimmedDistOutlierFilter::VarTrimmedDistOutlierFilter(const Parameters& params):
	FeatureOutlierFilter("VarTrimmedDistOutlierFilter", VarTrimmedDistOutlierFilter::availableParameters(), params),
	minRatio(Parametrizable::get<T>("minRatio")),
	maxRatio(Parametrizable::get<T>("maxRatio")),
	lambda(Parametrizable::get<T>("lambda"))
{
}

template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::VarTrimmedDistOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	const T tunedRatio = optimizeInlierRatio(input);
	LOG_INFO_STREAM("Optimized ratio: " << tunedRatio);

	const T limit = input.getDistsQuantile(tunedRatio);
	
	// select weight from median
	typename PointMatcher<T>::OutlierWeights w(input.dists.rows(), input.dists.cols());
	for (int x = 0; x < w.cols(); ++x)
	{
		for (int y = 0; y < w.rows(); ++y)
		{
			if (input.dists(y, x) > limit)
				w(y, x) = 0;
			else
				w(y, x) = 1;
		}
	}

	return w;
}

template<typename T>
T OutlierFiltersImpl<T>::VarTrimmedDistOutlierFilter::optimizeInlierRatio(const Matches& matches)
{
	typedef typename PointMatcher<T>::ConvergenceError ConvergenceError;
	typedef typename Eigen::Array<T, Eigen::Dynamic, 1> LineArray;
	
	const int points_nbr = matches.dists.rows() * matches.dists.cols();
	
	// vector containing the squared distances of the matches
	std::vector<T> tmpSortedDist;
	tmpSortedDist.reserve(points_nbr);
	for (int x = 0; x < matches.dists.cols(); ++x)
		for (int y = 0; y < matches.dists.rows(); ++y)
			if ((matches.dists(y, x) != numeric_limits<T>::infinity()) && (matches.dists(y, x) > 0))
				tmpSortedDist.push_back(matches.dists(y, x));
	if (tmpSortedDist.size() == 0)
		throw ConvergenceError("no outlier to filter");
			
	std::sort(tmpSortedDist.begin(), tmpSortedDist.end());

	const int minEl = floor(this->minRatio*points_nbr);
	const int maxEl = floor(this->maxRatio*points_nbr);

	// Return std::vector to an eigen::vector
	Eigen::Map<LineArray> sortedDist(&tmpSortedDist[0], points_nbr);

	const LineArray trunkSortedDist = sortedDist.segment(minEl, maxEl-minEl);
	const T lowerSum = sortedDist.head(minEl).sum();
	const LineArray ids = LineArray::LinSpaced(trunkSortedDist.rows(), minEl+1, maxEl);
	const LineArray ratio = ids / points_nbr;
	const LineArray deno = ratio.pow(this->lambda);
	const LineArray FRMS = deno.inverse().square() * ids.inverse() * (lowerSum + trunkSortedDist);
	int minIndex(0);// = FRMS.minCoeff();
	FRMS.minCoeff(&minIndex);
	const T optRatio = (float)(minIndex + minEl)/ (float)points_nbr;
	
	//cout << "Optimized ratio: " << optRatio << endl;
	
	return optRatio;

}

template struct OutlierFiltersImpl<float>::VarTrimmedDistOutlierFilter;
template struct OutlierFiltersImpl<double>::VarTrimmedDistOutlierFilter;



// NullDescriptorOutlierFilter
template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::NullDescriptorOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	return OutlierWeights::Constant(input.ids.rows(), input.ids.cols(), 1);
}

template struct OutlierFiltersImpl<float>::NullDescriptorOutlierFilter;
template struct OutlierFiltersImpl<double>::NullDescriptorOutlierFilter;


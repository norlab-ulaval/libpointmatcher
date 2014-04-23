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

#include "OutlierFiltersImpl.h"
#include "PointMatcherPrivate.h"
#include "Functions.h"

#include <algorithm>
#include <vector>
#include <iostream>
#include <limits>

using namespace std;
using namespace PointMatcherSupport;

// NullOutlierFilter
template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::NullOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	return OutlierWeights::Constant(input.ids.rows(), input.ids.cols(), 1);
}

template struct OutlierFiltersImpl<float>::NullOutlierFilter;
template struct OutlierFiltersImpl<double>::NullOutlierFilter;


// MaxDistOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::MaxDistOutlierFilter::MaxDistOutlierFilter(const Parameters& params):
	OutlierFilter("MaxDistOutlierFilter", MaxDistOutlierFilter::availableParameters(), params),
	maxDist(Parametrizable::get<T>("maxDist"))
{
}


template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::MaxDistOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	return OutlierWeights((input.dists.array() <= maxDist).template cast<T>());
}

template struct OutlierFiltersImpl<float>::MaxDistOutlierFilter;
template struct OutlierFiltersImpl<double>::MaxDistOutlierFilter;

// MinDistOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::MinDistOutlierFilter::MinDistOutlierFilter(const Parameters& params):
	OutlierFilter("MinDistOutlierFilter", MinDistOutlierFilter::availableParameters(), params),
	minDist(Parametrizable::get<T>("minDist"))
{
}

template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::MinDistOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	return OutlierWeights((input.dists.array() >= minDist).template cast<T>());
}

template struct OutlierFiltersImpl<float>::MinDistOutlierFilter;
template struct OutlierFiltersImpl<double>::MinDistOutlierFilter;



// MedianDistOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::MedianDistOutlierFilter::MedianDistOutlierFilter(const Parameters& params):
	OutlierFilter("MedianDistOutlierFilter", MedianDistOutlierFilter::availableParameters(), params),
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
	const T limit = factor * median;
	return OutlierWeights((input.dists.array() <= limit).template cast<T>());
}

template struct OutlierFiltersImpl<float>::MedianDistOutlierFilter;
template struct OutlierFiltersImpl<double>::MedianDistOutlierFilter;


// TrimmedDistOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::TrimmedDistOutlierFilter::TrimmedDistOutlierFilter(const Parameters& params):
	OutlierFilter("TrimmedDistOutlierFilter", TrimmedDistOutlierFilter::availableParameters(), params),
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
	return OutlierWeights((input.dists.array() <= limit).template cast<T>());
}

template struct OutlierFiltersImpl<float>::TrimmedDistOutlierFilter;
template struct OutlierFiltersImpl<double>::TrimmedDistOutlierFilter;

// VarTrimmedDistOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::VarTrimmedDistOutlierFilter::VarTrimmedDistOutlierFilter(const Parameters& params):
	OutlierFilter("VarTrimmedDistOutlierFilter", VarTrimmedDistOutlierFilter::availableParameters(), params),
	minRatio(Parametrizable::get<T>("minRatio")),
	maxRatio(Parametrizable::get<T>("maxRatio")),
	lambda(Parametrizable::get<T>("lambda"))
{
	if (this->minRatio >= this->maxRatio)
	{
		throw InvalidParameter((boost::format("VarTrimmedDistOutlierFilter: minRatio (%1%) should be smaller than maxRatio (%2%)") % minRatio % maxRatio).str());
	}
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
	return OutlierWeights((input.dists.array() <= limit).template cast<T>());
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

// SurfaceNormalOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::SurfaceNormalOutlierFilter::SurfaceNormalOutlierFilter(const Parameters& params):
	OutlierFilter("SurfaceNormalOutlierFilter", SurfaceNormalOutlierFilter::availableParameters(), params),
	eps(cos(Parametrizable::get<T>("maxAngle"))),
	warningPrinted(false)
{
	//waring: eps is change to cos(maxAngle)!
}

template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::SurfaceNormalOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	const BOOST_AUTO(normalsReading, filteredReading.getDescriptorViewByName("normals"));
	const BOOST_AUTO(normalsReference, filteredReference.getDescriptorViewByName("normals"));
	
	// select weight from median
	OutlierWeights w(input.dists.rows(), input.dists.cols());

	if(normalsReading.cols() != 0 && normalsReference.cols() != 0)
	{
		for (int x = 0; x < w.cols(); ++x) // pts in reading
		{
			const Vector normalRead = normalsReading.col(x).normalized();

			for (int y = 0; y < w.rows(); ++y) // knn 
			{
				const int idRef = input.ids(y, x);

				const Vector normalRef = normalsReference.col(idRef).normalized();

				const T value = anyabs(normalRead.dot(normalRef));

				if(value < eps) // test to keep the points
					w(y, x) = 0;
				else
					w(y, x) = 1;
			}
		}
	}
	else
	{
		if(warningPrinted == false)
		{
			LOG_INFO_STREAM("SurfaceNormalOutlierFilter: surface normals not available. Skipping filtering");
			warningPrinted = true;
		}

		w = Matrix::Ones(input.dists.rows(), input.dists.cols());
	}
	//abort();
	return w;
}

template struct OutlierFiltersImpl<float>::SurfaceNormalOutlierFilter;
template struct OutlierFiltersImpl<double>::SurfaceNormalOutlierFilter;

// GenericDescriptorOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::GenericDescriptorOutlierFilter::GenericDescriptorOutlierFilter(const Parameters& params):
	OutlierFilter("GenericDescriptorOutlierFilter", GenericDescriptorOutlierFilter::availableParameters(), params),
	source(Parametrizable::getParamValueString("source")),
	descName(Parametrizable::getParamValueString("descName")),
	useSoftThreshold(Parametrizable::get<bool>("useSoftThreshold")),
	useLargerThan(Parametrizable::get<bool>("useLargerThan")),
	threshold(Parametrizable::get<T>("threshold"))
{
	if(source != "reference" && source != "reading")
	{
		throw InvalidParameter(
		(boost::format("GenericDescriptorOutlierFilter: Error, the parameter named 'source' can only be set to 'reference' or 'reading' but was set to %1%") % source).str());
	}
}

template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::GenericDescriptorOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	typedef typename DataPoints::ConstView ConstView;

	const int knn = input.dists.rows();
	const int readPtsCount = input.dists.cols();
	
	OutlierWeights w(knn, readPtsCount);

	const DataPoints *cloud;

	if(source == "reference")
		cloud = &filteredReference;
	else
		cloud = &filteredReference;

	ConstView desc(cloud->getDescriptorViewByName(descName));

	if(desc.rows() != 1)
	{
		throw InvalidParameter(
		(boost::format("GenericDescriptorOutlierFilter: Error, the parameter named 'descName' must be a 1D descriptor but the field %1% is %2%D") % descName % desc.rows()).str());
	}

	for(int k=0; k < knn; k++)
	{
		for(int i=0; i < readPtsCount; i++)
		{
			if(useSoftThreshold == false)
			{
				if(useLargerThan == true)
				{
					if(desc(0, input.ids(k,i)) > threshold)
						w(k,i) = 1;
					else
						w(k,i) = 0;
				}
				else
				{
					if(desc(0, input.ids(k,i)) < threshold)
						w(k,i) = 1;
					else
						w(k,i) = 0;
				}
			}
			else
			{
				// use soft threshold by assigning the weight using the descriptor
				w(k,i) = desc(0, input.ids(k,i));
			}
		}
	}

	//Normalize
	if(useSoftThreshold)
		w = w/w.maxCoeff();

	return w;
}

template struct OutlierFiltersImpl<float>::GenericDescriptorOutlierFilter;
template struct OutlierFiltersImpl<double>::GenericDescriptorOutlierFilter;



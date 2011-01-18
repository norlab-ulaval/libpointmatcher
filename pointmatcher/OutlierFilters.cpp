// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab

#include "Core.h"
#include <algorithm>
#include <vector>
#include <iostream>

using namespace std;


// NullFeatureOutlierFilter
template<typename T>
typename MetricSpaceAligner<T>::OutlierWeights MetricSpaceAligner<T>::NullFeatureOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input,
	bool& iterate)
{
	return OutlierWeights::Constant(input.ids.rows(), input.ids.cols(), 1);
}

template struct MetricSpaceAligner<float>::NullFeatureOutlierFilter;
template struct MetricSpaceAligner<double>::NullFeatureOutlierFilter;


// MaxDistOutlierFilter
template<typename T>
MetricSpaceAligner<T>::MaxDistOutlierFilter::MaxDistOutlierFilter(const T maxDist):
	maxDist(maxDist)
{
	if (maxDist <= 0)
	{
		cerr << "MaxDistOutlierFilter: Error, maxDist " << maxDist << " is below or equal to 0." << endl;
		abort();
	}
}


template<typename T>
typename MetricSpaceAligner<T>::OutlierWeights MetricSpaceAligner<T>::MaxDistOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input,
	bool& iterate)
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

template struct MetricSpaceAligner<float>::MaxDistOutlierFilter;
template struct MetricSpaceAligner<double>::MaxDistOutlierFilter;


// MedianDistOutlierFilter
template<typename T>
MetricSpaceAligner<T>::MedianDistOutlierFilter::MedianDistOutlierFilter(const T factor):
	factor(factor)
{
	if (factor <= 0)
	{
		cerr << "MedianDistOutlierFilter: Error, factor " << factor << " is below 0." << endl;
		abort();
	}
}

template<typename T>
typename MetricSpaceAligner<T>::OutlierWeights MetricSpaceAligner<T>::MedianDistOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input,
	bool& iterate)
{
	// TODO: check alignment and use matrix underlying storage when available
	// build array
	vector<T> values;
	values.reserve(input.dists.rows() * input.dists.cols());
	for (int x = 0; x < input.dists.cols(); ++x)
		for (int y = 0; y < input.dists.rows(); ++y)
			if (input.dists(y, x) > 0)
				values.push_back(input.dists(y, x));
	
	// get median
	nth_element(values.begin(), values.begin() + (values.size() /2), values.end());
	const T median = values[values.size()/2];
	
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

template struct MetricSpaceAligner<float>::MedianDistOutlierFilter;
template struct MetricSpaceAligner<double>::MedianDistOutlierFilter;


// TrimmedDistOutlierFilter
template<typename T>
MetricSpaceAligner<T>::TrimmedDistOutlierFilter::TrimmedDistOutlierFilter(const T ratio):
	ratio(ratio)
{
	if (ratio >= 1 || ratio <= 0)
	{
		cerr << "TrimmedDistOutlierFilter: Error, trim ratio " << ratio << " is outside interval ]0;1[." << endl;
		abort();
	}
}

template<typename T>
typename MetricSpaceAligner<T>::OutlierWeights MetricSpaceAligner<T>::TrimmedDistOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input,
	bool& iterate)
{
	// TODO: check alignment and use matrix underlying storage when available
	// build array
	vector<T> values;
	values.reserve(input.dists.rows() * input.dists.cols());
	for (int x = 0; x < input.dists.cols(); ++x)
		for (int y = 0; y < input.dists.rows(); ++y)
			if (input.dists(y, x) > 0)
				values.push_back(input.dists(y, x));

	// get quartiles value
	nth_element(values.begin(), values.begin() + (values.size() * ratio), values.end());
	const T limit = values[values.size() * ratio];
	
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
			//if (w(y, x) == 0)
			//	cout << "rejeting long dist " <<  input.dists(y, x) << " on median " << median << endl;
		}
	}
	
	return w;
}

template struct MetricSpaceAligner<float>::TrimmedDistOutlierFilter;
template struct MetricSpaceAligner<double>::TrimmedDistOutlierFilter;

// MinDistOutlierFilter
template<typename T>
typename MetricSpaceAligner<T>::OutlierWeights MetricSpaceAligner<T>::MinDistOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input,
	bool& iterate)
{
	
	OutlierWeights w(input.dists.rows(), input.dists.cols());
	for (int x = 0; x < w.cols(); ++x)
	{
		for (int y = 0; y < w.rows(); ++y)
		{
			if (input.dists(y, x) < minDist)
				w(y, x) = input.dists(y, x)/minDist;
			else
				w(y, x) = 1;
		}
	}
	
	return w;
}

template struct MetricSpaceAligner<float>::MinDistOutlierFilter;
template struct MetricSpaceAligner<double>::MinDistOutlierFilter;



// NullDescriptorOutlierFilter
template<typename T>
typename MetricSpaceAligner<T>::OutlierWeights MetricSpaceAligner<T>::NullDescriptorOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input,
	bool& iterate)
{
	return OutlierWeights::Constant(input.ids.rows(), input.ids.cols(), 1);
}

template struct MetricSpaceAligner<float>::NullDescriptorOutlierFilter;
template struct MetricSpaceAligner<double>::NullDescriptorOutlierFilter;


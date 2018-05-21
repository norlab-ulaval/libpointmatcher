#include "MaxQuantileOnAxis.h"

#include <algorithm>
#include <vector>

// MaxQuantileOnAxisDataPointsFilter
// Constructor
template<typename T>
MaxQuantileOnAxisDataPointsFilter<T>::MaxQuantileOnAxisDataPointsFilter(
	const Parameters& params): 
	PointMatcher<T>::DataPointsFilter("MaxQuantileOnAxisDataPointsFilter",
		MaxQuantileOnAxisDataPointsFilter::availableParameters(), params),
	dim(Parametrizable::get<unsigned>("dim")),
	ratio(Parametrizable::get<T>("ratio"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints 
MaxQuantileOnAxisDataPointsFilter<T>::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void MaxQuantileOnAxisDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	if (int(dim) >= cloud.features.rows())
		throw InvalidParameter((boost::format("MaxQuantileOnAxisDataPointsFilter: Error, filtering on dimension number %1%, larger than feature dimensionality %2%") % dim % cloud.features.rows()).str());

	const int nbPointsIn = cloud.features.cols();
	const int nbPointsOut = nbPointsIn * ratio;

	// build array
	std::vector<T> values;
	values.reserve(nbPointsIn);
	for (int x = 0; x < nbPointsIn; ++x)
		values.push_back(cloud.features(dim, x));

	// get quartiles value
	std::nth_element(values.begin(), values.begin() + (values.size() * ratio), values.end());
	const T limit = values[nbPointsOut];

	// copy towards beginning the elements we keep
	int j = 0;
	for (int i = 0; i < nbPointsIn; ++i)
	{
		if (cloud.features(dim, i) < limit)
		{
			assert(j <= i);
			cloud.setColFrom(j, cloud, i);
			++j;
		}
	}
	assert(j <= nbPointsOut);

	cloud.conservativeResize(j);

}

template struct MaxQuantileOnAxisDataPointsFilter<float>;
template struct MaxQuantileOnAxisDataPointsFilter<double>;



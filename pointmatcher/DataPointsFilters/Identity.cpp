#include "Identity.h"

// IdentityDataPointsFilter
template<typename T>
typename PointMatcher<T>::DataPoints IdentityDataPointsFilter<T>::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void IdentityDataPointsFilter<T>::inPlaceFilter(
	DataPoints& cloud)
{
}

template struct IdentityDataPointsFilter<float>;
template struct IdentityDataPointsFilter<double>;

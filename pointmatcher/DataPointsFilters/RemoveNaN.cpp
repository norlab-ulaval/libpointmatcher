#include "RemoveNaN.h"

// RemoveNaNDataPointsFilter
template<typename T>
typename PointMatcher<T>::DataPoints RemoveNaNDataPointsFilter<T>::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void RemoveNaNDataPointsFilter<T>::inPlaceFilter(
	DataPoints& cloud)
{
	const int nbPointsIn = cloud.features.cols();

	int j = 0;
	for (int i = 0; i < nbPointsIn; ++i)
	{
		const BOOST_AUTO(colArray, cloud.features.col(i).array());
		const BOOST_AUTO(hasNaN, !(colArray == colArray).all());
		if (!hasNaN)
		{
			cloud.setColFrom(j, cloud, i);
			++j;
		}
	}

	cloud.conservativeResize(j);
}

template struct RemoveNaNDataPointsFilter<float>;
template struct RemoveNaNDataPointsFilter<double>;


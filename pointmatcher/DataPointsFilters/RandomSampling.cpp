#include "RandomSampling.h"

// RandomSamplingDataPointsFilter
// Constructor
template<typename T>
RandomSamplingDataPointsFilter<T>::RandomSamplingDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("RandomSamplingDataPointsFilter", RandomSamplingDataPointsFilter::availableParameters(), params),
	prob(Parametrizable::get<double>("prob"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints
RandomSamplingDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void RandomSamplingDataPointsFilter<T>::inPlaceFilter(
	DataPoints& cloud)
{
	const int nbPointsIn = cloud.features.cols();

	int j = 0;
	for (int i = 0; i < nbPointsIn; ++i)
	{
		const float r = (float)std::rand()/(float)RAND_MAX;
		if (r < prob)
		{
			cloud.setColFrom(j, cloud, i);
			++j;
		}
	}

	cloud.conservativeResize(j);
}

template struct RandomSamplingDataPointsFilter<float>;
template struct RandomSamplingDataPointsFilter<double>;



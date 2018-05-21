#include "MinDist.h"

#include "Functions.h"

// MinDistDataPointsFilter
// Constructor
template<typename T>
MinDistDataPointsFilter<T>::MinDistDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("MinDistDataPointsFilter",
		MinDistDataPointsFilter::availableParameters(), params),
	dim(Parametrizable::get<unsigned>("dim")),
	minDist(Parametrizable::get<T>("minDist"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints MinDistDataPointsFilter<T>::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void MinDistDataPointsFilter<T>::inPlaceFilter(
	DataPoints& cloud)
{
	using namespace PointMatcherSupport;
	
	if (dim >= cloud.features.rows() - 1)
		throw InvalidParameter((boost::format("MinDistDataPointsFilter: Error, filtering on dimension number %1%, larger than feature dimensionality %2%") % dim % (cloud.features.rows() - 2)).str());

	const int nbPointsIn = cloud.features.cols();
	const int nbRows = cloud.features.rows();

	int j = 0;
	if(dim == -1) // Euclidean distance
	{
		const T absMinDist = anyabs(minDist);
		for (int i = 0; i < nbPointsIn; ++i)
		{
			if (cloud.features.col(i).head(nbRows-1).norm() > absMinDist)
			{
				cloud.setColFrom(j, cloud, i);
				++j;
			}
		}
	}
	else // Single axis distance
	{
		for (int i = 0; i < nbPointsIn; ++i)
		{
			if ((cloud.features(dim, i)) > minDist)
			{
				cloud.setColFrom(j, cloud, i);
				++j;
			}
		}
	}

	cloud.conservativeResize(j);

}

template struct MinDistDataPointsFilter<float>;
template struct MinDistDataPointsFilter<double>;

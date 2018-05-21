#include "MaxDist.h"

#include "pointmatcher/Functions.h"

// MaxDistDataPointsFilter
// Constructor
template<typename T>
MaxDistDataPointsFilter<T>::MaxDistDataPointsFilter(const Parameters& params) :
	PointMatcher<T>::DataPointsFilter("MaxDistDataPointsFilter",
		MaxDistDataPointsFilter::availableParameters(), params),
	dim(Parametrizable::get<unsigned>("dim")),
	maxDist(Parametrizable::get<T>("maxDist"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints MaxDistDataPointsFilter<T>::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void MaxDistDataPointsFilter<T>::inPlaceFilter(
	DataPoints& cloud)
{
	using namespace PointMatcherSupport;
	
	if (dim >= cloud.features.rows() - 1)
	{
		throw InvalidParameter(
			(boost::format("MaxDistDataPointsFilter: Error, filtering on dimension number %1%, larger than authorized axis id %2%") % dim % (cloud.features.rows() - 2)).str());
	}

	const int nbPointsIn = cloud.features.cols();
	const int nbRows = cloud.features.rows();

	int j = 0;
	if(dim == -1) // Euclidean distance
	{
		const T absMaxDist = anyabs(maxDist);
		for (int i = 0; i < nbPointsIn; ++i)
		{
			if (cloud.features.col(i).head(nbRows-1).norm() < absMaxDist)
			{
				cloud.setColFrom(j, cloud, i);
				++j;
			}
		}
	}
	else // Single-axis distance
	{
		for (int i = 0; i < nbPointsIn; ++i)
		{
			if ((cloud.features(dim, i)) < maxDist)
			{
				cloud.setColFrom(j, cloud, i);
				++j;
			}
		}
	}

	cloud.conservativeResize(j);
}

template struct MaxDistDataPointsFilter<float>;
template struct MaxDistDataPointsFilter<double>;

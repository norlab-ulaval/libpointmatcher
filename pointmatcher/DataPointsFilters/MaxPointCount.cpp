#include "MaxPointCount.h"

// MaxPointCountDataPointsFilter
// Constructor
template<typename T>
MaxPointCountDataPointsFilter<T>::MaxPointCountDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("MaxPointCountDataPointsFilter", 
		MaxPointCountDataPointsFilter::availableParameters(), params),
	maxCount(Parametrizable::get<unsigned>("maxCount"))
{
	try 
	{
		seed = Parametrizable::get<unsigned>("seed");
	} 
	catch (const InvalidParameter& e) 
	{
		seed = static_cast<unsigned int> (1); // rand default seed number
	}
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints 
MaxPointCountDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void MaxPointCountDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	unsigned N = static_cast<unsigned> (cloud.features.cols());
	if (maxCount < N) 
	{
		DataPoints cloud_filtered = cloud.createSimilarEmpty(maxCount);
		std::srand(seed);

		unsigned top = N - maxCount;
		unsigned i = 0;
		unsigned index = 0;
		for (size_t n = maxCount; n >= 2; --n)
		{
			const float V = static_cast<float>(std::rand () / double (RAND_MAX));
			unsigned S = 0;
			float quot = static_cast<float> (top) / static_cast<float> (N);
			while (quot > V)
			{
				++S;
				--top;
				--N;
				quot = quot * static_cast<float> (top) / static_cast<float> (N);
			}
			index += S;
			cloud_filtered.setColFrom(i++, cloud, index++);
			--N;
		}
		//FIXME
		index += N * static_cast<unsigned> (static_cast<float>(std::rand() / double (RAND_MAX)));
		cloud_filtered.setColFrom(i++, cloud, index++);
		
		PointMatcher<T>::swapDataPoints(cloud, cloud_filtered);
		cloud.conservativeResize(i);
	}
}

template struct MaxPointCountDataPointsFilter<float>;
template struct MaxPointCountDataPointsFilter<double>;


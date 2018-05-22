#include "CutAtDescriptorThreshold.h"

// CutAtDescriptorThresholdDataPointsFilter
// Constructor
template<typename T>
CutAtDescriptorThresholdDataPointsFilter<T>::CutAtDescriptorThresholdDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("CutAtDescriptorThresholdDataPointsFilter", 
		CutAtDescriptorThresholdDataPointsFilter::availableParameters(), params),
	descName(Parametrizable::get<std::string>("descName")),
	useLargerThan(Parametrizable::get<bool>("useLargerThan")),
	threshold(Parametrizable::get<T>("threshold"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints
CutAtDescriptorThresholdDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void CutAtDescriptorThresholdDataPointsFilter<T>::inPlaceFilter(
	DataPoints& cloud)
{
	// Check field exists
	if (!cloud.descriptorExists(descName))
	{
		throw InvalidField("CutAtDescriptorThresholdDataPointsFilter: Error, field not found in descriptors.");
	}

	const int nbPointsIn = cloud.features.cols();
	typename DataPoints::View values = cloud.getDescriptorViewByName(descName);

	// fill cloud values
	int j = 0;
	if (useLargerThan)
	{
		for (int i = 0; i < nbPointsIn; ++i)
		{
			const T value(values(0,i));
			if (value <= threshold)
			{
				cloud.setColFrom(j, cloud, i);
				++j;
			}
		}
	}
	else
	{
		for (int i = 0; i < nbPointsIn; ++i)
		{
			const T value(values(0,i));
			if (value >= threshold)
			{
				cloud.setColFrom(j, cloud, i);
				++j;
			}
		}
	}
	cloud.conservativeResize(j);
}

template struct CutAtDescriptorThresholdDataPointsFilter<float>;
template struct CutAtDescriptorThresholdDataPointsFilter<double>;

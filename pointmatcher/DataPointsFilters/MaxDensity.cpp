#include "MaxDensity.h"

// MaxDensityDataPointsFilter
// Constructor
template<typename T>
MaxDensityDataPointsFilter<T>::MaxDensityDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("MaxDensityDataPointsFilter", 
		MaxDensityDataPointsFilter::availableParameters(), params),
	maxDensity(Parametrizable::get<T>("maxDensity"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints MaxDensityDataPointsFilter<T>::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void MaxDensityDataPointsFilter<T>::inPlaceFilter(
	DataPoints& cloud)
{
	typedef typename DataPoints::View View;

	// Force densities to be computed
	if (!cloud.descriptorExists("densities"))
	{
		throw InvalidField("MaxDensityDataPointsFilter: Error, no densities found in descriptors.");
	}

	const int nbPointsIn = cloud.features.cols();
	View densities = cloud.getDescriptorViewByName("densities");
	const T lastDensity = densities.maxCoeff();
	const int nbSaturatedPts = (densities.array() == lastDensity).count();

	// fill cloud values
	int j = 0;
	for (int i = 0; i < nbPointsIn; ++i)
	{
		const T density(densities(0,i));
		if (density > maxDensity)
		{
			const float r = (float)std::rand()/(float)RAND_MAX;
			float acceptRatio = maxDensity/density;

			// Handle saturation value of density
			if (density == lastDensity)
			{
				acceptRatio = acceptRatio * (1-nbSaturatedPts/nbPointsIn);
			}

			if (r < acceptRatio)
			{
				cloud.setColFrom(j, cloud, i);
				++j;
			}
		}
		else
		{
			cloud.setColFrom(j, cloud, i);
			++j;
		}
	}

	cloud.conservativeResize(j);
}

template struct MaxDensityDataPointsFilter<float>;
template struct MaxDensityDataPointsFilter<double>;


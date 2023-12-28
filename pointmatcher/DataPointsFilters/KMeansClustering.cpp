// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab

#include "KMeansClustering.h"

template <typename T>
KMeansClusteringDataPointsFilter<T>::KMeansClusteringDataPointsFilter(const Parameters& params) :
	PointMatcher<T>::DataPointsFilter("KMeansClusteringDataPointsFilter",
		KMeansClusteringDataPointsFilter::availableParameters(), params),
	k(Parametrizable::get<std::size_t>("k")),
	iter(Parametrizable::get<std::size_t>("iter")),
	epsilon(Parametrizable::get<T>("epsilon"))
{
}

// KMeansClusteringDataPointsFilter
template<typename T>
typename PointMatcher<T>::DataPoints KMeansClusteringDataPointsFilter<T>::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void KMeansClusteringDataPointsFilter<T>::inPlaceFilter(
        DataPoints& cloud)
{

}

template struct KMeansClusteringDataPointsFilter<float>;
template struct KMeansClusteringDataPointsFilter<double>;

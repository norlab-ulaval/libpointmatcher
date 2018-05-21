#include "BoundingBox.h"

// BoundingBoxDataPointsFilter
// Constructor
template<typename T>
BoundingBoxDataPointsFilter<T>::BoundingBoxDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("BoundingBoxDataPointsFilter",
		BoundingBoxDataPointsFilter::availableParameters(), params),
	xMin(Parametrizable::get<T>("xMin")),
	xMax(Parametrizable::get<T>("xMax")),
	yMin(Parametrizable::get<T>("yMin")),
	yMax(Parametrizable::get<T>("yMax")),
	zMin(Parametrizable::get<T>("zMin")),
	zMax(Parametrizable::get<T>("zMax")),
	removeInside(Parametrizable::get<bool>("removeInside"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints BoundingBoxDataPointsFilter<T>::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void BoundingBoxDataPointsFilter<T>::inPlaceFilter(
	DataPoints& cloud)
{
	const int nbPointsIn = cloud.features.cols();
	const int nbRows = cloud.features.rows();

	int j = 0;
	for (int i = 0; i < nbPointsIn; ++i)
	{
		bool keepPt = false;
		const Vector point = cloud.features.col(i);

		// FIXME: improve performance by using Eigen array operations
		const bool x_in = (point(0) > xMin && point(0) < xMax);
		const bool y_in = (point(1) > yMin && point(1) < yMax);
		const bool z_in = (point(2) > zMin && point(2) < zMax) || nbRows == 3;
		const bool in_box = x_in && y_in && z_in;

		if(removeInside)
			keepPt = !in_box;
		else
			keepPt = in_box;

		if(keepPt)
		{
			cloud.setColFrom(j, cloud, i);
			++j;
		}
	}

	cloud.conservativeResize(j);
}

template struct BoundingBoxDataPointsFilter<float>;
template struct BoundingBoxDataPointsFilter<double>;



#include "ObservationDirection.h"

// ObservationDirectionDataPointsFilter
// Constructor
template<typename T>
ObservationDirectionDataPointsFilter<T>::ObservationDirectionDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("ObservationDirectionDataPointsFilter", 
		ObservationDirectionDataPointsFilter::availableParameters(), params),
	centerX(Parametrizable::get<T>("x")),
	centerY(Parametrizable::get<T>("y")),
	centerZ(Parametrizable::get<T>("z"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints
ObservationDirectionDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void ObservationDirectionDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	const int dim(cloud.features.rows() - 1);
	const int featDim(cloud.features.cols());
	if (dim != 2 && dim != 3)
	{
		throw InvalidField(
			(boost::format("ObservationDirectionDataPointsFilter: Error, works only in 2 or 3 dimensions, cloud has %1% dimensions.") % dim).str()
		);
	}

	Vector center(dim);
	center[0] = centerX;
	center[1] = centerY;
	if (dim == 3)
		center[2] = centerZ;

	cloud.allocateDescriptor("observationDirections", dim);
	BOOST_AUTO(observationDirections, cloud.getDescriptorViewByName("observationDirections"));

	for (int i = 0; i < featDim; ++i)
	{
		// Check normal orientation
		const Vector p(cloud.features.block(0, i, dim, 1));
		observationDirections.col(i) = center - p;
	}

}

template struct ObservationDirectionDataPointsFilter<float>;
template struct ObservationDirectionDataPointsFilter<double>;


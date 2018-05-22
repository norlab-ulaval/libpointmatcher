#include "OrientNormals.h"

// OrientNormalsDataPointsFilter
// Constructor
template<typename T>
OrientNormalsDataPointsFilter<T>::OrientNormalsDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("OrientNormalsDataPointsFilter", 
		OrientNormalsDataPointsFilter::availableParameters(), params),
	towardCenter(Parametrizable::get<bool>("towardCenter"))
{
}

// OrientNormalsDataPointsFilter
// Compute
template<typename T>
typename PointMatcher<T>::DataPoints 
OrientNormalsDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void OrientNormalsDataPointsFilter<T>::inPlaceFilter(
	DataPoints& cloud)
{
	if (!cloud.descriptorExists("normals"))
		throw InvalidField("OrientNormalsDataPointsFilter: Error, cannot find normals in descriptors.");
	if (!cloud.descriptorExists("observationDirections"))
		throw InvalidField("OrientNormalsDataPointsFilter: Error, cannot find observation directions in descriptors.");

	BOOST_AUTO(normals, cloud.getDescriptorViewByName("normals"));
	const BOOST_AUTO(observationDirections, cloud.getDescriptorViewByName("observationDirections"));
	assert(normals.rows() == observationDirections.rows());
	const int featDim(cloud.features.cols());
	for (int i = 0; i < featDim; ++i)
	{
		// Check normal orientation
		const Vector vecP = observationDirections.col(i);
		const Vector vecN = normals.col(i);
		const double scalar = vecP.dot(vecN);

		// Swap normal
		if(towardCenter)
		{
			if (scalar < 0)
				normals.col(i) = -vecN;
		}
		else
		{
			if (scalar > 0)
				normals.col(i) = -vecN;
		}
	}

}

template struct OrientNormalsDataPointsFilter<float>;
template struct OrientNormalsDataPointsFilter<double>;


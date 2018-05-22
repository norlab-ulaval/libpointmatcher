#include "IncidenceAngle.h"

// IncidenceAngleDataPointsFilter
// Compute
template<typename T>
typename PointMatcher<T>::DataPoints 
IncidenceAngleDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void IncidenceAngleDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{

	if (!cloud.descriptorExists("normals"))
		throw InvalidField("IncidenceAngleDataPointsFilter: Error, cannot find normals in descriptors.");
	if (!cloud.descriptorExists("observationDirections"))
		throw InvalidField("IncidenceAngleDataPointsFilter: Error, cannot find observation directions in descriptors.");

	cloud.allocateDescriptor("incidenceAngles", 1);
	BOOST_AUTO(angles, cloud.getDescriptorViewByName("incidenceAngles"));

	const BOOST_AUTO(normals, cloud.getDescriptorViewByName("normals"));
	const BOOST_AUTO(observationDirections, cloud.getDescriptorViewByName("observationDirections"));
	assert(normals.rows() == observationDirections.rows());
	
	const unsigned int nbPts(cloud.getNbPoints());

	for (unsigned int i = 0; i < nbPts; ++i)
	{
		// Check normal orientation
		const Vector vecP = observationDirections.col(i).normalized();
		const Vector vecN = normals.col(i);// assumed to be normalized
		angles(0,i) = acos(vecP.dot(vecN));
	}
}

template struct IncidenceAngleDataPointsFilter<float>;
template struct IncidenceAngleDataPointsFilter<double>;

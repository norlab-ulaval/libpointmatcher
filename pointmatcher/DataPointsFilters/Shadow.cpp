#include "Shadow.h"

#include "Functions.h"

// ShadowDataPointsFilter
// Constructor
template<typename T>
ShadowDataPointsFilter<T>::ShadowDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("ShadowDataPointsFilter", 
		ShadowDataPointsFilter::availableParameters(), params),
	eps(sin(Parametrizable::get<T>("eps")))
{
	//waring: maxAngle is change to sin(maxAngle)!
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints
ShadowDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void ShadowDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	using namespace PointMatcherSupport;
	
	// Check if normals are present
	if (!cloud.descriptorExists("normals"))
	{
		throw InvalidField("ShadowDataPointsFilter, Error: cannot find normals in descriptors");
	}

	const int dim(cloud.features.rows());
	const int featDim(cloud.features.cols());

	const BOOST_AUTO(normals, cloud.getDescriptorViewByName("normals"));
	int j = 0;

	for(int i=0; i < featDim; ++i)
	{
		const Vector normal = normals.col(i).normalized();
		const Vector point = cloud.features.block(0, i, dim-1, 1).normalized();

		const T value = anyabs(normal.dot(point));

		if(value > eps) // test to keep the points
		{
			cloud.features.col(j) = cloud.features.col(i);
			cloud.descriptors.col(j) = cloud.descriptors.col(i);
			++j;
		}
	}

	cloud.conservativeResize(j);
}

template struct ShadowDataPointsFilter<float>;
template struct ShadowDataPointsFilter<double>;


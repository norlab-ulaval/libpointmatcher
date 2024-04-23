// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab

#include "AddDescriptor.h"

template <typename T>
AddDescriptorDataPointsFilter<T>::AddDescriptorDataPointsFilter(const Parameters& params) :
	PointMatcher<T>::DataPointsFilter("AddDescriptorDataPointsFilter",
		AddDescriptorDataPointsFilter::availableParameters(), params),
	descriptorName(Parametrizable::get<std::string>("descriptorName")),
	descriptorDimension(Parametrizable::get<std::size_t>("descriptorDimension")),
	descriptorValues(Parametrizable::getVector<T>("descriptorValues"))
{
    assert(descriptorDimension == descriptorValues.size());
}

// AddDescriptorDataPointsFilter
template<typename T>
typename PointMatcher<T>::DataPoints AddDescriptorDataPointsFilter<T>::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void AddDescriptorDataPointsFilter<T>::inPlaceFilter(
        DataPoints& cloud)
{
    Matrix matrix = PM::Matrix::Ones(descriptorDimension, cloud.getNbPoints());
    for(std::size_t i = 0; i < descriptorDimension; ++i)
    {
        matrix.row(i) *= descriptorValues[i];
    }
    if(!cloud.descriptorExists(descriptorName))
    {
        cloud.addDescriptor(descriptorName, matrix);
    }
    else
    {
        if(descriptorDimension == cloud.getDescriptorDimension(descriptorName))
        {
            cloud.getDescriptorViewByName(descriptorName) = matrix;
        }
        else
        {
            // FIXME deal with overwriting descriptors that have different dimensions
            throw std::runtime_error("Can't overwrite existing descriptor " + descriptorName + " with dimension " +
                                     std::to_string(cloud.getDescriptorDimension(descriptorName)) +
                                     " by a new descriptor with dimension " +
                                     std::to_string(descriptorDimension) + ".");
        }
    }

}

template struct AddDescriptorDataPointsFilter<float>;
template struct AddDescriptorDataPointsFilter<double>;

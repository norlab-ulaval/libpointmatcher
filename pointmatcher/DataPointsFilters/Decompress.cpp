//
// Created by Matěj Boxan on 2023-12-04.
//

#include "Decompress.h"

template<typename T>
DecompressDataPointsFilter<T>::DecompressDataPointsFilter(const DecompressDataPointsFilter::Parameters& params):
        PointMatcher<T>::DataPointsFilter("DecompressDataPointsFilter",
                                          DecompressDataPointsFilter::availableParameters(), params),
                                          seed(Parametrizable::get<int>("seed"))
{
	try
	{
		const auto pg = this->template get<unsigned>("pointsGenerator");
		pointsGeneratorType = PointsGeneratorType(pg);
	}
	catch (const InvalidParameter&)
	{
		pointsGeneratorType = PointsGeneratorType::GAUSSIAN;
	}
}

template<typename T>
typename PointMatcher<T>::DataPoints DecompressDataPointsFilter<T>::filter(const DataPoints& input)
{
    DataPoints output(input);
    inPlaceFilter(output);
    return output;
}

template<typename T>
void DecompressDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
    switch(pointsGeneratorType)
    {
        case PointsGeneratorType::GAUSSIAN:
        {
            GaussianGenerator generator(seed);
            generator.processCloud(cloud);
            break;
        }
        case PointsGeneratorType::UNIFORM:
        {
            UniformGenerator generator(seed);
            generator.processCloud(cloud);
            break;
        }
    }
}

template struct DecompressDataPointsFilter<float>;
template struct DecompressDataPointsFilter<double>;
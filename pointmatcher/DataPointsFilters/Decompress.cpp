//
// Created by Matěj Boxan on 2023-12-04.
//

#include "Decompress.h"

template<typename T>
DecompressDataPointsFilter<T>::DecompressDataPointsFilter(const DecompressDataPointsFilter::Parameters& params):
        PointMatcher<T>::DataPointsFilter("DecompressDataPointsFilter",
                                          DecompressDataPointsFilter::availableParameters(), params)
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
            GaussianGenerator generator;
            generator.processCloud(cloud);
        }
        case PointsGeneratorType::UNIFORM:
        {
            UniformGenerator generator;
            generator.processCloud(cloud);
        }
    }
}

template struct DecompressDataPointsFilter<float>;
template struct DecompressDataPointsFilter<double>;
// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab

#pragma once

#include "PointMatcher.h"

//! Add new descriptor to an existing point cloud
template<typename T>
struct AddDescriptorDataPointsFilter : public PointMatcher<T>::DataPointsFilter
{
    // Type definitions
    typedef PointMatcher<T> PM;
    typedef typename PM::DataPoints DataPoints;
    typedef typename PM::DataPointsFilter DataPointsFilter;

    typedef PointMatcherSupport::Parametrizable Parametrizable;
    typedef PointMatcherSupport::Parametrizable P;
    typedef Parametrizable::Parameters Parameters;
    typedef Parametrizable::ParameterDoc ParameterDoc;
    typedef Parametrizable::ParametersDoc ParametersDoc;
    typedef Parametrizable::InvalidParameter InvalidParameter;

    typedef typename PointMatcher<T>::Matrix Matrix;
    typedef typename PointMatcher<T>::Vector Vector;
    typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;

    const std::string descriptorName;
    const std::size_t descriptorDimension;
    const std::vector<T> descriptorValues;

    inline static const std::string description()
    {
        return "Adds a new descriptor to an existing point cloud or overwrites existing descriptor with the same name.\n\n"
			   "Required descriptors: none.\n"
		       "Produced descriptors:  User defined.\n"
			   "Altered descriptors:  none.\n"
			   "Altered features:     none.";
    }

    inline static const ParametersDoc availableParameters()
    {
        return {
                {"descriptorName",      "Name of the descriptor to be added.", "testDescriptor" },
                {"descriptorDimension", "Length of the descriptor to be added.", "1",            "1", "4294967295", &P::Comp < std::size_t > },
                {"descriptorValues",    "Values of the descriptor to be added.\n"
                                        "List of 'descriptorDimension' numbers of type T, separated by commas, closed in brackets,\n"
                                        "e.g. [2.2, 3.0, 6.1]", ""}
        };
    }

	//Constructor, uses parameter interface
	explicit AddDescriptorDataPointsFilter(const Parameters& params = Parameters());

    virtual DataPoints filter(const DataPoints& input);
    virtual void inPlaceFilter(DataPoints& cloud);
};

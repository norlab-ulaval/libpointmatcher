#pragma once

#include "PointMatcher.h"

//! Remove points having NaN as coordinate
template<typename T>
struct RemoveNaNDataPointsFilter: public PointMatcher<T>::DataPointsFilter
{
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	
	inline static const std::string description()
	{
		return "Remove points having NaN as coordinate.";
	}
	
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};

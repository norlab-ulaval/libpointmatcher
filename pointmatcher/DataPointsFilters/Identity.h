#pragma once

#include "PointMatcher.h"

//! IdentityDataPointsFilter, does nothing
template< typename T>
struct IdentityDataPointsFilter: public PointMatcher<T>::DataPointsFilter
{
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	
	inline static const std::string description()
	{
		return "Does nothing.";
	}
	
	//! Constructor, uses parameter interface
	//IdentityDataPointsFilter(const Parameters& params = Parameters());
	
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};

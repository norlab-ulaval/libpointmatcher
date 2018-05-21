#pragma once

#include "PointMatcher.h"

//! Subsampling. Filter points before a minimum distance measured on a specific axis
template< typename T>
struct MinDistDataPointsFilter: public PointMatcher<T>::DataPointsFilter
{
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	typedef Parametrizable::InvalidParameter InvalidParameter;
	
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	
	inline static const std::string description()
	{
		return "Subsampling. Filter points before a minimum distance measured on a specific axis. If dim is set to -1, points are filtered based on a minimum radius.";
	}
	inline static const ParametersDoc availableParameters()
	{
		return boost::assign::list_of<ParameterDoc>
			( "dim", "dimension on which the filter will be applied. x=0, y=1, z=2, radius=-1", "-1", "-1", "2", &P::Comp<int> )
			( "minDist", "minimum value authorized. If dim is set to -1 (radius), the absolute value of minDist will be used. All points before that will be filtered.", "1", "-inf", "inf", &P::Comp<T> )
		;
	}
	
	const int dim;
	const T minDist;

	//! Constructor, uses parameter interface
	MinDistDataPointsFilter(const Parameters& params = Parameters());
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};

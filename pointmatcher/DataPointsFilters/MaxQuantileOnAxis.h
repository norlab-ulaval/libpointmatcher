#pragma once

#include "PointMatcher.h"

//! Subsampling. Filter points beyond a maximum quantile measured on a specific axis
template< typename T>
struct MaxQuantileOnAxisDataPointsFilter: public PointMatcher<T>::DataPointsFilter
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
		return "Subsampling. Filter points beyond a maximum quantile measured on a specific axis.";
	}
	inline static const ParametersDoc availableParameters()
	{
		return boost::assign::list_of<ParameterDoc>
			( "dim", "dimension on which the filter will be applied. x=0, y=1, z=2", "0", "0", "2", &P::Comp<unsigned> )
			( "ratio", "maximum quantile authorized. All points beyond that will be filtered.", "0.5", "0.0000001", "0.9999999", &P::Comp<T> )
		;
	}
	
	const unsigned dim;
	const T ratio;
	
	//! Constructor, uses parameter interface
	MaxQuantileOnAxisDataPointsFilter(const Parameters& params = Parameters());
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};

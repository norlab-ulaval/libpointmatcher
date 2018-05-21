#pragma once

#include "PointMatcher.h"

//! Subsampling. Reduce the points number by randomly removing points with a dentsity higher than a treshold.
template< typename T>
struct MaxDensityDataPointsFilter: public PointMatcher<T>::DataPointsFilter
{
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	typedef Parametrizable::InvalidParameter InvalidParameter;
	
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;
	
	inline static const std::string description()
	{
		return "Subsampling. Reduce the points number by randomly removing points with a density highler than a treshold.";
	}
	inline static const ParametersDoc availableParameters()
	{
		return boost::assign::list_of<ParameterDoc>
			( "maxDensity", "Maximum density of points to target. Unit: number of points per m^3.", "10", "0.0000001", "inf", &P::Comp<T> )
		;
	}
	
	const T maxDensity;
	
	//! Constructor, uses parameter interface
	MaxDensityDataPointsFilter(const Parameters& params = Parameters());
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};


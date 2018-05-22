#pragma once

#include "PointMatcher.h"

//! Maximum number of points
template<typename T>
struct MaxPointCountDataPointsFilter: public PointMatcher<T>::DataPointsFilter
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
		return "Conditional subsampling. This filter reduces the size of the point cloud by randomly dropping points if their number is above maxCount. Based on \\cite{Masuda1996Random}";
	}
	inline static const ParametersDoc availableParameters()
	{
		return boost::assign::list_of<ParameterDoc>
		( "seed", "srand seed", "1", "0", "2147483647", &P::Comp<unsigned> )
		( "maxCount", "maximum number of points", "1000", "0", "2147483647", &P::Comp<unsigned> )
		;
	}

	const unsigned maxCount;
	unsigned seed;

	MaxPointCountDataPointsFilter(const Parameters& params = Parameters());
	virtual ~MaxPointCountDataPointsFilter() {};
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};

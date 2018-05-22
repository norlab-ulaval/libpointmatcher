#pragma once

#include "PointMatcher.h"

#include <string>

//! Random sampling
template<typename T>
struct RandomSamplingDataPointsFilter: public PointMatcher<T>::DataPointsFilter
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
		return "Subsampling. This filter reduces the size of the point cloud by randomly dropping points. Based on \\cite{Masuda1996Random}";
	}
	inline static const ParametersDoc availableParameters()
	{
		return boost::assign::list_of<ParameterDoc>
			( "prob", "probability to keep a point, one over decimation factor ", "0.75", "0", "1", &P::Comp<T> )
		;
	}
	
	const double prob;
	
	RandomSamplingDataPointsFilter(const Parameters& params = Parameters());
	virtual ~RandomSamplingDataPointsFilter() {};
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};

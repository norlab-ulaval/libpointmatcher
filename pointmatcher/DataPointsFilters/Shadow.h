#pragma once

#include "PointMatcher.h"

//! Shadow filter, remove ghost points appearing on edges
template<typename T>
struct ShadowDataPointsFilter: public PointMatcher<T>::DataPointsFilter
{
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	typedef Parametrizable::InvalidParameter InvalidParameter;
	
	typedef typename PointMatcher<T>::Vector Vector;
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;
	
	inline static const std::string description()
	{
		return "Remove ghost points appearing on edge discontinuties. Assume that the origine of the point cloud is close to where the laser center was. Requires surface normal for every points";
	}
	
	inline static const ParametersDoc availableParameters()
	{
		return boost::assign::list_of<ParameterDoc>
			( "eps", "Small angle (in rad) around which a normal shoudn't be observable", "0.1", "0.0", "3.1416", &P::Comp<T> )
		;
	}

	const T eps;

	//! Constructor, uses parameter interface
	ShadowDataPointsFilter(const Parameters& params = Parameters());
	
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};

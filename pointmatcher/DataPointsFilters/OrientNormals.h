#pragma once

#include "PointMatcher.h"

//! Reorientation of normals
template<typename T>
struct OrientNormalsDataPointsFilter: public PointMatcher<T>::DataPointsFilter
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
		return "Normals. Reorient normals so that they all point in the same direction, with respect to the observation points.";
	}
	
	inline static const ParametersDoc availableParameters()
	{
		return boost::assign::list_of<ParameterDoc>
			( "towardCenter", "If set to true(1), all the normals will point inside the surface (i.e. toward the observation points).", "1", "0", "1", &P::Comp<bool> )
		;
	}

	OrientNormalsDataPointsFilter(const Parameters& params = Parameters());
	virtual ~OrientNormalsDataPointsFilter() {};
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);

	const bool towardCenter;
};

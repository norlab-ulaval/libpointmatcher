#pragma once

#include "PointMatcher.h"

//! Subsampling. Remove point laying in a bounding box
template< typename T>
struct BoundingBoxDataPointsFilter: public PointMatcher<T>::DataPointsFilter
{
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	typedef Parametrizable::InvalidParameter InvalidParameter;
	
	typedef typename PointMatcher<T>::Vector Vector;
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	
	inline static const std::string description()
	{
		return "Subsampling. Remove points laying in a bounding box which is axis aligned.";
	}
	inline static const ParametersDoc availableParameters()
	{
		return boost::assign::list_of<ParameterDoc>
			( "xMin", "minimum value on x-axis defining one side of the bounding box", "-1", "-inf", "inf", &P::Comp<T> )
			( "xMax", "maximum value on x-axis defining one side of the bounding box", "1", "-inf", "inf", &P::Comp<T> )
			( "yMin", "minimum value on y-axis defining one side of the bounding box", "-1", "-inf", "inf", &P::Comp<T> )
			( "yMax", "maximum value on y-axis defining one side of the bounding box", "1", "-inf", "inf", &P::Comp<T> )
			( "zMin", "minimum value on z-axis defining one side of the bounding box", "-1", "-inf", "inf", &P::Comp<T> )
			( "zMax", "maximum value on z-axis defining one side of the bounding box", "1", "-inf", "inf", &P::Comp<T> )
			( "removeInside", "If set to true (1), remove points inside the bounding box; else (0), remove points outside the bounding box", "1", "0", "1", P::Comp<bool> )
		;
	}

	const T xMin;
	const T xMax;
	const T yMin;
	const T yMax;
	const T zMin;
	const T zMax;
	const bool removeInside;
	
	//! Constructor, uses parameter interface
	BoundingBoxDataPointsFilter(const Parameters& params = Parameters());
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};

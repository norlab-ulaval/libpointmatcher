#pragma once

#include "PointMatcher.h"

//! Extract observation direction
template<typename T>
struct ObservationDirectionDataPointsFilter: public PointMatcher<T>::DataPointsFilter
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
		return "This filter extracts observation directions (vector from point to sensor), considering a sensor at position (x,y,z).\n\n"
			   "Required descriptors: none.\n"
		       "Produced descritors:  observationDirections.\n"
			   "Altered descriptors:  none.\n"
			   "Altered features:     none.";
	}
	
	inline static const ParametersDoc availableParameters()
	{
		return boost::assign::list_of<ParameterDoc>
			( "x", "x-coordinate of sensor", "0" )
			( "y", "y-coordinate of sensor", "0" )
			( "z", "z-coordinate of sensor", "0" )
		;
	}

	const T centerX;
	const T centerY;
	const T centerZ;

	//! Constructor, uses parameter interface
	ObservationDirectionDataPointsFilter(const Parameters& params = Parameters());
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};

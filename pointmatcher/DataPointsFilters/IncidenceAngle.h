#pragma once

#include "PointMatcher.h"

//! Incidence angle, compute the incidence angle of a surface normal with the observation direction
template<typename T>
struct IncidenceAngleDataPointsFilter: public PointMatcher<T>::DataPointsFilter
{	
	typedef typename PointMatcher<T>::Vector Vector;
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;
	
	inline static const std::string description()
	{
		return "Compute the incidence angle using the dot product of the viewing direction and the surface normal.\n\n"
			   "Required descriptors: normals, observationDirections.\n"
		       "Produced descritors:  incidenceAngles.\n"
			   "Altered descriptors:  none.\n"
			   "Altered features:     none.";
	}
	
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};

#pragma once

#include "PointMatcher.h"

//! Sick LMS-xxx noise model
template<typename T>
struct SimpleSensorNoiseDataPointsFilter: public PointMatcher<T>::DataPointsFilter
{
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	typedef Parametrizable::InvalidParameter InvalidParameter;

	typedef typename PointMatcher<T>::Matrix Matrix;	
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;
	
	inline static const std::string description()
	{
		return "Add a 1D descriptor named <sensorNoise> that would represent the noise radius expressed in meter based on SICK LMS specifications \\cite{Pomerleau2012Noise}.";
	}
	
	inline static const ParametersDoc availableParameters()
	{
		return boost::assign::list_of<ParameterDoc>
			( "sensorType", "Type of the sensor used. Choices: 0=Sick LMS-1xx, 1=Hokuyo URG-04LX, 2=Hokuyo UTM-30LX, 3=Kinect/Xtion", "0", "0", "2147483647", &P::Comp<unsigned> )
			( "gain", "If the point cloud is coming from an untrusty source, you can use the gain to augment the uncertainty", "1", "1", "inf", &P::Comp<T> )
		;
	}

	const unsigned sensorType;
	const T gain;
	
	//! Constructor, uses parameter interface
	SimpleSensorNoiseDataPointsFilter(const Parameters& params = Parameters());
	
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);

private:
	/// @param minRadius in meter, noise level of depth measurements
	/// @param beamAngle in rad, half of the total laser beam
	/// @param beamConst in meter, minimum size of the laser beam
	/// @param features points from the sensor
	Matrix computeLaserNoise(const T minRadius, const T beamAngle, const T beamConst, const Matrix& features);

};

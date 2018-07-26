// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2018,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#pragma once

#include "PointMatcher.h"
#include <array>

template<typename T>
struct RemoveSensorBiasDataPointsFilter: public PointMatcher<T>::DataPointsFilter
{
	// Type definitions
	typedef PointMatcher<T> PM;
	typedef typename PM::DataPoints DataPoints;
	typedef typename PM::DataPointsFilter DataPointsFilter;
	
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	
	typedef Parametrizable::InvalidParameter InvalidParameter;
	
	typedef typename PM::Matrix Matrix;
	typedef typename PM::Vector Vector;
	
	typedef typename DataPoints::InvalidField InvalidField;
	
	inline static const std::string description()
	{
		return "Remove the bias induced by the angle of incidence\n\n";
			  // "Required descriptors: incidenceAngles, observationDirections.\n"
		      // "Produced descritors:  none.\n"
			  // "Altered descriptors:  none.\n"
			  // "Altered features:     none.";
	}
	
	inline static const ParametersDoc availableParameters()
	{
		return boost::assign::list_of<ParameterDoc>
			( "sensorType", "Type of the sensor used. Choices: 0=Sick LMS-1xx, 1=Velodyne HDL-32E", "0", "0", "255", &P::Comp<int> )
		;
	}
	
	enum SensorType : int { LMS_1XX=0, HDL_32E=1}; //add sensor here
	
//attributes here
	const SensorType sensorType;
	
	//! Constructor, uses parameter interface
	RemoveSensorBiasDataPointsFilter(const Parameters& params = Parameters());
	
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);


private:
	const T tau=50e-9;//s - pulse length
	const T pulse_intensity=0.39;//w.m^-2 - pulse intensity
	const T lambda_light=905e-9;//m - wavelength of the laser
	const T c = 299792458.0; // m.s^-1 - celerity of light

	std::array<T,4> getCoefficients(const T depth, const T theta, const T aperture);
	T diffDist(const T depth, const T theta, const T aperture);
	T ratioCurvature(const T depth, const T theta, const T aperture);

	struct SensorParameters{
	private:
		SensorParameters(T aperture_, T k1_, T k2_):
			aperture{aperture_},
			k1{k1_},
			k2{k2_}
		{
		
		}
	public:
		const T aperture;
		const T k1;
		const T k2;
	
		static const SensorParameters LMS_1xx;
		static const SensorParameters HDL_32E;
	};
};

template<typename T>
const typename RemoveSensorBiasDataPointsFilter<T>::SensorParameters RemoveSensorBiasDataPointsFilter<T>::SensorParameters::LMS_1xx =
	RemoveSensorBiasDataPointsFilter<T>::SensorParameters(1.413717e-2,  1.54987849, 0.00359711);

template<typename T>
const typename RemoveSensorBiasDataPointsFilter<T>::SensorParameters RemoveSensorBiasDataPointsFilter<T>::SensorParameters::HDL_32E =
	RemoveSensorBiasDataPointsFilter<T>::SensorParameters(2.967060e-3,2.52773563, 0.007084910);


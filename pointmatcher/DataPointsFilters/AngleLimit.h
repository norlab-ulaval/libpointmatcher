// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2024,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
Matej Boxan, Norlab, Université Laval, Canada
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net> and <matej dot boxan at gmail dot com>

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

//! Subsampling. Filter points if they lie inside / outside of a given spherical wedge.
//! The spherical wedge is defined by the lower and upper bounds of two angles in spherical coordinates.
//! The angles follow the spherical physics convention, as described on Wikipedia https://en.wikipedia.org/wiki/Spherical_coordinate_system
template< typename T>
struct AngleLimitDataPointsFilter: public PointMatcher<T>::DataPointsFilter
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
		return "Filter points based on a given angle range in spherical coordinates. Assumes the angles in degrees.";
	}
	inline static const ParametersDoc availableParameters()
	{
		return {
				{"phiMin", "Lower bound of the azimuthal angle", "-inf", "-inf", "inf", &P::Comp<T>},
				{"phiMax", "Upper bound of the azimuthal angle", "-inf", "-inf", "inf", &P::Comp<T>},
				{"thetaMin", "Lower bound of the polar angle", "-inf", "-inf", "inf", &P::Comp<T>},
				{"thetaMax", "Upper bound of the polar angle", "-inf", "-inf", "inf", &P::Comp<T>},
				{"removeInside", "If set to true (1), remove points inside the spherical wedge; else (0), remove points outside the wedge", "1", "0", "1", P::Comp<bool>}
		};
	}

	const T phiMin;
	const T phiMax;
	const T thetaMin;
	const T thetaMax;
	const bool removeInside;

	//! Constructor, uses parameter interface
	AngleLimitDataPointsFilter(const Parameters& params = Parameters());
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};

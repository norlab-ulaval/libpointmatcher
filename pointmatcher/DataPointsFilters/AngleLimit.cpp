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
#include "AngleLimit.h"
#include "pointmatcher/Functions.h"

// AngleLimitDataPointsFilter
// Constructor
template<typename T>
AngleLimitDataPointsFilter<T>::AngleLimitDataPointsFilter(const Parameters& params) :
		PointMatcher<T>::DataPointsFilter("AngleLimitDataPointsFilter",
										  AngleLimitDataPointsFilter::availableParameters(), params),
		phiMin(Parametrizable::get<T>("phiMin")),
		phiMax(Parametrizable::get<T>("phiMax")),
		thetaMin(Parametrizable::get<T>("thetaMin")),
		thetaMax(Parametrizable::get<T>("thetaMax")),
		removeInside(Parametrizable::get<bool>("removeInside"))
{
}

template<typename T>
typename PointMatcher<T>::DataPoints AngleLimitDataPointsFilter<T>::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void AngleLimitDataPointsFilter<T>::inPlaceFilter(
	DataPoints& cloud)
{
	using namespace PointMatcherSupport;


	const int nbPoints = cloud.getNbPoints();
	const int nbRows = cloud.features.rows();

	int j = 0;
    for(int i = 0; i < nbPoints; ++i)
    {
        auto point = cloud.features.col(i).head(nbRows-1);
        T r = point.norm();
        T r_plane = point.head(nbRows-2).norm();
        T thetaPoint = PointMatcherSupport::radToDeg(acos(point(nbRows-2) / r));
        T phiPoint = PointMatcherSupport::radToDeg(acos(point(0) / r_plane));

        if (point(1) < 0)
            phiPoint = -phiPoint;

        if(removeInside)
        {
            if((phiPoint < phiMin || phiMax < phiPoint) || (thetaPoint < thetaMin || thetaMax < thetaPoint)) // point is outside range, keep it
            {
                cloud.setColFrom(j, cloud, i);
                ++j;
            }
        }
        else
        {
            if (phiMin < phiPoint && phiPoint < phiMax && thetaMin < thetaPoint && thetaPoint < thetaMax) // point is inside range, keep it
            {
                cloud.setColFrom(j, cloud, i);
                ++j;
            }
        }
    }
	cloud.conservativeResize(j);
}

template struct AngleLimitDataPointsFilter<float>;
template struct AngleLimitDataPointsFilter<double>;

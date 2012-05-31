// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2011,
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

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>

using namespace std;
using namespace PointMatcherSupport;

int main(int argc, char *argv[])
{
	if (argc != 3)
	{
		cerr << "Usage " << argv[0] << " INPUT.csv OUTPUT.vtk\n";
		return 1;
	}

	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DataPoints;

	PM pm;
	
	setLogger(pm.LoggerRegistrar.create("FileLogger"));
	DataPoints d = PM::loadCSV(argv[1]);
	PM::DataPointsFilter* dataPointFilter1;
	PM::DataPointsFilter* dataPointFilter2;
	PM::DataPointsFilter* dataPointFilter3;
	PM::DataPointsFilter* dataPointFilter4;
	PM::DataPointsFilter* dataPointFilter5;
	PM::DataPointsFilter* dataPointFilter6;
	PM::DataPointsFilter* dataPointFilter7;

	dataPointFilter1 = pm.DataPointsFilterRegistrar.create(
		"MinDistDataPointsFilter", PM::Parameters({
			{"minDist", "0.45"}
			}));
	

	dataPointFilter3 = pm.DataPointsFilterRegistrar.create(
		"SamplingSurfaceNormalDataPointsFilter", PM::Parameters({
			{"ratio", "0.8"},
			{"binSize", "20"},
			{"samplingMethod", "1"},
			{"keepNormals", "1"},
			{"keepDensities", "1"},
			}));
	
	dataPointFilter4 = pm.DataPointsFilterRegistrar.create(
		"OrientNormalsDataPointsFilter"
		);

	

	dataPointFilter6 = pm.DataPointsFilterRegistrar.create(
		"ShadowDataPointsFilter", PM::Parameters({
			{"eps", "0.1"}
			}));

	dataPointFilter7 = pm.DataPointsFilterRegistrar.create(
		"SimpleSensorNoiseDataPointsFilter", PM::Parameters({
			{"sensorType", "0"}
			}));

	PM::DataPointsFilter* subSample;
	subSample= pm.DataPointsFilterRegistrar.create(
		"RandomSamplingDataPointsFilter", PM::Parameters({
			{"prob", "0.1"}
			}));

	PM::DataPointsFilter* maxDensity;
	maxDensity = pm.DataPointsFilterRegistrar.create(
		"MaxDensityDataPointsFilter"
		);
	
	PM::DataPointsFilter* computeDensity;
	computeDensity = pm.DataPointsFilterRegistrar.create(
		"SurfaceNormalDataPointsFilter", PM::Parameters({
			{"knn", "20"},
			{"keepDensities", "1"}
		}));


	//d = dataPointFilter1->filter(d);
	//d = dataPointFilter2->filter(d);
	//d = dataPointFilter3->filter(d);
	//d = dataPointFilter4->filter(d);
	//d = dataPointFilter5->filter(d);
	//d = dataPointFilter6->filter(d);
	//d = dataPointFilter7->filter(d);
	
	//d = subSample->filter(d);
	d = computeDensity->filter(d);
	//d = maxDensity->filter(d);
	//d = computeDensity->filter(d);

	// Example of moving 3D points
	//Eigen::Matrix4f T;
	//T << 0.98106,	0.17298,	-0.08715, 0.1, -0.15610,	0.97247,	0.17298, 0.2, 0.11468,	-0.15610,	0.98106, 0, 0,0,0,1;
	//cout << "Moving points using: " << endl << T << endl;
	
	//d.features = T * d.features;
	
	PM::saveVTK(d, argv[2]);
	
	return 0;
}

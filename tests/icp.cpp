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

int main(int argc, char *argv[])
{
	if (argc != 3)
	{
		cerr << "Error in command line, usage " << argv[0] << " reference.csv reading.csv" << endl;
		return 1;
	}
	
	typedef float T;
	typedef MetricSpaceAligner<T> MSA;
	MSA::ICP icp;
	
	icp.transformations.push_back(new MSA::TransformFeatures());
	icp.transformations.push_back(new MSA::TransformDescriptors());
	
	icp.readingDataPointsFilters.push_back(new MSA::RandomSamplingDataPointsFilter(0.5));
	
	icp.referenceDataPointsFilters.push_back(new MSA::RandomSamplingDataPointsFilter(0.5));
	//icp.referenceDataPointsFilters.push_back(new MSA::SurfaceNormalDataPointsFilter(10, 0, true, true, true, true, true));
	
	icp.matcher = new MSA::KDTreeMatcher();
	
	icp.featureOutlierFilters.push_back(new MSA::MaxDistOutlierFilter(0.05));
	//icp.featureOutlierFilters.push_back(new MSA::MedianDistOutlierFilter(3));
	//icp.featureOutlierFilters.push_back(new MSA::TrimmedDistOutlierFilter(0.85));
	
	icp.descriptorOutlierFilter = new MSA::NullDescriptorOutlierFilter();

	icp.errorMinimizer = new MSA::PointToPointErrorMinimizer();
	//icp.errorMinimizer = new MSA::PointToPlaneErrorMinimizer();
	
	icp.transformationCheckers.push_back(new MSA::CounterTransformationChecker(60));
	icp.transformationCheckers.push_back(new MSA::ErrorTransformationChecker(0.001, 0.001, 3));
	
	icp.inspector = new MSA::VTKFileInspector("test");
	//icp.inspector = new MSA::Inspector;
	
	icp.outlierMixingWeight = 1;
	
	typedef MSA::TransformationParameters TP;
	typedef MSA::DataPoints DP;
	
	const DP ref(loadCSV<MSA::ScalarType>(argv[1]));
	DP data(loadCSV<MSA::ScalarType>(argv[2]));
	TP t(TP::Identity(data.features.rows(), data.features.rows()));
	
	for (int i = 0; i < data.features.cols(); ++i)
	{
		data.features.block(0, i, 2, 1) = Eigen::Rotation2D<T>(0.2) * data.features.block(0, i, 2, 1);
		data.features(0, i) += 0.2;
		data.features(1, i) -= 0.1;
	}
	
	TP res = icp(t, data, ref);

	return 0;
}

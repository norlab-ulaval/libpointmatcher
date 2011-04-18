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
#include <fstream>
#include <boost/format.hpp>

using namespace std;

/**
  * Code example for ICP taking a sequence of point clouds relatively close 
  * and computing the transformation between them.
  */
int main(int argc, char *argv[])
{
	if (argc != 3)
	{
		cerr << "Error in command line, usage " << argv[0] << " inputBaseFileName outputBaseFileName" << endl;
		cerr << endl << "Example:" << endl;
		cerr << argv[0] << " ../examples/data/cloud ./cloud_out" << endl << endl;

		return 1;
	}
	
	const string inputBaseFileName(argv[1]);
	const string outputBaseFileName(argv[2]);
	
	typedef MetricSpaceAlignerD MSA;
	
	// Main algorithm definition
	MSA::ICP icp;
	// Also available ICPSequence
	
	// Defines which space needs is influenced by the transformation
	icp.transformations.push_back(new MSA::TransformFeatures());
	//icp.transformations.push_back(new MSA::TransformDescriptors());
	
	// Defines preprocessing filters for reference and reading point clouds
	icp.readingDataPointsFilters.push_back(new MSA::RandomSamplingDataPointsFilter(0.5));
	icp.referenceDataPointsFilters.push_back(new MSA::RandomSamplingDataPointsFilter(0.5));
	icp.referenceDataPointsFilters.push_back(new MSA::SurfaceNormalDataPointsFilter(15, 0, true, false, false, false, false));
	
	// Defines the matching method
	icp.matcher = new MSA::KDTreeMatcher();

	// Defines features outlier filters. It can be stacked up.
	//icp.featureOutlierFilters.push_back(new MSA::MedianDistOutlierFilter(25));
	icp.featureOutlierFilters.push_back(new MSA::TrimmedDistOutlierFilter(0.92));
	//icp.featureOutlierFilters.push_back(new MSA::MaxDistOutlierFilter(0.5));
	//icp.featureOutlierFilters.push_back(new MSA::MinDistOutlierFilter(0.000001));

	// Defines descriptor outlier filters (not implemented yet)
	icp.descriptorOutlierFilter = new MSA::NullDescriptorOutlierFilter();

	// Defines how to mix feature and descriptor outlier (not implemented yet)
	icp.outlierMixingWeight = 1;
	
	// Defines the type of error to minimize
	//icp.errorMinimizer = newdd MSA::PointToPointErrorMinimizer();
	icp.errorMinimizer = new MSA::PointToPlaneErrorMinimizer();
	
	// Defines out conditions for the iterative loop. It can be stacked up.
	icp.transformationCheckers.push_back(new MSA::CounterTransformationChecker(40));
	icp.transformationCheckers.push_back(new MSA::ErrorTransformationChecker(0.001, 0.01, 3));
	
	// Defines how to inspect the iterative process
	// This will output a VTK file for each iteration
	icp.inspector = new MSA::VTKFileInspector(outputBaseFileName);
	//icp.inspector = new MSA::NullInspector;
	
	
	typedef MSA::TransformationParameters TP;
	typedef MSA::DataPoints DP;
	
	
	DP lastCloud, newCloud;
	MSA::TransformFeatures tf;
	TP tp;
	for (unsigned frameCounter = 0; frameCounter < 10; ++frameCounter)
	{
		const string inputFileName((boost::format("%s.%05d.vtk") % inputBaseFileName % frameCounter).str());
		const string outputFileName((boost::format("%s.%05d.vtk") % outputBaseFileName % frameCounter).str());

		ifstream ifs(inputFileName.c_str());
		if (!ifs.good())
		{
			cout << "Stopping at frame " << frameCounter << endl;
			break;
		}

		newCloud = loadVTK<MSA::ScalarType>(ifs);
		if (frameCounter == 0)
		{
			tp = TP::Identity(newCloud.features.rows(), newCloud.features.rows());
		}	
		
		if (frameCounter != 0)
		{
			tp = icp(tp, newCloud, lastCloud);
			newCloud = tf.compute(newCloud, tp);

			cout << "Transformation of frame " << frameCounter << " to " << frameCounter - 1 << endl;
			cout << tp << endl;
		}
		
		cout << "outputFileName: " << outputFileName << endl;
		saveVTK<MSA::ScalarType>(newCloud, outputFileName);
		lastCloud = newCloud;
	}
	return 0;
}

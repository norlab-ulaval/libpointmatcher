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

void validateArgs(int argc, char *argv[]);

/**
  * Code example for ICP taking a sequence of point clouds relatively close 
  * and computing the transformation between them.
  */
int main(int argc, char *argv[])
{
	validateArgs(argc, argv);
	
	typedef MetricSpaceAligner<float> MSA;
	
	const string inputBaseFileName(argv[1]);
	const string outputBaseFileName(argv[2]);
	
	
	// Main algorithm definition
	MSA::ICPSequence icp(3);

	icp.setDefault();
	
	typedef MSA::TransformationParameters TP;
	typedef MSA::DataPoints DP;
	
	
	MSA::DataPoints lastCloud, newCloud;
	MSA::TransformFeatures transform;
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

		if(frameCounter == 3)
			abort();

		newCloud = loadVTK<MSA::ScalarType>(ifs);
		// call icp
		try 
		{
			tp = icp(newCloud);
			//tp = icp.getDeltaTransform();
			//cout << "Transformation: "<< tp << endl;
			cout << "match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl;
			
			newCloud = transform.compute(newCloud, tp);
		}
		catch (MSA::ConvergenceError error)
		{
			cout << "ERROR MSA::ICP failed to converge: " << endl;
			cout << "   " << error.what() << endl;
			cout << "Reseting tracking" << endl;
			icp.resetTracking(newCloud);
		}
		
		cout << "outputFileName: " << outputFileName << endl;
		saveVTK<MSA::ScalarType>(newCloud, outputFileName);
	}
	return 0;
}

void validateArgs(int argc, char *argv[])
{
	if (argc != 3)
	{
		cerr << "Error in command line, usage " << argv[0] << " inputBaseFileName outputBaseFileName" << endl;
		cerr << endl << "Example:" << endl;
		cerr << argv[0] << " ../examples/data/cloud ./cloud_out" << endl << endl;

		abort();
	}
}

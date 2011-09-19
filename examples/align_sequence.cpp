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
void setupArgs(int argc, char *argv[], unsigned int& startId, unsigned int& endId, string& extension);

/**
  * Code example for ICP taking a sequence of point clouds relatively close 
  * and computing the transformation between them.
  */
int main(int argc, char *argv[])
{
	validateArgs(argc, argv);

	unsigned int startId, endId;
	string extension = "vtk";
	setupArgs(argc, argv, startId, endId, extension);
	
	

	typedef PointMatcher<float> PM;
	
	const string inputBaseFileName(argv[1]);
	const string outputBaseFileName(argv[2]);
	
	
	// Main algorithm definition
	PM::ICPSequence icp(3);

	icp.setDefault();

	typedef PM::TransformationParameters TP;
	typedef PM::DataPoints DP;
	
	
	PM::DataPoints lastCloud, newCloud;
	PM::TransformFeatures transform;
	TP tp;
	for (unsigned frameCounter = startId; frameCounter < endId; ++frameCounter)
	{
		const string inputFileName((boost::format("%s.%05d.%s") % inputBaseFileName % frameCounter % extension).str());
		const string outputFileName((boost::format("%s.%05d.vtk") % outputBaseFileName % frameCounter).str());

		ifstream ifs(inputFileName.c_str());
		if (!ifs.good())
		{
			cout << "Stopping at frame " << frameCounter << endl;
			cout << "No file named " << inputFileName << endl;
			break;
		}

		if(frameCounter == 3)
			abort();

		if(extension == "vtk")
			newCloud = loadVTK<PM::ScalarType>(ifs);
		else if (extension == "csv")
			newCloud = loadCSV<PM::ScalarType>(ifs);
		else
		{
			cerr << "Unkowned extension" << endl;
			abort();
		}
		
		// call icp
		try 
		{
			tp = icp(newCloud);
			//tp = icp.getDeltaTransform();
			//cout << "Transformation: "<< tp << endl;
			cout << "match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl;
			
			newCloud = transform.compute(newCloud, tp);
		}
		catch (PM::ConvergenceError error)
		{
			cout << "ERROR PM::ICP failed to converge: " << endl;
			cout << "   " << error.what() << endl;
			cout << "Reseting tracking" << endl;
			icp.resetTracking(newCloud);
		}
		
		cout << "outputFileName: " << outputFileName << endl;
		saveVTK<PM::ScalarType>(newCloud, outputFileName);
	}
	return 0;
}

void validateArgs(int argc, char *argv[])
{
	if (!(argc == 3 || argc == 5 || argc == 6))
	{
		cerr << "Error in command line, usage " << argv[0] << " inputBaseFileName outputBaseFileName [startId endId] [-csv]" << endl;
		cerr << endl << "Example:" << endl;
		cerr << argv[0] << " ../examples/data/cloud ./cloud_out" << endl << endl;

		abort();
	}
}


void setupArgs(int argc, char *argv[], unsigned int& startId, unsigned int& endId, string& extension)
{
	if(argc >= 5)
	{
		startId = atoi(argv[3]);
		endId = atoi(argv[4]);
	}
	else
	{
		startId = 0;
		endId = 1000;
	}

	if(argc == 6)
	{
		const string command(argv[5]);
		if(command == "-csv")
			extension = "csv";
	}
}

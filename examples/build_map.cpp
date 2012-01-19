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
#include "boost/filesystem/path.hpp"
#include "boost/filesystem/operations.hpp"
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace PointMatcherSupport;

void validateArgs(int argc, char *argv[]);
void setupArgs(int argc, char *argv[], unsigned int& startId, unsigned int& endId, string& extension);
vector<string> loadYamlFile(string listFileName);

/**
  * Code example for DataFilter taking a sequence of point clouds with  
  * their global coordinates and build a map with a fix (manageable) number of points
  */
int main(int argc, char *argv[])
{
	validateArgs(argc, argv);

	typedef PointMatcher<float> PM;
	typedef PM::TransformationParameters TP;
	typedef PM::DataPoints DP;

	// Process arguments
	PM::FileList list = PM::loadList(argv[1]);
	const unsigned totalPointCount = boost::lexical_cast<unsigned>(argv[2]);	
	string outputFileName(argv[3]);
	
	
	PM pm;
	
	setLogger(pm.LoggerRegistrar.create("FileLogger"));
	
	PM::DataPoints mapCloud;

	PM::DataPoints lastCloud, newCloud;
	TP T = TP::Identity(4,4);

	for(unsigned i=0; i < list.size(); i++)
	{
		if(list[i].fileExtension == ".vtk")
			newCloud = PM::loadVTK(list[i].readingPath);
		else if(list[i].fileExtension == ".csv")
			newCloud = PM::loadCSV(list[i].readingPath);
		else
		{
			cout << "Only VTK or CSV files are supported" << endl;
			abort();
		}

		cout << "Point cloud loaded" << endl;
	
		if(list[i].initTransformation.rows() != 0)
			T = list[i].initTransformation;
		
		PM::Parameters params;
		
		// Remove the scanner
		PM::DataPointsFilter* removeScanner;
		removeScanner = pm.DataPointsFilterRegistrar.create("MinDistDataPointsFilter", PM::Parameters({{"minDist", "1.0"}}));
		newCloud = removeScanner->filter(newCloud);


		// Accelerate the process and disolve lines
		PM::DataPointsFilter* randSubsample;
		params = PM::Parameters({{"prob", toParam(0.75)}});
		randSubsample = pm.DataPointsFilterRegistrar.create("RandomSamplingDataPointsFilter", params);
		newCloud = randSubsample->filter(newCloud);
		
		// Build filter to remove shadow points and down sample
		params = PM::Parameters({{"binSize", "20"},{"epsilon", "5"}, {"ratio", "0.25"}});
		PM::DataPointsFilter* normalFilter;
		normalFilter = pm.DataPointsFilterRegistrar.create("SamplingSurfaceNormalDataPointsFilter", params);
		newCloud = normalFilter->filter(newCloud);


		PM::DataPointsFilter* shadowFilter;
		shadowFilter = pm.DataPointsFilterRegistrar.create("ShadowDataPointsFilter");
		newCloud = shadowFilter->filter(newCloud);

		// Transforme pointCloud
		newCloud.features = T * newCloud.features;

		if(i==0)
		{
			mapCloud = newCloud;
		}
		else
		{
			mapCloud.concatenate(newCloud);

			// Control point density
			PM::DataPointsFilter* uniformSubsample;
			params = PM::Parameters({{"aggressivity", toParam(0.15)}});
			uniformSubsample = pm.DataPointsFilterRegistrar.create("UniformizeDensityDataPointsFilter", params);
			
			mapCloud = uniformSubsample->filter(mapCloud);

			// Control point cloud size
			const double probToKeep = totalPointCount/(double)mapCloud.features.cols();
			if(probToKeep < 1)
			{
				cout << "Randomly keep " << probToKeep*100 << "\% points" << endl; 
				
				params = PM::Parameters({{"prob", toParam(probToKeep)}});
				randSubsample = pm.DataPointsFilterRegistrar.create("RandomSamplingDataPointsFilter", params);
				mapCloud = randSubsample->filter(mapCloud);
			}
		}

		stringstream outputFileNameIter;
		outputFileNameIter << boost::filesystem::path(outputFileName).stem() << "_" << i;
		
		cout << "Number of points: " << mapCloud.features.cols() << endl;
		cout << "OutputFileName: " << outputFileNameIter.str() << endl;
		PM::saveVTK(mapCloud, outputFileNameIter.str());

	}

	return 0;
}

void validateArgs(int argc, char *argv[])
{
	if (!(argc == 4))
	{
		cerr << "Error in command line, usage " << argv[0] << " listOfFiles.csv maxPoint outputFileName.vtk" << endl;
		abort();
	}
}




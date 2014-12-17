// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
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
#include "pointmatcher/IO.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include <boost/format.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
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
	typedef PointMatcherIO<float> PMIO;
	typedef PM::TransformationParameters TP;
	typedef PM::DataPoints DP;

	// Process arguments
	PMIO::FileInfoVector list(argv[1]);
	const unsigned totalPointCount = boost::lexical_cast<unsigned>(argv[2]);	
	string outputFileName(argv[3]);
	
	
	setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
	
	PM::DataPoints mapCloud;

	PM::DataPoints lastCloud, newCloud;
	TP T = TP::Identity(4,4);

	// Define transformation chain
	PM::Transformations transformations;
	PM::Transformation* transformPoints;
	transformPoints = PM::get().TransformationRegistrar.create("TransformFeatures");
	PM::Transformation* transformNormals;
	transformNormals = PM::get().TransformationRegistrar.create("TransformNormals");
	
	transformations.push_back(transformPoints);
	transformations.push_back(transformNormals);

	// Define filters for later use
	PM::DataPointsFilter* removeScanner(
		PM::get().DataPointsFilterRegistrar.create(
			"MinDistDataPointsFilter", 
			map_list_of
				("minDist", "1.0")
		)
	);
	
	PM::DataPointsFilter* randSubsample(
		PM::get().DataPointsFilterRegistrar.create(
			"RandomSamplingDataPointsFilter", 
			map_list_of
				("prob", toParam(0.65))
		)
	);

	PM::DataPointsFilter* normalFilter(
		PM::get().DataPointsFilterRegistrar.create(
			"SurfaceNormalDataPointsFilter",
			map_list_of
				("binSize", "10")
				("epsilon", "5") 
				("keepNormals","1")
				("keepDensities","0")
		)
	);

	PM::DataPointsFilter* densityFilter(
		PM::get().DataPointsFilterRegistrar.create(
			"SurfaceNormalDataPointsFilter",
			map_list_of
				("binSize", "10")
				("epsilon", "5") 
				("keepNormals","0")
				("keepDensities","1")
		)
	);

	PM::DataPointsFilter* orientNormalFilter(
		PM::get().DataPointsFilterRegistrar.create(
			"OrientNormalsDataPointsFilter",
			map_list_of
				("towardCenter", "1")
		)
	);

	PM::DataPointsFilter* uniformSubsample(
		PM::get().DataPointsFilterRegistrar.create(
			"MaxDensityDataPointsFilter",
			map_list_of
				("maxDensity", toParam(30))
		)
	);
	
	PM::DataPointsFilter* shadowFilter(
		PM::get().DataPointsFilterRegistrar.create(
			"ShadowDataPointsFilter"
		)
	);

	for(unsigned i=0; i < list.size(); i++)
	{
		newCloud = DP::load(list[i].readingFileName);

		cout << "Point cloud loaded" << endl;
	
		if(list[i].groundTruthTransformation.rows() != 0)
			T = list[i].groundTruthTransformation;
		else
		{
			cout << "ERROR: the field gTXX (ground truth) is required" << endl;
			abort();
		}

		PM::Parameters params;
		
		// Remove the scanner
		newCloud = removeScanner->filter(newCloud);


		// Accelerate the process and dissolve lines
		newCloud = randSubsample->filter(newCloud);
		
		// Build filter to remove shadow points and down-sample
		newCloud = normalFilter->filter(newCloud);
		newCloud = orientNormalFilter->filter(newCloud);
		newCloud = shadowFilter->filter(newCloud);

		// Transforme pointCloud
		transformations.apply(newCloud, T);

		if(i==0)
		{
			mapCloud = newCloud;
		}
		else
		{
			mapCloud.concatenate(newCloud);
			
			// Control point cloud size
			double probToKeep = totalPointCount/(double)mapCloud.features.cols();
			if(probToKeep < 1)
			{
				
				mapCloud = densityFilter->filter(mapCloud);
				mapCloud = uniformSubsample->filter(mapCloud);

				probToKeep = totalPointCount/(double)mapCloud.features.cols();
				
				if(probToKeep < 1)
				{
					cout << "Randomly keep " << probToKeep*100 << "\% points" << endl; 
					randSubsample = PM::get().DataPointsFilterRegistrar.create(
						"RandomSamplingDataPointsFilter", 
						map_list_of
							("prob", toParam(probToKeep))
					);
					mapCloud = randSubsample->filter(mapCloud);
				}
			}
		}

		stringstream outputFileNameIter;
		outputFileNameIter << boost::filesystem::path(outputFileName).stem() << "_" << i << ".vtk";
		
		cout << "Number of points: " << mapCloud.features.cols() << endl;
		mapCloud.save(outputFileNameIter.str());
		cout << "OutputFileName: " << outputFileNameIter.str() << endl;
	}
	
	mapCloud = densityFilter->filter(mapCloud);
	mapCloud = uniformSubsample->filter(mapCloud);
	
	mapCloud = densityFilter->filter(mapCloud);
	
	cout << "Number of points: " << mapCloud.features.cols() << endl;
	mapCloud.save(outputFileName);
	cout << "OutputFileName: " << outputFileName << endl;

	return 0;
}

void validateArgs(int argc, char *argv[])
{
	if (!(argc == 4))
	{
		cerr << "Error in command line, usage " << argv[0] << " listOfFiles.csv maxPoint outputFileName.vtk/.csv" << endl;
		abort();
	}
}




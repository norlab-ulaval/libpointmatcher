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


using namespace std;
using namespace PointMatcherSupport;

void validateArgs(int argc, char *argv[]);

/**
  * Code example for ICP taking a sequence of point clouds relatively close 
  * and computing the transformation between them.
  */
int main(int argc, char *argv[])
{
	validateArgs(argc, argv);

	typedef PointMatcher<float> PM;
	typedef PointMatcherIO<float> PMIO;
	typedef PM::TransformationParameters TP;
	typedef PM::DataPoints DP;
	typedef Parametrizable::Parameter Parameter;
	
	const int maxMapPointCount = 200000;

	string outputFileName(argv[0]);
	
	// Main algorithm definition
	PM::ICP icp;

	// load YAML config
	ifstream ifs(argv[1]);
	validateFile(argv[1]);
	icp.loadFromYaml(ifs);

	PMIO::FileInfoVector list(argv[2]);

	PM::DataPoints mapPointCloud, newCloud;
	TP tp;

	for(unsigned i=0; i < list.size(); i++)
	{
		cout << "---------------------\nLoading: " << list[i].readingFileName << endl; 
		newCloud = DP::load(list[i].readingFileName);
		
		if(mapPointCloud.features.rows() == 0)
		{
			mapPointCloud = newCloud;
		}
		else
		{

			// call ICP
			try 
			{
				tp = icp(mapPointCloud, newCloud);
				//tp = icp.getDeltaTransform();
				//cout << "Transformation: "<< tp << endl;
				cout << "match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl;
				
				newCloud.features = tp.inverse()*newCloud.features;
			
				PM::DataPointsFilter* densityFilter(
					PM::get().DataPointsFilterRegistrar.create(
						"SurfaceNormalDataPointsFilter",
						map_list_of
							("binSize", "10")
							("epsilon", "5") 
							("keepNormals", "0")
							("keepDensities", "1")
					)
				);

				PM::DataPointsFilter* maxDensitySubsample(
					PM::get().DataPointsFilterRegistrar.create(
						"MaxDensityDataPointsFilter",
						map_list_of
							("maxDensity", toParam(30))
					)
				);
				
				// Merge point clouds to map
				mapPointCloud.concatenate(newCloud);
				mapPointCloud = densityFilter->filter(mapPointCloud);
				mapPointCloud = maxDensitySubsample->filter(mapPointCloud);

				// Controle the size of the point cloud
				const double probToKeep = maxMapPointCount/(double)mapPointCloud.features.cols();
				if(probToKeep < 1.0)
				{
					PM::DataPointsFilter* randSubsample(
						PM::get().DataPointsFilterRegistrar.create(
							"RandomSamplingDataPointsFilter",
							map_list_of
								("prob", toParam(probToKeep))
						)
					);
					mapPointCloud = randSubsample->filter(mapPointCloud);
				}
			}
			catch (PM::ConvergenceError& error)
			{
				cout << "ERROR PM::ICP failed to converge: " << endl;
				cout << "   " << error.what() << endl;
				//cout << "Reseting tracking" << endl;
				//icp.resetTracking(newCloud);
			}

			stringstream outputFileNameIter;
			outputFileNameIter << outputFileName << "_" << i;
			
			cout << "outputFileName: " << outputFileNameIter.str() << endl;
			mapPointCloud.save(outputFileNameIter.str());
		}
	}

	return 0;
}

void validateArgs(int argc, char *argv[])
{
	if (!(argc == 3))
	{
		cerr << "Error in command line, usage " << argv[0] << " icpConfiguration.yaml listOfFiles.csv" << endl;
		cerr << endl << "Example:" << endl;
		cerr << argv[0] << " ../examples/data/default.yaml ../examples/data/carCloudList.csv" << endl << endl;

		abort();
	}
}



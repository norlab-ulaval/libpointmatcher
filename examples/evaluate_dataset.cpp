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
  * Code example for ICP evaluation taking pairs of scan with  
  * their global coordinates (first guest) and the ground truth coordinates 
  * to give evaluations of different ICP version
  */
int main(int argc, char *argv[])
{
	validateArgs(argc, argv);

	typedef PointMatcher<float> PM;
	typedef PM::TransformationParameters TP;
	typedef PM::DataPoints DP;

	// Process arguments
	PM::FileInfoVector list(argv[2]);
	string outputFileName(argv[3]);
	std::fstream resultFile;
	resultFile.open(outputFileName, fstream::out);
	if (!resultFile.good())
	{
		  cerr << "Error, invalid file " << outputFileName << endl;
		  abort();
	}
	
	PM pm;
	
	//setLogger(pm.LoggerRegistrar.create("FileLogger"));
	
	PM::DataPoints refCloud, readCloud;
	
	//TODO: handle 2D
	TP Tinit = TP::Identity(4,4);
	TP Ttruth = TP::Identity(4,4);
	TP Tout = TP::Identity(4,4);
	PM::ICP icp;
	//icp.inspector->dumpStatsHeader(resultFile);

	for(unsigned i=0; i < list.size(); i++)
	{
		readCloud = PM::loadCSV(list[i].readingFileName);
		cout << "Reading cloud " << list[i].readingFileName << " loaded" << endl;
		refCloud = PM::loadCSV(list[i].referenceFileName);
		cout << "Reference cloud " << list[i].referenceFileName << " loaded" << endl;
	
		ifstream ifs(list[i].configFileName);
		icp.loadFromYaml(ifs);

		if(list[i].initialTransformation.rows() != 0)
			Tinit = list[i].initialTransformation;
		if(list[i].groundTruthTransformation.rows() != 0)
			Ttruth = list[i].groundTruthTransformation;

		Tout = icp(refCloud, readCloud, Tinit);

		// define errors

		// Output results
		icp.inspector->dumpStatsHeader(resultFile);
		icp.inspector->dumpStats(resultFile);
	}

	resultFile.close();

	return 0;
}

void validateArgs(int argc, char *argv[])
{
	if (!(argc == 4))
	{
		cerr << "Error in command line, usage " << argv[0] << " /path/To/Data/ experimentFileName.csv resultFileName.csv" << endl;
		abort();
	}
}


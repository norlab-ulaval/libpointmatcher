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
#include "boost/filesystem.hpp"

using namespace std;

void validateArgs(int argc, char *argv[], bool& isCSV);
void basicUsage(char *argv[]);

/**
  * Code example for ICP taking 2 points clouds (2D or 3D) relatively close 
  * and computing the transformation between them.
  */
int main(int argc, char *argv[])
{
	bool isCSV = true;
	validateArgs(argc, argv, isCSV);
	
	typedef PointMatcher<float> PM;
	
	
	// Load point clouds
	PM::DataPoints ref;
	PM::DataPoints data;
	if(isCSV)
	{
		ref = loadCSV<PM::ScalarType>(argv[1]);
		data = loadCSV<PM::ScalarType>(argv[2]);
	}
	else
	{
		ref = loadVTK<PM::ScalarType>(argv[1]);
		data= loadVTK<PM::ScalarType>(argv[2]);
	}


	// Create the default ICP algorithm
	PM::ICP icp;
	// See the implementation of setDefault() to create a custom ICP algorithm
	icp.setDefault();

	// Modify the default Inspector to output vtk file
	if(argc == 4)
	{
		string baseFolder(argv[3]);
		icp.inspector = new PM::VTKFileInspector({
			{ "baseFileName", baseFolder + "test" }
		});
	}
	
	// Compute the transformation to express data in ref
	PM::TransformationParameters T = icp(data, ref);

	// Transform data to express it in ref
	PM::TransformFeatures transform;
	PM::DataPoints data_out = transform.compute(data, T);
	
	// Safe files to see the results
	saveVTK<PM::ScalarType>(ref, "test_ref.vtk");
	saveVTK<PM::ScalarType>(data, "test_data_in.vtk");
	saveVTK<PM::ScalarType>(data_out, "test_data_out.vtk");
	cout << "Final transformation:" << endl << T << endl;

	return 0;
}


void validateArgs(int argc, char *argv[], bool& isCSV )
{
	if (argc == 2)
	{
		string cmd(argv[1]);
		if(cmd == "--help")
		{
			basicUsage(argv);
			cerr << "Will create 3 vtk files for inspection: ./test_ref.vtk, ./test_data_in.vtk and ./test_data_out.vtk" << endl;
			cerr << endl << "2D Example:" << endl;
			cerr << "  " << argv[0] << " ../examples/data/2D_twoBoxes.csv ../examples/data/2D_oneBox.csv" << endl;
			cerr << endl << "3D Example:" << endl;
			cerr << "  " << argv[0] << " ../examples/data/car_cloud400.csv ../examples/data/car_cloud401.csv" << endl;
			cerr << endl << "If you enter optional REPOSITORY name, a vtk file will be created for every iteration in that repository"	<< endl << endl;

			abort();

		}
	}
	if (!(argc == 3 || argc == 4))
	{
		basicUsage(argv);
		cerr << "Use " << argv[0] << " --help for more info" << endl << endl; 
		
		abort();
	}
	
	// Validate extension
	const boost::filesystem::path pathRef(argv[1]);
	const boost::filesystem::path pathData(argv[2]);

	string refExt = boost::filesystem::extension(pathRef);
	string dataExt = boost::filesystem::extension(pathData);

	if (!(refExt == ".vtk" || refExt == ".csv"))
	{
		cout << refExt << ", " << dataExt << endl;
		cerr << "Reference file extension must be .vtk or .csv" << endl;
		abort();
	}
	
	if (!(dataExt == ".vtk" || dataExt == ".csv"))
	{
		cerr << "Reading file extension must be .vtk or .csv" << endl;
		abort();
	}

	if (dataExt != refExt)
	{
		cerr << "File extension between reference and reading should be the same" << endl;
		abort();
	}

	if (dataExt == ".csv")
		isCSV = true;
	else
		isCSV = false;
}

void basicUsage(char *argv[])
{
	cerr << endl;
	cerr << "Error in command line, usage " << argv[0] << " reference.csv reading.csv [REPOSITORY]" << endl;
	cerr << endl;
}

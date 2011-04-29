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

void validateArgs(int argc, char *argv[]);

/**
  * Code example for ICP taking 2 points clouds (2D or 3D) relatively close 
  * and computing the transformation between them.
  */
int main(int argc, char *argv[])
{
	validateArgs(argc, argv);
	
	typedef MetricSpaceAligner<float> MSA;
	
	// Load point clouds
	const MSA::DataPoints ref = loadCSV<MSA::ScalarType>(argv[1]);
	MSA::DataPoints data = loadCSV<MSA::ScalarType>(argv[2]);
	
	// Create the default ICP algorithm
	MSA::ICP icp;
	icp.setDefault();
	
	// Compute the transformation to express data in ref
	MSA::TransformationParameters T = icp(data, ref);

	// Transform data to express it in ref
	MSA::TransformFeatures transform;
	MSA::DataPoints data_out = transform.compute(data, T);
	
	// Safe files to see the results
	saveVTK<MSA::ScalarType>(ref, "test_ref.vtk");
	saveVTK<MSA::ScalarType>(data, "test_data_in.vtk");
	saveVTK<MSA::ScalarType>(data_out, "test_data_out.vtk");
	cout << "Final transformation:" << endl << T << endl;

	return 0;
}


void validateArgs(int argc, char *argv[])
{
	if (argc != 3)
	{
		cerr << endl;
		cerr << "Error in command line, usage " << argv[0] << " reference.csv reading.csv" << endl;
		cerr << endl << "2D Example:" << endl;
		cerr << "  " << argv[0] << " ../examples/data/2D_twoBoxes.csv ../examples/data/2D_oneBox.csv" << endl;
		cerr << endl << "3D Example:" << endl;
		cerr << "  " << argv[0] << " ../examples/data/car_cloud400.csv ../examples/data/car_cloud401.csv" << endl << endl;


		abort();
	}
}

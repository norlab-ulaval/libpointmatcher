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
#include "gtest/gtest.h"
#include <string>

using namespace Eigen;
using namespace std;

// Utility classes
class PointCloud2DTest: public testing::Test
{

public:
	typedef PointMatcher<float> PM;
	
	PM pm;
	PM::Inspector* vtkInspector;
	PointMatcherSupport::Logger* console;
	
	std::string dataPath;
	PM::DataPoints ref2D;
	PM::DataPoints data2D;
	
	PM::TransformationParameters validT2d;
	PM::TransformationParameters validT2dInv;
	virtual void SetUp()
	{
		// Make available a VTK inspector for manual inspection
		vtkInspector = pm.InspectorRegistrar.create(
			"VTKFileInspector", 
			PM::Parameters({{"baseFileName","./tmp/utest"}})
			);
		
		// Make available a console logger for manual inspection
		console = pm.LoggerRegistrar.create("FileLogger");

		dataPath = "../examples/data/";
		ref2D =  loadCSV<PM::ScalarType>(dataPath + "2D_oneBox.csv");
		data2D = loadCSV<PM::ScalarType>(dataPath + "2D_twoBoxes.csv");
		
		// Result of data express in ref (from visual inspection)
		validT2d = PM::TransformationParameters(3,3);
		validT2d <<  0.987498,  0.157629, 0.0859918,
		            -0.157629,  0.987498,  0.203247,
		                    0,         0,         1;
		// Result of data express in ref (from visual inspection)
		validT2dInv = PM::TransformationParameters(3,3);
		validT2dInv <<   0.988049,  -0.154139, -0.0532977,
		                 0.154139,   0.988049,   -0.20996,
		                        0,          0,          1;
	}

	virtual void TearDown()
	{	
	}

	void validate2dTransformation(PM::TransformationParameters validT, PM::TransformationParameters testT)
	{
		int dim = validT.cols();

		auto validTrans = validT.block(0, dim-1, dim-1, 1).norm();
		auto testTrans = testT.block(0, dim-1, dim-1, 1).norm();
	
		auto validAngle = acos(validT(0,0));
		auto testAngle = acos(testT(0,0));
		
		EXPECT_NEAR(validTrans, testTrans, 0.01);
		EXPECT_NEAR(validAngle, testAngle, 0.02);
	}

};



TEST_F(PointCloud2DTest, ICP_default)
{
	PM::ICP icp;
	icp.setDefault();
	
	//icp.inspector.reset(vtkInspector);
	//icp.logger.reset(console);
	PM::TransformationParameters T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);

	//icp.inspector.reset(vtkInspector);
	//icp.logger.reset(console);
	PM::TransformationParameters T2 = icp(ref2D, data2D);
	validate2dTransformation(validT2dInv, T2);
	//cout << "validT2d:\n" << validT2dInv << endl;
	//cout << "T2:\n" << T2 << endl;
}

TEST_F(PointCloud2DTest, MaxDistDataPointsFilter)
{
	PM::TransformationParameters T;

	PM::ICP icp;
	icp.setDefault();

	PM::Parameters params;
	PM::DataPointsFilter* dataPointFilter;

	string mDist = "6";
	params = PM::Parameters({{"dim","0"}, {"maxDist", mDist}});
	
	// Filter on x axis
	params["dim"] = "0";
	dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"MaxDistDataPointsFilter", params);
	
	icp.readingDataPointsFilters.clear();
	icp.readingDataPointsFilters.push_back(dataPointFilter);
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);
	
	// Filter on y axis
	params["dim"] = "1";
	dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"MaxDistDataPointsFilter", params);
	
	icp.readingDataPointsFilters.clear();
	icp.readingDataPointsFilters.push_back(dataPointFilter);
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);
	
	// Filter on z axis (not existing)
	params["dim"] = "2";
	dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"MaxDistDataPointsFilter", params);
	
	icp.readingDataPointsFilters.clear();
	icp.readingDataPointsFilters.push_back(dataPointFilter);
	EXPECT_ANY_THROW(icp(data2D, ref2D));

	// Filter on a radius
	params["dim"] = "-1";
	dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"MaxDistDataPointsFilter", params);
	
	icp.readingDataPointsFilters.clear();
	icp.readingDataPointsFilters.push_back(dataPointFilter);
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);
	
	// Parameter outside valid range
	params["dim"] = "3";
	EXPECT_ANY_THROW(
		dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"MaxDistDataPointsFilter", params)
		);
	
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}




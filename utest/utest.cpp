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
	}
	virtual void TearDown()
	{	
	}

	void validate2dTransformation(PM::TransformationParameters T)
	{
		PM::TransformationParameters Tvalid(3,3);
		
		// Result from visual inspection
		Tvalid <<  0.987498,  0.157629, 0.0859918,
		          -0.157629,  0.987498,  0.203247,
		                  0,         0,         1;

		for(int i=0; i < Tvalid.rows(); i++)
		{
			for(int j=0; j < Tvalid.cols(); j++)
			{
				if(i != Tvalid.rows()-1)
					EXPECT_NEAR(Tvalid(i,j), T(i,j), 0.001);
				else
					EXPECT_EQ(Tvalid(i,j), T(i,j));
			}
		}
	}

};



TEST_F(PointCloud2DTest, ICP_default)
{
	PM::ICP icp;
	icp.setDefault();

	PM::TransformationParameters T = icp(data2D, ref2D);
	std::cout << T << std::endl;

	validate2dTransformation(T);

	icp.inspector.reset(vtkInspector);
	icp.logger.reset(console);
	PM::TransformationParameters T2 = icp(ref2D, data2D);
	{
	PM::TransformationParameters T3 = PM::TransformationParameters::Identity(4,4);
	//__sync_synchronize();	
	//std::cout << T2.inverse() << std::endl;
	std::cout << T3.inverse() <<std::endl;
	}
	//T2.inverse();
	//validate2dTransformation(T2.inverse());
}






int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}




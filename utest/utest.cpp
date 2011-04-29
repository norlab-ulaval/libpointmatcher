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

// Utility classes
class PointCloud2DTest: public ::testing::Test
{
	typedef MetricSpaceAligner<float> MSA;

protected:
	static void SetUpTestCase()
	{

		std::string dataPath = "../example/data/";
		//ref =  loadCSV<MSA::ScalarType>(dataPath + "2D_oneBox.csv");
		//data = loadCSV<MSA::ScalarType>(dataPath + "2D_twoBoxes.csv");
	}

	static MSA::DataPoints* ref;
	static MSA::DataPoints* data;

};



TEST_F(PointCloud2DTest, ICP_default)
{
	typedef MetricSpaceAligner<float> MSA;

	std::string dataPath = "../examples/data/";
	
	MSA::DataPoints ref =  loadCSV<MSA::ScalarType>(dataPath + "2D_oneBox.csv");
	MSA::DataPoints data = loadCSV<MSA::ScalarType>(dataPath + "2D_twoBoxes.csv");
		
	MSA::ICP icp;
	icp.setDefault();

	icp(data, ref);

	//TODO: add proper evaluation of the answer
	EXPECT_TRUE(0);
}


TEST(Sandbox, vector2Eigen)
{

	std::vector<float> vec;
	vec.push_back(4);
	vec.push_back(2);
	vec.push_back(1);

	std::sort(vec.begin(), vec.end());

	Map<RowVectorXf> v(&vec[0], vec.size());
	//std::cout << v << std::endl;
	//std::cout << v.array().inverse() << std::endl;
}



int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}




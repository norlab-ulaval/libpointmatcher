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

using namespace std;
using namespace PointMatcherSupport;

// TODO: avoid global!
std::string dataPath;

// Utility classes
class PointCloud2DTest: public testing::Test
{

public:
	typedef PointMatcher<float> PM;
	
	PM pm;
	shared_ptr<PM::Inspector> vtkInspector;
	
	PM::DataPoints ref2D;
	PM::DataPoints data2D;
	
	PM::TransformationParameters validT2d;
	PM::TransformationParameters validT2dInv;
	virtual void SetUp()
	{
		// Make available a VTK inspector for manual inspection
		vtkInspector.reset(pm.InspectorRegistrar.create(
			"VTKFileInspector", 
			PM::Parameters({{"baseFileName","./unitTest"}})
			)
		);
		
		ref2D =  PM::loadCSV(dataPath + "2D_oneBox.csv");
		data2D = PM::loadCSV(dataPath + "2D_twoBoxes.csv");
		
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
		
		EXPECT_NEAR(validTrans, testTrans, 0.05);
		EXPECT_NEAR(validAngle, testAngle, 0.05);
	}

};


//---------------------------
// DataFilter modules
//---------------------------

TEST_F(PointCloud2DTest, ICP_default)
{
	PM::ICP icp;
	icp.setDefault();
	
	//icp.inspector = vtkInspector;
	//setLogger(pm.LoggerRegistrar.create("FileLogger"));
	
	PM::TransformationParameters T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);

	//icp.inspector.reset(vtkInspector);
	//setLogger(pm.LoggerRegistrar.create("FileLogger"));
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

	// Max dist has been selected to not affect the points
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


TEST_F(PointCloud2DTest, MinDistDataPointsFilter)
{
	PM::TransformationParameters T;

	PM::ICP icp;
	icp.setDefault();

	PM::Parameters params;
	PM::DataPointsFilter* dataPointFilter;

	// Min dist has been selected to not affect the points too much
	string mDist = "0.05";
	params = PM::Parameters({{"dim","0"}, {"minDist", mDist}});
	
	// Filter on x axis
	params["dim"] = "0";
	dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"MinDistDataPointsFilter", params);
	
	icp.readingDataPointsFilters.clear();
	icp.readingDataPointsFilters.push_back(dataPointFilter);
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);
	
	// Filter on y axis
	params["dim"] = "1";
	dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"MinDistDataPointsFilter", params);
	
	icp.readingDataPointsFilters.clear();
	icp.readingDataPointsFilters.push_back(dataPointFilter);
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);

	//TODO: move that to specific 2D test
	// Filter on z axis (not existing)
	params["dim"] = "2";
	dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"MinDistDataPointsFilter", params);
	
	icp.readingDataPointsFilters.clear();
	icp.readingDataPointsFilters.push_back(dataPointFilter);
	EXPECT_ANY_THROW(icp(data2D, ref2D));

	// Filter on a radius
	params["dim"] = "-1";
	dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"MinDistDataPointsFilter", params);
	
	icp.readingDataPointsFilters.clear();
	icp.readingDataPointsFilters.push_back(dataPointFilter);
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);
	
	// Parameter outside valid range
	params["dim"] = "3";
	EXPECT_ANY_THROW(
		dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"MinDistDataPointsFilter", params)
		);
	
}

TEST_F(PointCloud2DTest, MaxQuantileOnAxisDataPointsFilter)
{
	PM::TransformationParameters T;

	PM::ICP icp;
	icp.setDefault();

	PM::Parameters params;
	PM::DataPointsFilter* dataPointFilter;

	// Ratio has been selected to not affect the points too much
	string ratio = "0.95";
	params = PM::Parameters({{"dim","0"}, {"ratio", ratio}});
	
	// Filter on x axis
	params["dim"] = "0";
	dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"MaxQuantileOnAxisDataPointsFilter", params);
	
	icp.readingDataPointsFilters.clear();
	icp.readingDataPointsFilters.push_back(dataPointFilter);
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);

	// Filter on y axis
	params["dim"] = "1";
	dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"MaxQuantileOnAxisDataPointsFilter", params);
	
	icp.readingDataPointsFilters.clear();
	icp.readingDataPointsFilters.push_back(dataPointFilter);
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);

	// Filter on z axis (not existing)
	params["dim"] = "2";
	dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"MaxQuantileOnAxisDataPointsFilter", params);
	
	icp.readingDataPointsFilters.clear();
	icp.readingDataPointsFilters.push_back(dataPointFilter);
	EXPECT_ANY_THROW(icp(data2D, ref2D));


	// Parameter outside valid range
	params["dim"] = "3";
	EXPECT_ANY_THROW(
		dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"MaxQuantileOnAxisDataPointsFilter", params)
		);

}


TEST_F(PointCloud2DTest, UniformizeDensityDataPointsFilter)
{
	PM::TransformationParameters T;

	PM::ICP icp;
	icp.setDefault();

	PM::Parameters params;
	PM::DataPointsFilter* dataPointFilter;

	// Ratio has been selected to not affect the points too much
	vector<double> ratio = vector<double>({0.1, 0.15});

	for(unsigned i=0; i < ratio.size(); i++)
	{
		params = PM::Parameters({{"ratio", toParam(ratio[i])}, {"nbBin", "20"}});
		
		dataPointFilter = pm.DataPointsFilterRegistrar.create(
				"UniformizeDensityDataPointsFilter", params);
		
		icp.readingDataPointsFilters.clear();
		icp.readingDataPointsFilters.push_back(dataPointFilter);
		T = icp(data2D, ref2D);
		validate2dTransformation(validT2d, T);

		const double nbInitPts = data2D.features.cols();
		const double nbRemainingPts = icp.getNbPrefilteredReadingPts();

		// FIXME: 10% seems of seems a little bit high
		EXPECT_NEAR(nbRemainingPts/nbInitPts, 1-ratio[i], 0.10);
	}
}

TEST_F(PointCloud2DTest, SurfaceNormalDataPointsFilter)
{
	PM::TransformationParameters T;

	PM::ICP icp;
	icp.setDefault();

	// Visual validation
	//icp.inspector.reset(vtkInspector);

	PM::Parameters params;
	PM::DataPointsFilter* dataPointFilter;

	// This filter create descriptor, so parameters should'nt impact results
	params = PM::Parameters({
		{"knn", "5"}, 
		{"epsilon", "0.1"}, 
		{"keepNormals", "1"},
		{"keepDensities", "1"},
		{"keepEigenValues", "1"},
		{"keepEigenVectors", "1" },
		{"keepMatchedIds" , "1" }
		});
	// FIXME: the parameter keepMatchedIds seems to do nothing...

	// Filter on x axis
	dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"SurfaceNormalDataPointsFilter", params);
	
	icp.readingDataPointsFilters.clear();
	icp.readingDataPointsFilters.push_back(dataPointFilter);
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);
}

TEST_F(PointCloud2DTest, SamplingSurfaceNormalDataPointsFilter)
{
	PM::TransformationParameters T;

	PM::ICP icp;
	icp.setDefault();

	// Visual validation
	//icp.inspector.reset(vtkInspector);

	PM::Parameters params;
	PM::DataPointsFilter* dataPointFilter;

	// This filter create descriptor AND subsample
	params = PM::Parameters({
		{"binSize", "5"}, 
		{"averageExistingDescriptors", "1"}, 
		{"keepNormals", "1"},
		{"keepDensities", "1"},
		{"keepEigenValues", "1"},
		{"keepEigenVectors", "1" },
		{"keepMatchedIds" , "1" }
		});
		
	dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"SamplingSurfaceNormalDataPointsFilter", params);
	
	icp.readingDataPointsFilters.clear();
	icp.readingDataPointsFilters.push_back(dataPointFilter);
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);
}

TEST_F(PointCloud2DTest, OrientNormalsDataPointsFilter)
{
	PM::TransformationParameters T;

	PM::ICP icp;
	icp.setDefault();

	// Visual validation
	//icp.inspector.reset(vtkInspector);

	PM::Parameters params;
	PM::DataPointsFilter* dataPointFilter1;
	PM::DataPointsFilter* dataPointFilter2;
	
	// Used to create normal for reading point cloud
	dataPointFilter1 = pm.DataPointsFilterRegistrar.create(
			"SurfaceNormalDataPointsFilter");

	// Filter to test, shouldn't affect the results
	params = PM::Parameters({{"towardCenter", toParam(false)}});
	dataPointFilter2 = pm.DataPointsFilterRegistrar.create(
			"OrientNormalsDataPointsFilter", params);
	
	icp.readingDataPointsFilters.clear();
	icp.readingDataPointsFilters.push_back(dataPointFilter1);
	icp.readingDataPointsFilters.push_back(dataPointFilter2);
	
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);
}


TEST_F(PointCloud2DTest, RandomSamplingDataPointsFilter)
{
	PM::TransformationParameters T;

	PM::ICP icp;
	icp.setDefault();

	// Visual validation
	//icp.inspector.reset(vtkInspector);

	PM::Parameters params;
	PM::DataPointsFilter* dataPointFilter;
	
	
	vector<double> prob = {0.80, 0.85, 0.90, 0.95};
	for(unsigned i=0; i<prob.size(); i++)
	{
		// Try to avoid to low value for the reduction to avoid under sampling
		params = PM::Parameters({{"prob", toParam(prob[i])}});
		
		dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"RandomSamplingDataPointsFilter", params);
	
		icp.readingDataPointsFilters.clear();
		icp.readingDataPointsFilters.push_back(dataPointFilter);
		
		T = icp(data2D, ref2D);
		validate2dTransformation(validT2d, T);
	}
}


TEST_F(PointCloud2DTest, FixStepSamplingDataPointsFilter)
{
	PM::TransformationParameters T;

	PM::ICP icp;
	icp.setDefault();

	// Visual validation
	//icp.inspector = vtkInspector;

	PM::Parameters params;
	PM::DataPointsFilter* dataPointFilter;
	
	
	vector<unsigned> steps = {1, 2, 3};
	for(unsigned i=0; i<steps.size(); i++)
	{
		// Try to avoid too low value for the reduction to avoid under sampling
		params = PM::Parameters({{"startStep", toParam(steps[i])}});
		
		dataPointFilter = pm.DataPointsFilterRegistrar.create(
			"FixStepSamplingDataPointsFilter", params);
	
		icp.readingDataPointsFilters.clear();
		icp.readingDataPointsFilters.push_back(dataPointFilter);
		
		T = icp(data2D, ref2D);
		validate2dTransformation(validT2d, T);
	}
}

//---------------------------
// Matching modules
//---------------------------

TEST_F(PointCloud2DTest, KDTreeMatcher)
{
	PM::TransformationParameters T;

	PM::ICP icp;
	icp.setDefault();
	//setLogger(pm.LoggerRegistrar.create("FileLogger"));

	// Visual validation
	//icp.inspector = vtkInspector;

	PM::Parameters params;
	PM::Matcher* matcher;

	vector<unsigned> knn = {1, 2, 3};
	vector<double> epsilon = {0.0, 0.2};
	vector<double> maxDist = {1.0, 0.5};

	for(unsigned i=0; i < knn.size(); i++)
	{
		for(unsigned j=0; j < epsilon.size(); j++)
		{
			for(unsigned k=0; k < maxDist.size(); k++)
			{
				params = PM::Parameters({
					{"knn", toParam(knn[i])}, // remove end parenthesis for bug
					{"epsilon", toParam(epsilon[j])}, 
					{"searchType", "1"},
					{"maxDist", toParam(maxDist[k])},
					});
				
				matcher = pm.MatcherRegistrar.create(
						"KDTreeMatcher", params);

				icp.matcher.reset(matcher);
				
				T = icp(data2D, ref2D);
				validate2dTransformation(validT2d, T);
			}
		}
	}
}


//---------------------------
// Outlier modules
//---------------------------

TEST_F(PointCloud2DTest, MaxDistOutlierFilter)
{
	PM::TransformationParameters T;

	PM::ICP icp;
	icp.setDefault();

	// Visual validation
	//icp.inspector = vtkInspector;

	PM::Parameters params;
	PM::FeatureOutlierFilter* outlierFilter;
	
	params = PM::Parameters({{"maxDist", toParam(0.02)}});

	outlierFilter = pm.FeatureOutlierFilterRegistrar.create(
		"MaxDistOutlierFilter", params);
	
	icp.featureOutlierFilters.clear();
	icp.featureOutlierFilters.push_back(outlierFilter);
	
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);
}

TEST_F(PointCloud2DTest, MinDistOutlierFilter)
{
	PM::TransformationParameters T;

	PM::ICP icp;
	icp.setDefault();

	// Visual validation
	icp.inspector = vtkInspector;

	PM::Parameters params;
	PM::FeatureOutlierFilter* outlierFilter;
	
	params = PM::Parameters({{"minDist", toParam(0.002)}});

	outlierFilter = pm.FeatureOutlierFilterRegistrar.create(
		"MinDistOutlierFilter", params);

	// Since not sure how useful is that filter, we keep the one by default
	// and add that one over it
	icp.featureOutlierFilters.push_back(outlierFilter);
	
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);
}

TEST_F(PointCloud2DTest, MedianDistOutlierFilter)
{
	PM::TransformationParameters T;

	PM::ICP icp;
	icp.setDefault();

	// Visual validation
	//icp.inspector = vtkInspector;

	PM::Parameters params;
	PM::FeatureOutlierFilter* outlierFilter;
	
	params = PM::Parameters({{"factor", toParam(3.5)}});

	outlierFilter = pm.FeatureOutlierFilterRegistrar.create(
		"MedianDistOutlierFilter", params);
	
	icp.featureOutlierFilters.clear();
	icp.featureOutlierFilters.push_back(outlierFilter);
	
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);
}


TEST_F(PointCloud2DTest, TrimmedDistOutlierFilter)
{
	PM::TransformationParameters T;

	PM::ICP icp;
	icp.setDefault();

	// Visual validation
	//icp.inspector = vtkInspector;

	PM::Parameters params;
	PM::FeatureOutlierFilter* outlierFilter;
	
	params = PM::Parameters({{"ratio", toParam(0.85)}});

	outlierFilter = pm.FeatureOutlierFilterRegistrar.create(
		"TrimmedDistOutlierFilter", params);
	
	icp.featureOutlierFilters.clear();
	icp.featureOutlierFilters.push_back(outlierFilter);
	
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);
}


TEST_F(PointCloud2DTest, VarTrimmedDistOutlierFilter)
{
	PM::TransformationParameters T;

	PM::ICP icp;
	icp.setDefault();

	// Visual validation
	icp.inspector = vtkInspector;

	PM::Parameters params;
	PM::FeatureOutlierFilter* outlierFilter;
	
	params = PM::Parameters({
		{"minRatio", toParam(0.75)},
		{"maxRatio", toParam(0.90)},
		{"lambda", toParam(0.4)},
	});

	outlierFilter = pm.FeatureOutlierFilterRegistrar.create(
		"VarTrimmedDistOutlierFilter", params);
	
	icp.featureOutlierFilters.clear();
	icp.featureOutlierFilters.push_back(outlierFilter);
	
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);
}
//---------------------------
// Error modules
//---------------------------

TEST_F(PointCloud2DTest, PointToPointErrorMinimizer)
{
	PM::TransformationParameters T;

	PM::ICP icp;
	icp.setDefault();

	// Visual validation
	//icp.inspector = vtkInspector;

	PM::ErrorMinimizer* errorMin;
	
	icp.readingDataPointsFilters.clear();
	icp.keyframeDataPointsFilters.clear();
	
	errorMin = pm.ErrorMinimizerRegistrar.create(
		"PointToPointErrorMinimizer");

	icp.errorMinimizer.reset(errorMin);
	
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);
}

TEST_F(PointCloud2DTest, PointToPlaneErrorMinimizer)
{
	PM::TransformationParameters T;

	PM::ICP icp;
	icp.setDefault();

	// Visual validation
	//icp.inspector = vtkInspector;

	PM::ErrorMinimizer* errorMin;
	
	errorMin = pm.ErrorMinimizerRegistrar.create(
		"PointToPlaneErrorMinimizer");

	icp.errorMinimizer.reset(errorMin);
	
	T = icp(data2D, ref2D);
	validate2dTransformation(validT2d, T);
}

//---------------------------
// Transformation Checker modules
//---------------------------

// TODO

//---------------------------
// Main
//---------------------------
int main(int argc, char **argv)
{
	dataPath = "";
	for(int i=1; i < argc; i++)
	{
		if(strcmp(argv[i], "--path") == 0 && i+1 < argc)
				dataPath = argv[i+1];
	}

	if(dataPath == "")
	{
		cerr << "Missing the flag --path ./path/to/examples/data\n Please give the path to the test data folder which should be included with the source code. The folder is named 'examples/data'." << endl;
		return -1;
	}

	testing::GTEST_FLAG(print_time) = true;
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}




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
#include "../contrib/gtest/gtest.h"

#include <string>

#include <fstream>

#include "boost/filesystem.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/filesystem/operations.hpp"

using namespace std;
using namespace PointMatcherSupport;
using boost::assign::map_list_of;

// TODO: avoid global by using testing::Environment
// TODO: split the test into different cpp files:
// - ut_DataPoints.cpp
// - ut_IO.cpp
// - ut_Icp.cpp
// - ut_DataFilters.cpp
// - ut_Matchers.cpp
// - ut_Outliers.cpp
// - ut_ErrorMinimizers.cpp
// - ut_Transmations.cpp (checker and parameter)
// - ut_
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

std::string dataPath;

DP ref2D;
DP data2D;
DP ref3D;
DP data3D;
PM::TransformationParameters validT2d;
PM::TransformationParameters validT3d;

//---------------------------
// Test ICP with all existing filters.

// Algorithm:
// 1. Iterate over all yaml files in
//    libpointmatcher/examples/data/icp_data, each file tests ICP
//    with one or more filters.
// 2. Run ICP with the given yaml file. The filters in the yaml
//    file are applied along the way.
// 3. Write the obtained ICP transform to disk, to the same directory,
//    with file extension .cur_trans (for easy future comparisons).
// 4. Load the reference (known as correct) ICP transform from disk,
//    from the same directory, with file extension .ref_trans.
// 5. See if the current and reference transforms are equal.

// To update an existing test or add a new test, simply add/modify
// the desired yaml file, run the tests (they may fail this time), then
// copy the (just written) desired current transform file on top of the
// corresponding reference transform file. Run the tests again. This
// time they will succeed.
//---------------------------

TEST(icpTest, icpTest)
{
	DP ref  = DP::load(dataPath + "cloud.00000.vtk");
	DP data = DP::load(dataPath + "cloud.00001.vtk");

	namespace fs = boost::filesystem;
	fs::path config_dir(dataPath + "icp_data");
	EXPECT_TRUE( fs::exists(config_dir) && fs::is_directory(config_dir) );

	fs::directory_iterator end_iter;
	for( fs::directory_iterator d(config_dir); d != end_iter; ++d)
	{
		if (!fs::is_regular_file(d->status()) ) continue;

		// Load config file, and form ICP object
		PM::ICP icp;
		std::string config_file = d->path().string();
		if (fs::extension(config_file) != ".yaml") continue;
		std::ifstream ifs(config_file.c_str());
		EXPECT_NO_THROW(icp.loadFromYaml(ifs)) << "This error was caused by the test file:" << endl << "   " << config_file;

		// Compute current ICP transform
		PM::TransformationParameters curT = icp(data, ref);

		// Write current transform to disk (to easily compare it
		// with reference transform offline)
		fs::path cur_file = d->path();
		cur_file.replace_extension(".cur_trans");
		//std::cout << "Writing: " << cur_file << std::endl;
		std::ofstream otfs(cur_file.c_str());
		otfs.precision(16);
		otfs << curT;
		otfs.close();
                
		// Load reference transform
		fs::path ref_file = d->path();
		ref_file.replace_extension(".ref_trans");
		PM::TransformationParameters refT = 0*curT;
		//std::cout << "Reading: " << ref_file << std::endl;
		std::ifstream itfs(ref_file.c_str());
		for (int row = 0; row < refT.cols(); row++)
		{
			for (int col = 0; col < refT.cols(); col++)
			{
				itfs >>refT(row, col);
			}
		}

		// Dump the reference transform and current one
		//std::cout.precision(17);
		//std::cout << "refT:\n" << refT << std::endl;
		//std::cout << "curT:\n" << curT << std::endl;

		// Tolerance for change in rotation and translation
		double rotTol = 0.1, transTol = 0.1;

		// Find how much the reference rotation and translation
		// differ from the current values.
		PM::TransformationParameters refRot   = refT.block(0, 0, 3, 3);
		PM::TransformationParameters refTrans = refT.block(0, 3, 3, 1);
		PM::TransformationParameters curRot   = curT.block(0, 0, 3, 3);
		PM::TransformationParameters curTrans = curT.block(0, 3, 3, 1);
		PM::TransformationParameters rotErrMat = refRot*(curRot.transpose())
		  - PM::TransformationParameters::Identity(3, 3);
		PM::TransformationParameters transErrMat = refTrans - curTrans;
		double rotErr = rotErrMat.array().abs().sum();
		double transErr = transErrMat.array().abs().sum();

		//std::cout << "Rotation error:    " << rotErr   << std::endl;
		//std::cout << "Translation error: " << transErr << std::endl;
		
		EXPECT_LT(rotErr,   rotTol) << "This error was caused by the test file:" << endl << "   " << config_file;
		EXPECT_LT(transErr, transTol) << "This error was caused by the test file:" <<  endl << "   " << config_file;
	}
}

//---------------------------
// Point-cloud structures
//---------------------------

TEST(PointCloudTest, CopyConstructor2D)
{
	const DP ref2DCopy(ref2D);
	EXPECT_TRUE(ref2DCopy.features == ref2D.features);
	EXPECT_TRUE(ref2DCopy.featureLabels == ref2D.featureLabels);
	EXPECT_TRUE(ref2DCopy.descriptors == ref2D.descriptors);
	EXPECT_TRUE(ref2DCopy.descriptorLabels == ref2D.descriptorLabels);
	EXPECT_TRUE(ref2DCopy == ref2D);
}


TEST(PointCloudTest, FeatureConstructor2D)
{
	const DP ref2DCopy(ref2D.features, ref2D.featureLabels);
	EXPECT_TRUE(ref2DCopy.features == ref2D.features);
	EXPECT_TRUE(ref2DCopy.featureLabels == ref2D.featureLabels);
	EXPECT_TRUE(ref2DCopy == ref2D);
	EXPECT_TRUE(ref2DCopy.descriptors.rows() == 0);
	EXPECT_TRUE(ref2DCopy.descriptors.cols() == 0);
}

TEST(PointCloudTest, FeatureConstructor3D)
{
	// Note: this test cover also the operator ==

	////// 1-Empty constructor
	DP ref3DCopy = DP();
	EXPECT_TRUE(ref3DCopy.features.rows() == 0);
	EXPECT_TRUE(ref3DCopy.features.cols() == 0);
	EXPECT_TRUE(ref3DCopy.descriptors.rows() == 0);
	EXPECT_TRUE(ref3DCopy.descriptors.cols() == 0);


	////// 2-Constructor with only features
	ref3DCopy = DP(ref3D.features, ref3D.featureLabels);
	EXPECT_TRUE(ref3DCopy.features == ref3D.features);
	EXPECT_TRUE(ref3DCopy.featureLabels == ref3D.featureLabels);
	
	// descriptor missing in ref3DCopy
	EXPECT_FALSE(ref3DCopy == ref3D); 
	
	EXPECT_TRUE(ref3DCopy.descriptors.rows() == 0);
	EXPECT_TRUE(ref3DCopy.descriptors.cols() == 0);

	////// 3-Constructor with features and descriptors
	ref3DCopy = DP(ref3D.features, ref3D.featureLabels, ref3D.descriptors, ref3D.descriptorLabels);
	
	EXPECT_TRUE(ref3DCopy.features == ref3D.features);
	EXPECT_TRUE(ref3DCopy.featureLabels == ref3D.featureLabels);
	EXPECT_TRUE(ref3DCopy.descriptors== ref3D.descriptors);
	EXPECT_TRUE(ref3DCopy.descriptorLabels == ref3D.descriptorLabels);
	

	EXPECT_TRUE(ref3DCopy == ref3D); 
	
	////// 4-Copy constructor
	ref3DCopy = DP(ref3D);
	
	EXPECT_TRUE(ref3DCopy.features == ref3D.features);
	EXPECT_TRUE(ref3DCopy.featureLabels == ref3D.featureLabels);
	EXPECT_TRUE(ref3DCopy.descriptors== ref3D.descriptors);
	EXPECT_TRUE(ref3DCopy.descriptorLabels == ref3D.descriptorLabels);
	

	EXPECT_TRUE(ref3DCopy == ref3D);
}

TEST(PointCloudTest, ConcatenateFeatures2D)
{
	const int leftPoints(ref2D.features.cols() / 2);
	const int rightPoints(ref2D.features.cols() - leftPoints);
	DP lefts(
		ref2D.features.leftCols(leftPoints),
		ref2D.featureLabels
	);
	DP rights(
		ref2D.features.rightCols(rightPoints),
		ref2D.featureLabels
	);
	lefts.concatenate(rights);
	EXPECT_TRUE(lefts == ref2D);
}

TEST(PointCloudTest, ConcatenateFeatures3D)
{
	const int leftPoints(ref3D.features.cols() / 2);
	const int rightPoints(ref3D.features.cols() - leftPoints);
	DP lefts(
		ref3D.features.leftCols(leftPoints),
		ref3D.featureLabels
	);
	DP rights(
		ref3D.features.rightCols(rightPoints),
		ref3D.featureLabels
	);
	lefts.concatenate(rights);
	EXPECT_TRUE(lefts.features == ref3D.features);
}

TEST(PointCloudTest, ConcatenateDescSame)
{
	typedef DP::Label Label;
	typedef DP::Labels Labels;
	
	const int leftPoints(ref2D.features.cols() / 2);
	const int rightPoints(ref2D.features.cols() - leftPoints);
	DP lefts(
		ref2D.features.leftCols(leftPoints),
		ref2D.featureLabels,
		PM::Matrix::Random(5, leftPoints),
		Labels(Label("Desc5D", 5))
	);
	DP rights(
		ref2D.features.rightCols(rightPoints),
		ref2D.featureLabels,
		PM::Matrix::Random(5, rightPoints),
		Labels(Label("Desc5D", 5))
	);
	lefts.concatenate(rights);
	EXPECT_TRUE(lefts.descriptors.rows() == 5);
	EXPECT_TRUE(lefts.descriptors.cols() == lefts.features.cols());
}

TEST(PointCloudTest, ConcatenateDescSame2)
{
	typedef DP::Label Label;
	typedef DP::Labels Labels;
	
	DP ref3DCopy(ref3D.features, ref3D.featureLabels);
	ref3DCopy.descriptorLabels.push_back(Label("Desc5D", 5));
	ref3DCopy.descriptors = PM::Matrix::Random(5, ref3DCopy.features.cols());
	
	const int leftPoints(ref3DCopy.features.cols() / 2);
	const int rightPoints(ref3DCopy.features.cols() - leftPoints);
	DP lefts(
		ref3DCopy.features.leftCols(leftPoints),
		ref3DCopy.featureLabels,
		ref3DCopy.descriptors.leftCols(leftPoints),
		ref3DCopy.descriptorLabels
	);
	DP rights(
		ref3DCopy.features.rightCols(rightPoints),
		ref3DCopy.featureLabels,
		ref3DCopy.descriptors.rightCols(rightPoints),
		ref3DCopy.descriptorLabels
	);
	lefts.concatenate(rights);
	EXPECT_TRUE(lefts == ref3DCopy);
}

TEST(PointCloudTest, ConcatenateDescDiffName)
{
	typedef DP::Label Label;
	typedef DP::Labels Labels;
	
	const int leftPoints(ref2D.features.cols() / 2);
	const int rightPoints(ref2D.features.cols() - leftPoints);
	DP lefts(
		ref2D.features.leftCols(leftPoints),
		ref2D.featureLabels,
		PM::Matrix::Random(5, leftPoints),
		Labels(Label("MyDesc5D", 5))
	);
	DP rights(
		ref2D.features.rightCols(rightPoints),
		ref2D.featureLabels,
		PM::Matrix::Random(5, rightPoints),
		Labels(Label("YourDesc5D", 5))
	);
	lefts.concatenate(rights);
	EXPECT_TRUE(lefts.descriptors.rows() == 0);
	EXPECT_TRUE(lefts.descriptors.cols() == 0);
}

TEST(PointCloudTest, ConcatenateDescDiffSize)
{
	typedef DP::Label Label;
	typedef DP::Labels Labels;
	
	const int leftPoints(ref2D.features.cols() / 2);
	const int rightPoints(ref2D.features.cols() - leftPoints);
	DP lefts(
		ref2D.features.leftCols(leftPoints),
		ref2D.featureLabels,
		PM::Matrix::Random(3, leftPoints),
		Labels(Label("DescND", 3))
	);
	DP rights(
		ref2D.features.rightCols(rightPoints),
		ref2D.featureLabels,
		PM::Matrix::Random(5, rightPoints),
		Labels(Label("DescND", 5))
	);
	EXPECT_THROW(lefts.concatenate(rights), DP::InvalidField);
}

TEST(PointCloudTest, GetInfo)
{
	//cerr << ref2D.features.rows() << endl;
	//cerr << ref2D.features.cols() << endl;
	//cerr << ref2D.descriptors.rows() << endl;
	//cerr << ref2D.descriptors.cols() << endl;
	
	EXPECT_EQ(ref3D.getNbPoints(), 24989u);
	EXPECT_EQ(ref3D.getEuclideanDim(), 3u);
	EXPECT_EQ(ref3D.getHomogeneousDim(), 4u);
	EXPECT_EQ(ref3D.getNbGroupedDescriptors(), 1u);
	EXPECT_EQ(ref3D.getDescriptorDim(), 3u);
	
	EXPECT_EQ(ref2D.getNbPoints(), 361u);
	EXPECT_EQ(ref2D.getEuclideanDim(), 2u);
	EXPECT_EQ(ref2D.getHomogeneousDim(), 3u);
	EXPECT_EQ(ref2D.getNbGroupedDescriptors(), 0u);
	EXPECT_EQ(ref2D.getDescriptorDim(), 0u);

}

TEST(PointCloudTest, AddRemove)
{
	DP ref3DCopy = ref3D;
	const int testedValue = 9;

	//////Add features
	PM::Matrix newFeature = PM::Matrix::Ones(1,ref3DCopy.getNbPoints())*testedValue;
	ref3DCopy.addFeature("testF", newFeature);

	//Is the new row added?
	EXPECT_EQ(ref3DCopy.getHomogeneousDim(), ref3D.getHomogeneousDim()+1);
	
	//Is padding still at the end?
	EXPECT_EQ(ref3DCopy.featureLabels.back().text, "pad");

	//Is the value right?
	DP::View newFeatureView = ref3DCopy.getFeatureViewByName("testF");
	EXPECT_EQ(newFeatureView(0,0), testedValue);

	//////Remove features
	ref3DCopy.removeFeature("testF");

	// Is the extra data removed?
	EXPECT_TRUE(ref3DCopy.features.isApprox(ref3D.features));


	//////Add descriptors
	const int testedValue2 = 88;
	PM::Matrix newDescriptor4D = PM::Matrix::Ones(4,ref3DCopy.getNbPoints())*testedValue;
	PM::Matrix newDescriptor2D = PM::Matrix::Ones(2,ref3DCopy.getNbPoints())*testedValue2;

	ref3DCopy.addDescriptor("test4D", newDescriptor4D);
	ref3DCopy.addDescriptor("test2D", newDescriptor2D);
	
	//Is the new row added?
	EXPECT_EQ(ref3DCopy.getDescriptorDim(), ref3D.getDescriptorDim()+6);
	EXPECT_EQ(ref3DCopy.getNbGroupedDescriptors(), ref3D.getNbGroupedDescriptors()+2);

	//Is the value right?
	DP::View newDescriptor4DView = ref3DCopy.getDescriptorViewByName("test4D");
	EXPECT_EQ(newDescriptor4DView(0,0), testedValue);
	DP::View newDescriptor2DView = ref3DCopy.getDescriptorViewByName("test2D");
	EXPECT_EQ(newDescriptor2DView(0,0), testedValue2);


	//////Remove descriptors
	ref3DCopy.removeDescriptor("test4D");
	ref3DCopy.removeDescriptor("test2D");
	
	//removing random name shoudn't have any effect
	ref3DCopy.removeDescriptor("grrrrr");

	// Is the extra data removed?
	EXPECT_TRUE(ref3DCopy.descriptors.isApprox(ref3D.descriptors));

}

//---------------------------
// Tests for IO
//---------------------------

TEST(IOTest, loadYaml)
{

	// Test loading configuration files for data filters
	std::ifstream ifs0((dataPath + "default-convert.yaml").c_str());
	EXPECT_NO_THROW(PM::DataPointsFilters filters(ifs0));

	// Test loading configuration files for ICP
	PM::ICP icp;

	std::ifstream ifs1((dataPath + "default.yaml").c_str());
	EXPECT_NO_THROW(icp.loadFromYaml(ifs1));

	std::ifstream ifs2((dataPath + "unit_tests/badIcpConfig_InvalidParameter.yaml").c_str());
	EXPECT_THROW(icp.loadFromYaml(ifs2), PointMatcherSupport::Parametrizable::InvalidParameter);
	
	std::ifstream ifs3((dataPath + "unit_tests/badIcpConfig_InvalidModuleType.yaml").c_str());
	EXPECT_THROW(icp.loadFromYaml(ifs3), PointMatcherSupport::InvalidModuleType);
}

TEST(IOTest, loadPLY)
{
	typedef PointMatcherIO<float> IO;
	std::istringstream is;
	
	is.str(
	""
	);

	EXPECT_THROW(IO::loadPLY(is), runtime_error);

	is.clear();
	is.str(
	"ply\n"
	"format binary_big_endian 1.0\n"
	);

	EXPECT_THROW(IO::loadPLY(is), runtime_error);
	
	is.clear();
	is.str(
	"ply\n"
	"format ascii 2.0\n"
	);
	
	EXPECT_THROW(IO::loadPLY(is), runtime_error);

	is.clear();
	is.str(
	"ply\n"
	"format ascii 1.0\n"
	);
	
	EXPECT_THROW(IO::loadPLY(is), runtime_error);

	is.clear();
	is.str(
	"ply\n"
	"format ascii 1.0\n"
	"element vertex 5\n"
	"\n" //empty line
	"property float z\n" // wrong order
	"property float y\n"
	"property float x\n"
	"property float grrrr\n" //unknown property
	"property float nz\n" // wrong order
	"property float ny\n"
	"property float nx\n"
	"end_header\n"
	"3 2 1 99 33 22 11\n"
	"3 2 1 99 33 22 11\n"
	"\n" //empty line
	"3 2 1 99 33 22 11 3 2 1 99 33 22 11\n" // no line break
	"3 2 1 99 33 22 11\n"

	);
	
	DP pointCloud = IO::loadPLY(is);
	
	// Confirm sizes and dimensions
	EXPECT_TRUE(pointCloud.features.cols() == 5);
	EXPECT_TRUE(pointCloud.features.rows() == 4);
	EXPECT_TRUE(pointCloud.descriptors.cols() == 5);
	EXPECT_TRUE(pointCloud.descriptors.rows() == 3);
	
	// Random value check
	EXPECT_TRUE(pointCloud.features(0, 0) == 1);
	EXPECT_TRUE(pointCloud.features(2, 2) == 3);
	EXPECT_TRUE(pointCloud.descriptors(1, 1) == 22);
	EXPECT_TRUE(pointCloud.descriptors(2, 4) == 33);

}


class IOLoadSaveTest : public testing::Test
{

public:
	virtual void SetUp()
	{
		nbPts = 10;
		addRandomFeature("x", 1);
		addRandomFeature("y", 1);
		addRandomFeature("z", 1);
		ptCloud.addFeature("pad", PM::Matrix::Ones(1, nbPts));

		addRandomDescriptor("normals",3);
		addRandomDescriptor("eigValues",3);
		addRandomDescriptor("eigVectors",9);
		addRandomDescriptor("color",4);
	}

	void addRandomFeature(const string& featureName, const int rows)
	{
		ptCloud.addFeature(featureName,PM::Matrix::Random(rows, nbPts));
	}

	void addRandomDescriptor(const string& descriptorName, const int rows)
	{
		ptCloud.addDescriptor(descriptorName, PM::Matrix::Random(rows, nbPts));
	}

	virtual void loadSaveTest(const string& testFileName, const int nbPts = 10)
	{
		this->testFileName = testFileName;
		ptCloud.save(testFileName);

		ptCloudFromFile = DP::load(testFileName);

		EXPECT_TRUE(ptCloudFromFile.features.cols() == ptCloud.features.cols());
		EXPECT_TRUE(ptCloudFromFile.descriptorExists("normals",3));
		EXPECT_TRUE(ptCloudFromFile.getDescriptorViewByName("normals").isApprox(ptCloud.getDescriptorViewByName("normals")));
		EXPECT_TRUE(ptCloudFromFile.descriptorExists("eigValues",3));
		EXPECT_TRUE(ptCloudFromFile.getDescriptorViewByName("eigValues").isApprox(ptCloud.getDescriptorViewByName("eigValues")));
		EXPECT_TRUE(ptCloudFromFile.descriptorExists("eigVectors",9));
		EXPECT_TRUE(ptCloudFromFile.getDescriptorViewByName("eigVectors").isApprox(ptCloud.getDescriptorViewByName("eigVectors")));
		EXPECT_TRUE(ptCloudFromFile.descriptorExists("color",4));
		EXPECT_TRUE(ptCloudFromFile.getDescriptorViewByName("color").isApprox(ptCloud.getDescriptorViewByName("color")));

		EXPECT_TRUE(ptCloudFromFile.features.isApprox(ptCloud.features));

	}

	virtual void TearDown()
	{
		EXPECT_TRUE(boost::filesystem::remove(boost::filesystem::path(testFileName)));
	}


protected:
	int nbPts;
	DP::Labels featureLabels;
	DP ptCloud;
	DP ptCloudFromFile;
	string testFileName;

};

TEST_F(IOLoadSaveTest, VTK)
{
	ptCloud.addDescriptor("genericScalar", PM::Matrix::Random(1, nbPts));
	ptCloud.addDescriptor("genericVector", PM::Matrix::Random(3, nbPts));

	loadSaveTest("unit_test.vtk");

	EXPECT_TRUE(ptCloudFromFile.descriptorExists("genericScalar",1));
	EXPECT_TRUE(ptCloudFromFile.descriptorExists("genericVector",3));

}

TEST_F(IOLoadSaveTest, PLY)
{
	loadSaveTest("unit_test.ply");
}

TEST_F(IOLoadSaveTest, PCD)
{
	loadSaveTest("unit_test.pcd");
}

TEST_F(IOLoadSaveTest, CSV)
{
	loadSaveTest("unit_test.csv");
}


//---------------------------
// Base for ICP tests
//---------------------------

class IcpHelper: public testing::Test
{
public:
	
	PM::ICP icp;
	
	PM::Parameters params;

	virtual void dumpVTK()
	{
		// Make available a VTK inspector for manual inspection
		icp.inspector.reset(
			PM::get().InspectorRegistrar.create(
				"VTKFileInspector", 
				boost::assign::map_list_of
					("baseFileName","./unitTest")
			)
		);
	}
	
	void validate2dTransformation()
	{
		const PM::TransformationParameters testT = icp(data2D, ref2D);
		const int dim = validT2d.cols();

		const BOOST_AUTO(validTrans, validT2d.block(0, dim-1, dim-1, 1).norm());
		const BOOST_AUTO(testTrans, testT.block(0, dim-1, dim-1, 1).norm());
	
		const BOOST_AUTO(validAngle, acos(validT2d(0,0)));
		const BOOST_AUTO(testAngle, acos(testT(0,0)));
		
		EXPECT_NEAR(validTrans, testTrans, 0.05);
		EXPECT_NEAR(validAngle, testAngle, 0.05);
	}

	void validate3dTransformation()
	{
		//dumpVTK();

		const PM::TransformationParameters testT = icp(data3D, ref3D);
		const int dim = validT2d.cols();

		const BOOST_AUTO(validTrans, validT3d.block(0, dim-1, dim-1, 1).norm());
		const BOOST_AUTO(testTrans, testT.block(0, dim-1, dim-1, 1).norm());
	
		const BOOST_AUTO(testRotation, Eigen::Quaternion<float>(Eigen::Matrix<float,3,3>(testT.topLeftCorner(3,3))));
		const BOOST_AUTO(validRotation, Eigen::Quaternion<float>(Eigen::Matrix<float,3,3>(validT3d.topLeftCorner(3,3))));
		
		const BOOST_AUTO(angleDist, validRotation.angularDistance(testRotation));
		
		//cout << testT << endl;
		//cout << "angleDist: " << angleDist << endl;
		//cout << "transDist: " << abs(validTrans-testTrans) << endl;
		EXPECT_NEAR(validTrans, testTrans, 0.1);
		EXPECT_NEAR(angleDist, 0.0, 0.1);

	}
};

// Utility classes
class GenericTest: public IcpHelper
{

public:

	// Will be called for every tests
	virtual void SetUp()
	{
		icp.setDefault();
		// Uncomment for consol outputs
		//setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
	}

	// Will be called for every tests
	virtual void TearDown()
	{	
	}
};


//---------------------------
// Generic tests
//---------------------------

TEST_F(GenericTest, ICP_default)
{
	validate2dTransformation();
	validate3dTransformation();
}

//---------------------------
// DataFilter modules
//---------------------------

// Utility classes
class DataFilterTest: public IcpHelper
{
public:
	// Will be called for every tests
	virtual void SetUp()
	{
		icp.setDefault();
		// Uncomment for console outputs
		//setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
		
		// We'll test the filters on reading point cloud
		icp.readingDataPointsFilters.clear();
	}

	// Will be called for every tests
	virtual void TearDown()	{}

	void addFilter(string name, PM::Parameters params)
	{
		PM::DataPointsFilter* testedDataPointFilter = 
			PM::get().DataPointsFilterRegistrar.create(name, params);
	
		icp.readingDataPointsFilters.push_back(testedDataPointFilter);
	}
	
	void addFilter(string name)
	{
		PM::DataPointsFilter* testedDataPointFilter = 
			PM::get().DataPointsFilterRegistrar.create(name);
		
		icp.readingDataPointsFilters.push_back(testedDataPointFilter);
	}
};


TEST_F(DataFilterTest, RemoveNaNDataPointsFilter)
{
	// build test cloud
	DP ref2DCopy(ref2D);
	int goodCount(0);
	const float nan(std::numeric_limits<float>::quiet_NaN());
	for (int i(0); i < ref2DCopy.features.cols(); ++i)
	{
		if (rand() % 3 == 0)
		{
			ref2DCopy.features(rand() % ref2DCopy.features.rows(), i) = nan;
		}
		else
			++goodCount;
	}
	
	// apply and checked
	addFilter("RemoveNaNDataPointsFilter");
	icp.readingDataPointsFilters.apply(ref2DCopy);
	EXPECT_TRUE(ref2DCopy.features.cols() == goodCount);
}

TEST_F(DataFilterTest, MaxDistDataPointsFilter)
{
	// Max dist has been selected to not affect the points
	params = map_list_of<string,string>
		("dim","0")
		("maxDist", toParam(6.0))
	;
	
	// Filter on x axis
	params["dim"] = "0";
	icp.readingDataPointsFilters.clear();
	addFilter("MaxDistDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
	
	// Filter on y axis
	params["dim"] = "1";
	icp.readingDataPointsFilters.clear();
	addFilter("MaxDistDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
	
	// Filter on z axis (not existing)
	params["dim"] = "2";
	icp.readingDataPointsFilters.clear();
	addFilter("MaxDistDataPointsFilter", params);
	EXPECT_ANY_THROW(validate2dTransformation());
	validate3dTransformation();
	
	// Filter on a radius
	params["dim"] = "-1";
	icp.readingDataPointsFilters.clear();
	addFilter("MaxDistDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
	
	// Parameter outside valid range
	params["dim"] = "3";
	//TODO: specify the exception, move that to GenericTest
	EXPECT_ANY_THROW(addFilter("MaxDistDataPointsFilter", params));
	
}

TEST_F(DataFilterTest, MinDistDataPointsFilter)
{
	// Min dist has been selected to not affect the points too much
	params = map_list_of<string,string>
		("dim","0")
		("minDist", toParam(0.05))
	;
	
	// Filter on x axis
	params["dim"] = "0";
	icp.readingDataPointsFilters.clear();
	addFilter("MinDistDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
	
	// Filter on y axis
	params["dim"] = "1";
	icp.readingDataPointsFilters.clear();
	addFilter("MinDistDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
	
	//TODO: move that to specific 2D test
	// Filter on z axis (not existing)
	params["dim"] = "2";
	icp.readingDataPointsFilters.clear();
	addFilter("MinDistDataPointsFilter", params);
	EXPECT_ANY_THROW(validate2dTransformation());
	validate3dTransformation();
	
	// Filter on a radius
	params["dim"] = "-1";
	icp.readingDataPointsFilters.clear();
	addFilter("MinDistDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
		
}

TEST_F(DataFilterTest, MaxQuantileOnAxisDataPointsFilter)
{
	// Ratio has been selected to not affect the points too much
	string ratio = "0.95";
	params = map_list_of<string,string>
		("dim","0")
		("ratio", ratio)
	;
	
	// Filter on x axis
	params["dim"] = "0";
	icp.readingDataPointsFilters.clear();
	addFilter("MaxQuantileOnAxisDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
	
	// Filter on y axis
	params["dim"] = "1";
	icp.readingDataPointsFilters.clear();
	addFilter("MaxQuantileOnAxisDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
	
	// Filter on z axis (not existing)
	params["dim"] = "2";
	icp.readingDataPointsFilters.clear();
	addFilter("MaxQuantileOnAxisDataPointsFilter", params);
	EXPECT_ANY_THROW(validate2dTransformation());
	validate3dTransformation();
}



TEST_F(DataFilterTest, SurfaceNormalDataPointsFilter)
{
	// This filter create descriptor, so parameters should'nt impact results
	params = map_list_of
		("knn", "5") 
		("epsilon", "0.1") 
		("keepNormals", "1")
		("keepDensities", "1")
		("keepEigenValues", "1")
		("keepEigenVectors", "1" )
		("keepMatchedIds" , "1" )
	;
	// FIXME: the parameter keepMatchedIds seems to do nothing...

	addFilter("SurfaceNormalDataPointsFilter", params);
	validate2dTransformation();	
	validate3dTransformation();

	// TODO: standardize how filter are tested:
	// 1- impact on number of points
	// 2- impact on descriptors
	// 3- impact on ICP (that's what we test now)
}

TEST_F(DataFilterTest, MaxDensityDataPointsFilter)
{
	// Ratio has been selected to not affect the points too much
 	vector<double> ratio = list_of (100) (1000) (5000);
 
 	for(unsigned i=0; i < ratio.size(); i++)
 	{
 		icp.readingDataPointsFilters.clear();
		params = map_list_of
			("knn", "5") 
			("epsilon", "0.1") 
			("keepNormals", "0")
			("keepDensities", "1")
			("keepEigenValues", "0")
			("keepEigenVectors", "0" )
			("keepMatchedIds" , "0" )
		;

		addFilter("SurfaceNormalDataPointsFilter", params);

 		params = map_list_of ("maxDensity", toParam(ratio[i]));
 		addFilter("MaxDensityDataPointsFilter", params);
 		
		// FIXME BUG: the density in 2D is not well computed
		//validate2dTransformation();	
 
 		//double nbInitPts = data2D.features.cols();
 		//double nbRemainingPts = icp.getPrefilteredReadingPtsCount();
 		//EXPECT_TRUE(nbRemainingPts < nbInitPts);
 		
 		validate3dTransformation();

		double nbInitPts = data3D.features.cols();
 		double nbRemainingPts = icp.getPrefilteredReadingPtsCount();
 		EXPECT_TRUE(nbRemainingPts < nbInitPts);
 	}
}

TEST_F(DataFilterTest, SamplingSurfaceNormalDataPointsFilter)
{
	// This filter create descriptor AND subsample
	params = map_list_of
		("knn", "5")
		("averageExistingDescriptors", "1")
		("keepNormals", "1")
		("keepDensities", "1")
		("keepEigenValues", "1")
		("keepEigenVectors", "1")
	;
	
	addFilter("SamplingSurfaceNormalDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();

}

TEST_F(DataFilterTest, OrientNormalsDataPointsFilter)
{
	// Used to create normal for reading point cloud
	PM::DataPointsFilter* extraDataPointFilter;
	extraDataPointFilter = PM::get().DataPointsFilterRegistrar.create(
			"SurfaceNormalDataPointsFilter");
	icp.readingDataPointsFilters.push_back(extraDataPointFilter);
	addFilter("ObservationDirectionDataPointsFilter");
	addFilter("OrientNormalsDataPointsFilter", map_list_of
		("towardCenter", toParam(false))
	);
	validate2dTransformation();
	validate3dTransformation();
}


TEST_F(DataFilterTest, RandomSamplingDataPointsFilter)
{
	vector<double> prob = list_of (0.80) (0.85) (0.90) (0.95);
	for(unsigned i=0; i<prob.size(); i++)
	{
		// Try to avoid to low value for the reduction to avoid under sampling
		params = map_list_of
			("prob", toParam(prob[i]))
		;
		icp.readingDataPointsFilters.clear();
		addFilter("RandomSamplingDataPointsFilter", params);
		validate2dTransformation();
		validate3dTransformation();
	}
}

TEST_F(DataFilterTest, FixStepSamplingDataPointsFilter)
{
	vector<unsigned> steps = list_of (1) (2) (3);
	for(unsigned i=0; i<steps.size(); i++)
	{
		// Try to avoid too low value for the reduction to avoid under sampling
		params = map_list_of
			("startStep", toParam(steps[i]))
		;
		icp.readingDataPointsFilters.clear();
		addFilter("FixStepSamplingDataPointsFilter", params);
		validate2dTransformation();
		validate3dTransformation();
	}
}

TEST_F(DataFilterTest, VoxelGridDataPointsFilter)
{
	vector<bool> useCentroid = list_of(false)(true);
	vector<bool> averageExistingDescriptors = list_of(false)(true);
	for (unsigned i = 0 ; i < useCentroid.size() ; i++) 
	{
		for (unsigned j = 0; j < averageExistingDescriptors.size(); j++) 
		{
			params = map_list_of<string,string>
					("vSizeX","0.02")
					("vSizeY","0.02")
					("vSizeZ","0.02")
					("useCentroid",toParam(true))
					("averageExistingDescriptors",toParam(true))
			;
			icp.readingDataPointsFilters.clear();
			addFilter("VoxelGridDataPointsFilter", params);
			validate2dTransformation();
		}
	}

	for (unsigned i = 0 ; i < useCentroid.size() ; i++)
	{
		for (unsigned j = 0; j < averageExistingDescriptors.size(); j++)
		{
			params = map_list_of<string,string>
			("vSizeX","1")
			("vSizeY","1")
			("vSizeZ","1")
			("useCentroid",toParam(true))
			("averageExistingDescriptors",toParam(true));
			icp.readingDataPointsFilters.clear();
			addFilter("VoxelGridDataPointsFilter", params);
			validate3dTransformation();
		}
	}
}

//---------------------------
// Matcher modules
//---------------------------

// Utility classes
class MatcherTest: public IcpHelper
{

public:

	PM::Matcher* testedMatcher;

	// Will be called for every tests
	virtual void SetUp()
	{
		icp.setDefault();
		// Uncomment for consol outputs
		//setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
	}

	// Will be called for every tests
	virtual void TearDown(){}

	void addFilter(string name, PM::Parameters params)
	{
		testedMatcher = 
			PM::get().MatcherRegistrar.create(name, params);
		icp.matcher.reset(testedMatcher);
	}

};

TEST_F(MatcherTest, KDTreeMatcher)
{
	vector<unsigned> knn = list_of (1) (2) (3);
	vector<double> epsilon = list_of (0.0) (0.2);
	vector<double> maxDist = list_of (1.0) (0.5);

	for(unsigned i=0; i < knn.size(); i++)
	{
		for(unsigned j=0; j < epsilon.size(); j++)
		{
			for(unsigned k=0; k < maxDist.size(); k++)
			{
				params = map_list_of
					("knn", toParam(knn[i])) // remove end parenthesis for bug
					("epsilon", toParam(epsilon[j]))
					("searchType", "1")
					("maxDist", toParam(maxDist[k]))
				;
			
				addFilter("KDTreeMatcher", params);
				validate2dTransformation();
				validate3dTransformation();
			}
		}
	}
}


//---------------------------
// Outlier modules
//---------------------------

// Utility classes
class OutlierFilterTest: public IcpHelper
{
public:
	PM::OutlierFilter* testedOutlierFilter;

	// Will be called for every tests
	virtual void SetUp()
	{
		icp.setDefault();
		// Uncomment for consol outputs
		//setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
		
		icp.outlierFilters.clear();
	}

	// Will be called for every tests
	virtual void TearDown(){}

	void addFilter(string name, PM::Parameters params)
	{
		testedOutlierFilter = 
			PM::get().OutlierFilterRegistrar.create(name, params);
		icp.outlierFilters.push_back(testedOutlierFilter);
	}

};


//No commun parameters were found for 2D and 3D, tests are splited
TEST_F(OutlierFilterTest, MaxDistOutlierFilter2D)
{
	addFilter("MaxDistOutlierFilter", map_list_of
		("maxDist", toParam(0.015))
	);
	validate2dTransformation();
}

TEST_F(OutlierFilterTest, MaxDistOutlierFilter3D)
{
	addFilter("MaxDistOutlierFilter", map_list_of
		("maxDist", toParam(0.1))
	);
	validate3dTransformation();
}

//No commun parameters were found for 2D and 3D, tests are splited
TEST_F(OutlierFilterTest, MinDistOutlierFilter2D)
{
	// Since not sure how useful is that filter, we keep the 
	// MaxDistOutlierFilter with it
	PM::OutlierFilter* extraOutlierFilter;
	
	extraOutlierFilter = 
		PM::get().OutlierFilterRegistrar.create(
			"MaxDistOutlierFilter", map_list_of 
				("maxDist", toParam(0.015))
		)
	;
	icp.outlierFilters.push_back(extraOutlierFilter);
	
	addFilter("MinDistOutlierFilter", map_list_of ("minDist", toParam(0.0002)) );
	
	validate2dTransformation();
}

TEST_F(OutlierFilterTest, MinDistOutlierFilter3D)
{
	// Since not sure how useful is that filter, we keep the 
	// MaxDistOutlierFilter with it
	PM::OutlierFilter* extraOutlierFilter;
	
	extraOutlierFilter = 
		PM::get().OutlierFilterRegistrar.create(
			"MaxDistOutlierFilter", map_list_of 
				("maxDist", toParam(0.1))
		)
	;
	icp.outlierFilters.push_back(extraOutlierFilter);
	
	addFilter("MinDistOutlierFilter", map_list_of ("minDist", toParam(0.0002)) );
	
	validate3dTransformation();
}

TEST_F(OutlierFilterTest, MedianDistOutlierFilter)
{
	addFilter("MedianDistOutlierFilter", map_list_of ("factor", toParam(3.5)));
	validate2dTransformation();
	validate3dTransformation();
}


TEST_F(OutlierFilterTest, TrimmedDistOutlierFilter)
{
	addFilter("TrimmedDistOutlierFilter", map_list_of ("ratio", toParam(0.85)) );
	validate2dTransformation();
	validate3dTransformation();
}


TEST_F(OutlierFilterTest, VarTrimmedDistOutlierFilter)
{
	addFilter("VarTrimmedDistOutlierFilter", map_list_of
		("minRatio", toParam(0.60))
		("maxRatio", toParam(0.80))
		("lambda", toParam(0.9))
	);
	validate2dTransformation();
	validate3dTransformation();
}

//---------------------------
// Error modules
//---------------------------

// Utility classes
class ErrorMinimizerTest: public IcpHelper
{
public:
	PM::ErrorMinimizer* errorMin;

	// Will be called for every tests
	virtual void SetUp()
	{
		icp.setDefault();
		// Uncomment for consol outputs
		//setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
	}

	// Will be called for every tests
	virtual void TearDown(){}

	void addFilter(string name)
	{
		errorMin = PM::get().ErrorMinimizerRegistrar.create(name);
		icp.errorMinimizer.reset(errorMin);
	}
};


TEST_F(ErrorMinimizerTest, PointToPointErrorMinimizer)
{
	addFilter("PointToPointErrorMinimizer");	
	validate2dTransformation();
	validate3dTransformation();
}

TEST_F(ErrorMinimizerTest, PointToPlaneErrorMinimizer)
{
	addFilter("PointToPlaneErrorMinimizer");	
	validate2dTransformation();
	validate3dTransformation();
}

//---------------------------
// Transformation Checker modules
//---------------------------

// Utility classes
class TransformationCheckerTest: public IcpHelper
{
public:
	PM::TransformationChecker* transformCheck;

	// Will be called for every tests
	virtual void SetUp()
	{
		icp.setDefault();
		// Uncomment for consol outputs
		//setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
		
		icp.transformationCheckers.clear();
	}

	// Will be called for every tests
	virtual void TearDown(){}

	void addFilter(string name, PM::Parameters params)
	{
		transformCheck = 
			PM::get().TransformationCheckerRegistrar.create(name, params);
		
		icp.transformationCheckers.push_back(transformCheck);
	}
};


TEST_F(TransformationCheckerTest, CounterTransformationChecker)
{
	addFilter("CounterTransformationChecker", map_list_of ("maxIterationCount", toParam(20)) );
	validate2dTransformation();
}

TEST_F(TransformationCheckerTest, DifferentialTransformationChecker)
{
	addFilter("DifferentialTransformationChecker", map_list_of
		("minDiffRotErr", toParam(0.001))
		("minDiffTransErr", toParam(0.001))
		("smoothLength", toParam(4))
	);
	validate2dTransformation();
}

TEST_F(TransformationCheckerTest, BoundTransformationChecker)
{
	// Since that transChecker is trigger when the distance is growing
	// and that we do not expect that to happen in the test dataset, we
	// keep the Counter to get out of the looop	
	PM::TransformationChecker* extraTransformCheck;
	
	extraTransformCheck = PM::get().TransformationCheckerRegistrar.create(
		"CounterTransformationChecker"
	);
	icp.transformationCheckers.push_back(extraTransformCheck);
	
	addFilter("BoundTransformationChecker", map_list_of
		("maxRotationNorm", toParam(1.0))
		("maxTranslationNorm", toParam(1.0))
	);
	validate2dTransformation();
}

//---------------------------
// Transformation
//---------------------------
TEST(Transformation, RigidTransformation)
{
	PM::Transformation* rigidTrans;
	rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

	//-------------------------------------
	// Construct a 3D non-orthogonal matrix
	PM::Matrix T_3D = PM::Matrix::Identity(4,4);
	T_3D(0,0) = 2.3;
	T_3D(0,1) = 0.03;

	EXPECT_FALSE(rigidTrans->checkParameters(T_3D));

	EXPECT_THROW(rigidTrans->compute(data3D, T_3D), TransformationError);

	// Check stability over iterations
	for(int i = 0; i < 10; i++)
	{
		T_3D = rigidTrans->correctParameters(T_3D);
		EXPECT_TRUE(rigidTrans->checkParameters(T_3D));
	}

	//-------------------------------------
	// Construct a 2D non-orthogonal matrix
	PM::Matrix T_2D = PM::Matrix::Identity(3,3);
	T_2D(1,0) = 8.99;
	T_2D(0,1) = 4.03;

	EXPECT_FALSE(rigidTrans->checkParameters(T_2D));

	EXPECT_THROW(rigidTrans->compute(data2D, T_2D), TransformationError);

	// Check stability over iterations
	for(int i = 0; i < 10; i++)
	{
		T_2D = rigidTrans->correctParameters(T_2D);
		EXPECT_TRUE(rigidTrans->checkParameters(T_2D));
	}

}

//---------------------------
// Inspectors
//---------------------------
TEST(Inspectors, PerformanceInspector)
{
	PM::Inspector* performances =
		PM::get().REG(Inspector).create(
			"PerformanceInspector", map_list_of
				("baseFileName", "/tmp/utest_performances")
				("dumpPerfOnExit", "1")
		)
	;

	//TODO: we only test constructor here, check other things...
}

TEST(Inspectors, VTKFileInspector)
{
	PM::Inspector* vtkFile = 
		PM::get().REG(Inspector).create(
			"VTKFileInspector", map_list_of
				("baseFileName", "/tmp/utest_vtk")
				("dumpPerfOnExit", "1")
		)
	;
	//TODO: we only test constructor here, check other things...
}

//---------------------------
// Loggers
//---------------------------

//TODO: FileLogger
//Log using std::stream.
//- infoFileName (default: /dev/stdout) - name of the file to output infos to
//- warningFileName (default: /dev/stderr) - name of the file to output warnings to
//- displayLocation (default: 0) - display the location of message in source code
TEST(Loggers, FileLogger)
{
	string infoFileName = "utest_info";
	string warningFileName = "utest_warn";

	Logger* fileLog = 
		PM::get().REG(Logger).create(
			"FileLogger", map_list_of
				("infoFileName", infoFileName)
				("warningFileName", warningFileName)
				("displayLocation", "1")
		)
	;
	//TODO: we only test constructor here, check other things...

	delete fileLog;

	// Remove file from disk
	EXPECT_TRUE(boost::filesystem::remove(boost::filesystem::path(infoFileName)));
	EXPECT_TRUE(boost::filesystem::remove(boost::filesystem::path(warningFileName)));
}


//---------------------------
// Main
//---------------------------
int main(int argc, char **argv)
{
	dataPath = "";
	for(int i=1; i < argc; i++)
	{
		if (strcmp(argv[i], "--path") == 0 && i+1 < argc)
			dataPath = argv[i+1];
	}

	if(dataPath == "")
	{
		cerr << "Missing the flag --path ./path/to/examples/data\n Please give the path to the test data folder which should be included with the source code. The folder is named 'examples/data'." << endl;
		return -1;
	}

	// Load point cloud for all test
	ref2D =  DP::load(dataPath + "2D_oneBox.csv");
	data2D = DP::load(dataPath + "2D_twoBoxes.csv");
	ref3D =  DP::load(dataPath + "car_cloud400.csv");
	data3D = DP::load(dataPath + "car_cloud401.csv");

	// Result of data express in ref (from visual inspection)
	validT2d = PM::TransformationParameters(3,3);
	validT2d <<  0.987498,  0.157629, 0.0859918,
				-0.157629,  0.987498,  0.203247,
						0,         0,         1;

	validT3d = PM::TransformationParameters(4,4);
	validT3d <<   0.982304,   0.166685,  -0.0854066,  0.0446816,
	 			 -0.150189,   0.973488,   0.172524,   0.191998,
	   			  0.111899,  -0.156644,   0.981296,  -0.0356313,
	              0,          0,          0,          1;

	testing::GTEST_FLAG(print_time) = true;
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}




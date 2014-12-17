#include "../utest.h"

using namespace std;
using namespace PointMatcherSupport;

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

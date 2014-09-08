#include "../utest.h"

using namespace std;
using namespace PointMatcherSupport;


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

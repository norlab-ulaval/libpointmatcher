#include "../utest.h"

using namespace std;
using namespace PointMatcherSupport;

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

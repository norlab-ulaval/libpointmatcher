#include "../utest.h"

using namespace std;
using namespace PointMatcherSupport;

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

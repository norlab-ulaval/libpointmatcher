#include "../utest.h"

using namespace std;
using namespace PointMatcherSupport;

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

#include "../utest.h"

using namespace std;
using namespace PointMatcherSupport;

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

TEST_F(DataFilterTest, CutAtDescriptorThresholdDataPointsFilter)
{
	// Copied from density ratio above
	vector<double> thresholds = list_of (100) (1000) (5000);

	DP ref3Ddensities = ref3D;
	// Adding descriptor "densities"
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
	icp.readingDataPointsFilters.apply(ref3Ddensities);

	for(unsigned i=0; i < thresholds.size(); i++)
	{
		int belowCount=0;
		int aboveCount=0;

		// counting points above and below
		PM::DataPoints::View densities = ref3Ddensities.getDescriptorViewByName("densities");
		for (unsigned j=0; j < densities.cols(); ++j)
		{
			if (densities(0, j) <= thresholds[i])
			{
				++belowCount;
			}
			if (densities(0, j) >= thresholds[i])
			{
				++aboveCount;
			}
		}

		for(bool useLargerThan(true); useLargerThan; useLargerThan=false)
		{
			DP ref3DCopy = ref3Ddensities;

			icp.readingDataPointsFilters.clear();
			params = map_list_of
				("descName", toParam("densities"))
				("useLargerThan", toParam(useLargerThan))
				("threshold", toParam(thresholds[i]))
			;

			addFilter("CutAtDescriptorThresholdDataPointsFilter", params);
			icp.readingDataPointsFilters.apply(ref3DCopy);
			if (useLargerThan)
			{
				EXPECT_TRUE(ref3DCopy.features.cols() == belowCount);
			}
			else
			{
				EXPECT_TRUE(ref3DCopy.features.cols() == aboveCount);
			}
		}
	}
}


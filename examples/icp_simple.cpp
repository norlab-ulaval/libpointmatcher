/*
 * filterProfiler.cpp
 *
 *  Created on: Feb 27, 2014
 *      Author: sam
 */

#include <iostream>
#include <pointmatcher/PointMatcher.h>
#include <boost/assign.hpp>
#include <ctime>
#include <time.h>

using namespace PointMatcherSupport;
using namespace std;
using namespace boost;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

int main(int argc, char *argv[])
{
	std::string cloud_filename = "/home/nicolas-lauzon/libs-norlab/libpointmatcher/examples/data/cloud.00000.vtk";
	DP in((DP::load(cloud_filename)));

	std::shared_ptr<PM::DataPointsFilter> randomSample =
		PM::get().DataPointsFilterRegistrar.create(
				"RandomSamplingDataPointsFilter",
				{{"prob", toParam(0.5)}}
		);

	cout << "starting random sample filter" << endl;
	clock_t time_a = clock();
	randomSample->inPlaceFilter(in);
	clock_t time_b = clock();

	if (time_a == ((clock_t)-1) || time_b == ((clock_t)-1))
	{
		perror("Unable to calculate elapsed time");
		return -1;
	}
	else
	{
		cout << "Performed random sampling in " << (float)(time_b - time_a)/CLOCKS_PER_SEC << " seconds" << endl;
	}

	std::shared_ptr<PM::DataPointsFilter> voxelhashf =
		PM::get().DataPointsFilterRegistrar.create(
				"VoxelHashMapDataPointsFilter",
				{
					{"voxelSize", toParam(1.0)},
					{"pointsPerVoxel", toParam(1)},
				}
		);

	cout << "starting voxel hash map sample filter" << endl;
	time_a = clock();
	voxelhashf->inPlaceFilter(in);
	time_b = clock();

	if (time_a == ((clock_t)-1) || time_b == ((clock_t)-1))
	{
		perror("Unable to calculate elapsed time");
		return -1;
	}
	else
	{
		cout << "Performed voxel grid sampling in " << (float)(time_b - time_a)/CLOCKS_PER_SEC << " seconds" << endl;
	}

	return 0;
}



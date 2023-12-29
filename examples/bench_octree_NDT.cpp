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

int main()
{
		DP in(DP::load("apartment.vtk"));

		std::shared_ptr<PM::DataPointsFilter> randomSample =
			PM::get().DataPointsFilterRegistrar.create(
					"OctreeGridDataPointsFilter",
					{{"maxSizeByNode", toParam(0.03182409371060671)},
                     {"samplingMethod", toParam(4)}}
			);

		cout << "starting octree sample filter" << endl;
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
		    cout << "Performed octree sampling in " << (float)(time_b - time_a)/CLOCKS_PER_SEC << " seconds" << endl;
		}

		return 0;
}



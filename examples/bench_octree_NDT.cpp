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

        auto paramsOmega = PM::Parameters();
        paramsOmega["descriptorName"] = toParam("omega");
        paramsOmega["descriptorDimension"] = toParam(1);
        paramsOmega["descriptorValues"] = toParam("[1]");

        std::shared_ptr<PM::DataPointsFilter> filterOmega = PM::get().DataPointsFilterRegistrar.create("AddDescriptorDataPointsFilter", paramsOmega);
        filterOmega->inPlaceFilter(in);

        auto paramsDeviation = PM::Parameters();
        paramsDeviation["descriptorName"] = toParam("deviation");
        paramsDeviation["descriptorDimension"] = toParam(9);
        paramsDeviation["descriptorValues"] = toParam("[0.0009, 0, 0, 0, 0.0009, 0, 0, 0, 0.0009]");

        std::shared_ptr<PM::DataPointsFilter> filterDeviation = PM::get().DataPointsFilterRegistrar.create("AddDescriptorDataPointsFilter", paramsDeviation);
        filterDeviation->inPlaceFilter(in);

		std::shared_ptr<PM::DataPointsFilter> octreeFilter =
			PM::get().DataPointsFilterRegistrar.create(
					"OctreeGridDataPointsFilter",
					{
//                        {"maxSizeByNode", toParam(0.1)},
                        {"maxSizeByNode", toParam(0.03182409371060671)},
                     {"samplingMethod", toParam(4)},
                        {"buildParallel", toParam(0)}
                    }
			);

		cout << "starting octree sample filter" << endl;
		clock_t time_a = clock();
		octreeFilter->inPlaceFilter(in);
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



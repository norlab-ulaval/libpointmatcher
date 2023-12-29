#include <iostream>
#include <pointmatcher/PointMatcher.h>
#include <boost/assign.hpp>
#include <ctime>
#include <omp.h>

using namespace PointMatcherSupport;
using namespace std;
using namespace boost;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

int main()
{
    DP in(DP::load("apartment.vtk"));

    std::shared_ptr<PM::DataPointsFilter> kmeansfilter =
        PM::get().DataPointsFilterRegistrar.create(
                "KMeansClusteringDataPointsFilter",
                {
                    {"k", toParam(4000)},
                    {"iter", toParam(4)},
                    {"epsilon", toParam(0.001)},
                    {"seed", toParam(-1)}
                }
        );

    cout << "starting k-means clustering filter" << endl;
    clock_t time_a = clock();
    auto out = kmeansfilter->filter(in);
    clock_t time_b = clock();

    if (time_a == ((clock_t)-1) || time_b == ((clock_t)-1))
    {
        perror("Unable to calculate elapsed time");
        return -1;
    }
    else
    {
        cout << "Performed k-means clustering in " << (float)(time_b - time_a)/CLOCKS_PER_SEC << " seconds" << endl;
    }
    out.save("kmeans_out.vtk");
    return 0;
}



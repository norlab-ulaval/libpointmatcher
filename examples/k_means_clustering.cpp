#include <iostream>
#include <pointmatcher/PointMatcher.h>
#include <boost/assign.hpp>
#include <ctime>
#include <omp.h>
#include <chrono>

using namespace PointMatcherSupport;
using namespace std;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

template<
        class result_t   = std::chrono::milliseconds,
        class clock_t    = std::chrono::steady_clock,
        class duration_t = std::chrono::milliseconds
>
auto since(std::chrono::time_point<clock_t, duration_t> const& start)
{
    return std::chrono::duration_cast<result_t>(clock_t::now() - start).count() / 1000.0;
}

int main()
{
    DP in(DP::load("apartment.vtk"));

    std::shared_ptr<PM::DataPointsFilter> kmeansfilter =
            PM::get().DataPointsFilterRegistrar.create(
                    "KMeansClusteringDataPointsFilter",
                    {
                            {"k",       toParam(4000)},
                            {"iter",    toParam(4)},
                            {"epsilon", toParam(0.001)},
                            {"seed",    toParam(-1)}
                    }
            );

    cout << "starting k-means clustering filter" << endl;
    auto start = chrono::steady_clock::now();
    clock_t time_a = clock();
    auto out = kmeansfilter->filter(in);
    clock_t time_b = clock();

    if(time_a == ((clock_t)-1) || time_b == ((clock_t)-1))
    {
        perror("Unable to calculate elapsed time");
        return -1;
    }
    else
    {
        unsigned number_of_threads = std::max(std::atoi(std::getenv("OMP_NUM_THREADS")), 1);
        cout << "Performed k-means clustering in " << (float)(time_b - time_a) / CLOCKS_PER_SEC << " seconds, distributed over " << number_of_threads << " threads." << endl;
        cout << "Elapsed time " << since(start) << " seconds." << endl;
    }
    out.save("kmeans_out.vtk");
    return 0;
}



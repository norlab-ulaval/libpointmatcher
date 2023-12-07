#include <iostream>
#include <pointmatcher/PointMatcher.h>
#include <ctime>
#include <pointmatcher/DataPointsFilters/utils/distribution.h>

using namespace PointMatcherSupport;
using namespace boost;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef typename Eigen::Vector3f Vector;
typedef typename PointMatcher<float>::Int64Matrix Times;
using Matrix = Eigen::Matrix<float, 3, 3>;

std::vector<Distribution<float>> get_random_distros(unsigned len)
{
    std::vector<Distribution<float>> distributions;
    for(int j = 0; j < len; ++j)
    {
        Vector point = Vector::Random();
        Vector diag = Vector::Random();
        diag += Vector::Ones();
        diag *= 100*diag[0];
        Matrix deviation = diag.array().matrix().asDiagonal();
        float omega = 1.0;
        Vector descriptors = Vector::Random();
        Times times(3, 1);
        times.setRandom(3, 1);

        distributions.emplace_back(point, omega, deviation, times, descriptors);
    }
    return distributions;
}

int main()
{
    unsigned num_iterations = 100;
    std::cout << "starting benchmarking of " << num_iterations << " iterations" << std::endl;
    clock_t total_time = 0;
    for(int i = 0; i < num_iterations; ++i)
    {
        std::vector<Distribution<float>> distributions = get_random_distros(100);
        clock_t time_a = clock();
        for(int j = 0; j < 1000; ++j)
        {
            for(auto &distro : distributions)
            {
                distro.computeVolume();
            }
        }
        total_time += (clock() - time_a);
    }

    std::cout << "Performed volume calculation in " << (float)(total_time)/(CLOCKS_PER_SEC) << " seconds" << std::endl;

    return 0;
}



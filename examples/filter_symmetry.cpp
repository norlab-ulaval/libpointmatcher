/*
 * filterSymmetry.cpp
 *
 *  Created on: Nov 02, 2023
 *      Author: Matej Boxan
 */

#include <iostream>
#include <pointmatcher/PointMatcher.h>
#include <boost/assign.hpp>
#include <ctime>

using namespace PointMatcherSupport;
using namespace std;
using namespace boost;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

int main(int argc, char* argv[])
{

    if(argc != 2)
    {
        std::cerr << "USAGE: filter_symmetry <path_to_input_cloud>" << std::endl;
        return -1;
    }

    DP in(DP::load(argv[1]));
    std::cout << "Starting with " << in.getNbPoints() << " points\n";
    std::shared_ptr<PM::DataPointsFilter> symmetrySample =
            PM::get().DataPointsFilterRegistrar.create(
                    "SymmetryDataPointsFilter",
                    {
                            {"vrs", toParam(5.0)},
                            {"vro", toParam(1.025)},
                            {"dt",  toParam(0.05)},
                            {"ct",  toParam(0.95)},
                            {"knn", toParam(10)}
                    }
            );

    cout << "starting symmetry filter" << endl;
    clock_t time_a = clock();
    auto out = symmetrySample->filter(in);
    clock_t time_b = clock();

    if(time_a == ((clock_t)-1) || time_b == ((clock_t)-1))
    {
        perror("Unable to calculate elapsed time");
        return -1;
    }
    else
    {
        cout << "Performed symmetry sampling in " << (float)(time_b - time_a) / CLOCKS_PER_SEC << " seconds" << endl;
    }

    cout << "Saving data" << std::endl;
//    out.save("out.vtk");

    return 0;
}



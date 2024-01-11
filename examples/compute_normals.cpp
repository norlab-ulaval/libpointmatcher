/*
 * compute_normals.cpp
 *
 *  Created on: Jan 11, 2024
 *      Author: Matej Boxan
 */

#include <iostream>
#include <pointmatcher/PointMatcher.h>

using namespace PointMatcherSupport;
using namespace std;
using namespace boost;

int main(int argc, char* argv[])
{
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    if(argc != 2)
    {
        std::cerr << "USAGE: compute_normals <path_to_input_cloud>" << std::endl;
        return -1;
    }

    DP in(DP::load(argv[1]));

    std::cout << "Starting with " << in.getNbPoints() << " points\n";
    std::shared_ptr<PM::DataPointsFilter> computeNormalsFilter =
            PM::get().DataPointsFilterRegistrar.create(
                    "ComputeNormalsDataPointsFilter"
            );

    auto out2 = computeNormalsFilter->filter(in);
    cout << "Base point cloud filtered. Ended with " << out2.getNbPoints() << " points" << std::endl;
    out2.save("out_normals.vtk");



    return 0;
}



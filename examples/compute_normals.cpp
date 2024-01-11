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

    DP cloud_in(DP::load(argv[1]));

    std::shared_ptr<PM::DataPointsFilter> computeEigVecsFilter =
            PM::get().DataPointsFilterRegistrar.create(
                    "SurfaceNormalDataPointsFilter",
                    {
                        {"knn", toParam(4)},
                        {"keepNormals", toParam(0)},
                        {"keepNormals", toParam(0)},
                        {"keepEigenValues", toParam(1)},
                        {"keepEigenVectors", toParam(1)},
                        }
            );
    auto cloud_eigenVecs = computeEigVecsFilter->filter(cloud_in);
    cout << "Base point cloud filtered. Ended with " << cloud_eigenVecs.getNbPoints() << " points" << std::endl;
    cloud_eigenVecs.save("out_eigvecs.vtk");

    std::cout << "Starting with " << cloud_eigenVecs.getNbPoints() << " points\n";
    std::shared_ptr<PM::DataPointsFilter> computeNormalsFilter =
            PM::get().DataPointsFilterRegistrar.create(
                    "ComputeNormalsDataPointsFilter"
            );

    auto out_normals = computeNormalsFilter->filter(cloud_eigenVecs);
    cout << "Base point cloud filtered. Ended with " << out_normals.getNbPoints() << " points" << std::endl;
    out_normals.save("out_normals.vtk");



    return 0;
}



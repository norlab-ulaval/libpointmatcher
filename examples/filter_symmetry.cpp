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
#include <iomanip>

using namespace PointMatcherSupport;
using namespace std;
using namespace boost;

template<typename T, typename U>
int compare_clouds(const typename PointMatcher<T>::DataPoints& cloud1, const typename PointMatcher<U>::DataPoints& cloud2)
{
    float difference_threshold = 1e-5;
    Eigen::IOFormat fmt(Eigen::StreamPrecision, 0, ", ", "; ", "", "", " [", "]");
    for(int i = 0; i < cloud1.getNbPoints(); ++i)
    {
        float difference = (cloud1.features.template cast<U>().col(i) - cloud2.features.col(i)).norm();
        if (difference > difference_threshold)
        {
            std::cout << "Point clouds differ in features on line " << i << " with difference value: " << difference << std::endl;
            std::cout << std::setprecision(9) << "Rotated: " << cloud1.features.col(i).transpose().format(fmt) << " Loaded: " << cloud2.features.col(i).transpose().format(fmt) << std::endl;
            return -1;
        }
        for(const auto& label:cloud1.descriptorLabels)
        {
            auto descriptor1 = cloud1.getDescriptorViewByName(label.text).col(i);
            auto descriptor2 = cloud2.getDescriptorViewByName(label.text).col(i);
            difference = (descriptor1.template cast<U>() - descriptor2).norm();
            if (difference > difference_threshold)
            {
                std::cout << "Point clouds differ in descriptor '" << label.text <<"' on line " << i << " with difference value: " << difference << std::endl;
                std::cout << std::setprecision(9) << "cloud1: " << descriptor1.transpose().format(fmt) << " cloud2: " << descriptor2.transpose().format(fmt) << std::endl;
                return -1;
            }
        }
    }
    return 0;
}


template<typename T>
void compare_rotation(int argc, char* argv[])
{
    typedef PointMatcher<T> PM;
    typedef typename PM::DataPoints DP;
    typedef typename PM::Parameters Parameters;
    if(argc != 2)
    {
        std::cerr << "USAGE: filter_symmetry <path_to_input_cloud>" << std::endl;
        return;
    }

    DP in(DP::load(argv[1]));
    std::cout << "Starting with " << in.getNbPoints() << " points\n";
    std::shared_ptr<typename PM::DataPointsFilter> symmetrySample =
            PM::get().DataPointsFilterRegistrar.create(
                    "SymmetryDataPointsFilter",
                    {
                            {"knn", toParam(10)},
                            {"vrs", toParam(6.0)},
                            {"vro", toParam(1.0515)},
                            {"dr",  toParam(0.102)},
                            {"ct",  toParam(1.0)}
                    }
            );

    clock_t time_a = clock();
    auto out = symmetrySample->filter(in);
    clock_t time_b = clock();

    cout << "Performed symmetry sampling in " << (float)(time_b - time_a) / CLOCKS_PER_SEC << " seconds" << endl;

    cout << "Base point cloud filtered. Ended with " << out.getNbPoints() << " points" << std::endl;
    out.save("tunnel" + std::to_string(out.getNbPoints()) + ".vtk");

    cout << "Applying transformation of 45 degrees in pitch and yaw" << std::endl;
    auto transformation =  PM::get().TransformationRegistrar.create("RigidTransformation");
    Eigen::Matrix4<T> rot = Eigen::Matrix4<T>::Identity();
    Eigen::Quaternion<T> q(0.8535534, 0.1464466, 0.3535534, 0.3535534);
    rot.template topLeftCorner<3, 3>() = q.normalized().toRotationMatrix();

    auto in_rotated = transformation->compute(in, rot);
    std::string rotated_cloud_name = "tunnel_rotated45deg.vtk";
    cout << "Saving rotated point cloud" << std::endl;
    in_rotated.save(rotated_cloud_name);
    auto out2 = symmetrySample->filter(in_rotated);
    cout << "Rotated point cloud filtered. Ended with " << out2.getNbPoints() << " points" << std::endl;

    DP in_rotated_loaded_from_file(DP::load(rotated_cloud_name));
    compare_clouds<T, T>(in_rotated, in_rotated_loaded_from_file);
    auto out3 = symmetrySample->filter(in_rotated_loaded_from_file);
    cout << "Loaded rotated point cloud filtered. Ended with " << out3.getNbPoints() << " points" << std::endl;
}

void compare_float_double(int argc, char* argv[])
{
 typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    if(argc != 2)
    {
        std::cerr << "USAGE: filter_symmetry <path_to_input_cloud>" << std::endl;
        return;
    }

    DP in(DP::load(argv[1]));
    std::cout << "Starting with " << in.getNbPoints() << " points\n";
    std::shared_ptr<PM::DataPointsFilter> symmetrySample =
            PM::get().DataPointsFilterRegistrar.create(
                    "SymmetryDataPointsFilter",
                    {
                            {"knn", toParam(10)},
                            {"vrs", toParam(6.0)},
                            {"vro", toParam(1.0515)},
                            {"dr",  toParam(0.102)},
                            {"ct",  toParam(1.0)}
                    }
            );

    auto out = symmetrySample->filter(in);
    cout << "Base point cloud filtered. Ended with " << out.getNbPoints() << " points" << std::endl;

    typedef PointMatcher<double> PM_double;
    typedef PM_double::DataPoints DP_double;

    DP_double in_double(DP_double::load(argv[1]));
    std::shared_ptr<PM_double::DataPointsFilter> symmetrySample_double =
            PM_double::get().DataPointsFilterRegistrar.create(
                    "SymmetryDataPointsFilter",
                    {
                            {"knn", toParam(10)},
                            {"vrs", toParam(6.0)},
                            {"vro", toParam(1.0515)},
                            {"dr",  toParam(0.102)},
                            {"ct",  toParam(1.0)}
                    }
            );

    auto out_double = symmetrySample_double->filter(in_double);

    cout << "Base point cloud filtered. Ended with " << out_double.getNbPoints() << " points" << std::endl;
    compare_clouds<float, double>(in, in_double);
}

int main(int argc, char* argv[])
{
    cout << "==============Evaluating float point precision==============" << std::endl;
    compare_rotation<float>(argc, argv);
    cout << "==============Evaluating double point precision==============" << std::endl;
    compare_rotation<double>(argc, argv);
    compare_float_double(argc, argv);
    return 0;
}



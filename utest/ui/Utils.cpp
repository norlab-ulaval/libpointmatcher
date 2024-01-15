#include "../utest.h"
#include <cmath>
#include "pointmatcher/DataPointsFilters/utils/utils.h"

using namespace std;
using namespace PointMatcherSupport;

#include <vector>
#include <fstream>

template<typename M>
M load_csv (const std::string & path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<std::vector<double>> data;  // To store the CSV data temporarily
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        std::vector<double> row;
        while (std::getline(lineStream, cell, ',')) {
            row.push_back(std::stod(cell));
        }
        data.push_back(row);
    }
    indata.close();
    M matrix;
    for (int i = 0; i < data.size(); ++i) {
        for (int j = 0; j < data[0].size(); ++j) {
            matrix(i, j) = data[i][j];
        }
    }

    return matrix;
}

namespace Eigen {
    template<class T>
    void swap(T&& a, T&& b){
        a.swap(b);
    }
}


//---------------------------
// utils.h
//---------------------------

TEST(Utils, GetCuboid)
{
    DP in(DP::load(dataPath + "unit_tests/utils/cuboids_in.vtk"));
    auto cuboids = load_csv<Eigen::Matrix<float, 30, 8>>(dataPath + "unit_tests/utils/cuboids.csv");

    auto omegas = in.getDescriptorViewByName("omega");
    auto deviations = in.getDescriptorViewByName("deviation");
    for(int i = 0; i < in.getNbPoints(); ++i)
    {
        Eigen::Vector3<NumericType> point = in.features.col(i).head(3);
        PM::Matrix deviation = deviations.col(i).reshaped(3, 3);
        auto omega = omegas.col(i).value();

        Eigen::Matrix3<NumericType> cov = deviation / omega;
        auto cuboid = getCuboid<NumericType>(point, cov);
        auto cuboid_ref = cuboids.block<3, 8>(i*3, 0);

        std::sort(cuboid.colwise().begin(), cuboid.colwise().end(),
            [](auto const& r1, auto const& r2){return r1(0)<r2(0);});

        std::sort(cuboid_ref.colwise().begin(), cuboid_ref.colwise().end(),
            [](auto const& r1, auto const& r2){return r1(0)<r2(0);});

        EXPECT_TRUE(cuboid_ref.isApprox(cuboid, 1e-5));
    }
}
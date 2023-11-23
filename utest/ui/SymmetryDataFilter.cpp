#include "../utest.h"
#include <cmath>
#include "pointmatcher/DataPointsFilters/Symmetry.h"

using namespace std;
using namespace PointMatcherSupport;

//---------------------------
// Symmetry DataFilter modules
//---------------------------

TEST(SymmetryDataPointsFilter, Distributions) // TODO add test for incremental statistics (running average) for descriptors calculation
{
    const float sigmaLaser = 0.0009;

	typedef typename Eigen::Vector3f Vector;
    typedef typename PointMatcher<float>::Int64Matrix Times;
	using Matrix = Eigen::Matrix<float, 3, 3>;

    Vector point1;
    point1 << -2.0, -1.0, -0.25;
    Eigen::Matrix<float, 3, 1> diag;
    diag << sigmaLaser, sigmaLaser, sigmaLaser;
    Matrix deviation1 = diag.array().matrix().asDiagonal();
    float omega1 = 1.0;
    Vector descriptors1 = Vector::Random();
    Times times1(3, 1);
    times1.setRandom(3, 1);


    auto distro1 = Distribution<float>(point1, omega1, deviation1, times1, descriptors1);

    Vector point2(3);
    point2 << 2.0, 1.0, 0.25;
    Eigen::Matrix<float, 3, 1> diag2;
    diag2 << sigmaLaser, sigmaLaser, sigmaLaser;
    Matrix deviation2 = diag.array().matrix().asDiagonal();
    float omega2 = 1.0;
    Eigen::Vector3f descriptors2 = Eigen::Vector3f::Random();
    Times times2(3, 1);
    times2.setRandom(3, 1);

    auto distro2 = Distribution<float>(point2, omega2, deviation2, times2, descriptors2);

    auto distro_c = Distribution<float>::combineDistros(distro1, distro2);

    Eigen::Matrix<float, 3, 3>result_deviation;
    result_deviation << 8.0018, 4.0, 1.0,
                         4.0, 2.0018, 0.5,
                         1.0, 0.5, 0.1268;

    EXPECT_EQ(distro_c.point, Eigen::Vector3f::Zero());
    EXPECT_TRUE(distro_c.deviation.isApprox(result_deviation));
    EXPECT_EQ(distro_c.omega, 2.0);
    EXPECT_TRUE(distro_c.times.isApprox((times1 + times2)/2));
    EXPECT_TRUE(distro_c.descriptors.isApprox((descriptors1 + descriptors2)/2));
}
//
// Created by Matěj Boxan on 2023-12-04.
//

#ifndef LIBPOINTMATCHER_DISTRIBUTION_H
#define LIBPOINTMATCHER_DISTRIBUTION_H

#include <utility>

#include "Eigen/Eigenvalues"
#include "PointMatcher.h"
#include "vector"

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

template<typename T>
struct Distribution {

	typedef typename PointMatcher<T>::Vector Vector;
    typedef typename PM::Int64Matrix TimeViewBlock;
    typedef typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> DescriptorsViewBlock;
	using Matrix33 = Eigen::Matrix<T, 3, 3>;

    Vector point;
    T omega;
    Matrix33 deviation;
    T volume = -1;
    TimeViewBlock times;
    DescriptorsViewBlock descriptors;

    Distribution(Vector point, T  omega, Matrix33 deviation, TimeViewBlock times, DescriptorsViewBlock descriptors):
        point(point),
        omega(omega),
        deviation(deviation),
        times(std::move(times)),
        descriptors(descriptors)
    {
    }

    static Distribution<T> combineDistros(Distribution<T> distro1, Distribution<T> distro2, unsigned n=2)
    {
        T omega_12 = distro1.omega + distro2.omega;
        Vector delta = distro1.point - distro2.point;

        Vector mu_12 = distro2.point + (distro1.omega * delta) / omega_12;
        Matrix33 deviation_12 = distro1.deviation + distro2.deviation
                + ((distro1.omega * distro2.omega) / omega_12) * (delta * delta.transpose());

        TimeViewBlock times_combined = (distro2.times + (n-1) * distro1.times).eval();
        times_combined = times_combined.array() / n;
        DescriptorsViewBlock descriptors_combined = (distro2.descriptors + (n-1) * distro1.descriptors).eval();
        descriptors_combined = descriptors_combined.array() / n;
        Distribution<T> distro_out = Distribution<T>(mu_12, omega_12, deviation_12,
                               times_combined,
                               descriptors_combined);
        return distro_out;
    }
    /**
     * Note that this doesn't return the actual volume, but the determinant of the covariance matrix.
     * To get the true volume, the determinant needs to be multiplied by a constant factor
     * of (2*sqrt(3))^dim, where dim is the number of dimensions (either 2 or 3)
     */
    void computeVolume()
    {
        auto covariance = deviation / omega;
        T determinant = covariance.determinant();
        volume = std::sqrt(determinant); // FIXME can this square root be removed
    }

public:
    void setTimes(const TimeViewBlock& times_in) {
        times = times_in;
    }

    void setDescriptors(const DescriptorsViewBlock& descriptors_in) {
        descriptors = descriptors_in;
    }

    T getVolume()
    {
        if(volume == -1) {
            computeVolume();
        }
        return volume;
    }

    friend std::ostream& operator<<(std::ostream& os, const Distribution& distribution)
    {
        return os << "Point:\n" << distribution.point << "\nOmega:\n" << distribution.omega << "\nDeviation:\n" << distribution.deviation << '\n';
    }
};

#endif //LIBPOINTMATCHER_DISTRIBUTION_H

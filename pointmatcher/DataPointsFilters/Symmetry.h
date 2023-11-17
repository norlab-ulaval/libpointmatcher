//
// Created by Matěj Boxan on 2023-10-09.
//

#ifndef LIBPOINTMATCHER_SYMMETRY_H
#define LIBPOINTMATCHER_SYMMETRY_H

#pragma once

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

    Distribution(Vector point, T  omega, Matrix33 deviation, TimeViewBlock times, DescriptorsViewBlock descriptors);
    static Distribution<T> combineDistros(Distribution<T> distro1, Distribution<T> distro2, unsigned n=2)
    {
        T omega_12 = distro1.omega + distro2.omega;
        T omega_12_inv = 1. / omega_12;
        Vector delta = distro1.point - distro2.point;

        Vector mu_12 = distro2.point + omega_12_inv * distro1.omega * delta;
        Matrix33 deviation_12 = distro1.deviation + distro2.deviation
                + omega_12_inv * distro1.omega * distro2.omega * (delta * delta.transpose());

        TimeViewBlock times_combined = (distro2.times + (n-1) * distro1.times).eval();
        times_combined = times_combined.array() / n;
        DescriptorsViewBlock descriptors_combined = (distro2.descriptors + (n-1) * distro1.descriptors).eval();
        descriptors_combined = descriptors_combined.array() / n;
        Distribution<T> distro_out = Distribution<T>(mu_12, omega_12, deviation_12,
                               times_combined,
                               descriptors_combined);
        return distro_out;
    }

    void computeVolume()
    {
        auto covariance = deviation / omega;
        const Eigen::SelfAdjointEigenSolver<Matrix33> solver(covariance);
        auto eigenVa = solver.eigenvalues().real();
        volume = (2. * 1.73 * eigenVa.cwiseSqrt()).prod();
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

// Distribution
//Constructor
template<typename T>
Distribution<T>::Distribution(Vector point, T  omega, Matrix33 deviation, TimeViewBlock times, DescriptorsViewBlock descriptors):
        point(point),
        omega(omega),
        deviation(deviation),
        times(std::move(times)),
        descriptors(descriptors)
{
}
// TODO add optional produced descriptors: normals, eigenvalues, eigenvectors
template<typename T>
struct SymmetryDataPointsFilter : public PointMatcher<T>::DataPointsFilter
{
    typedef PointMatcher<T> PM;
    typedef PointMatcherSupport::Parametrizable Parametrizable;
    typedef PointMatcherSupport::Parametrizable P;
    typedef Parametrizable::Parameters Parameters;
    typedef Parametrizable::ParameterDoc ParameterDoc;
    typedef Parametrizable::ParametersDoc ParametersDoc;
    typedef Parametrizable::InvalidParameter InvalidParameter;

	typedef typename PointMatcher<T>::Vector Vector;
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	typedef typename PointMatcher<T>::Matrix Matrix;

	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;

	inline static const std::string description()
	{
        return "Lossy point cloud compression using incremental statistics.\n"
               "Required descriptors: none.\n"
               "Produced descriptors:  omega, deviation.\n"
               "Altered descriptors:  all.\n"
               "Altered features:     points coordinates and number of points.";
	}
	inline static const ParametersDoc availableParameters()
	{
		return {
			{"vrs", "volume ratio for symmetry sampling", "5", "0", "inf", &P::Comp<T>},
			{"vro", "volume ratio for overlap sampling", "1.025", "0", "inf", &P::Comp<T>},
			{"dt", "distance threshold [m] for symmetry sampling", "0.05", "0", "inf", &P::Comp<T>},
			{"ct", "compressions tolerance [%]", "0.95", "0", "1", &P::Comp<T>},
			{"knn", "number of nearest neighbors to consider, including the point itself", "5", "3", "2147483647", &P::Comp<unsigned>},
			{"initialVariance", "Variance on individual point positions (isotropic)", "0.0009", "0", "inf", &P::Comp<T>},
			};
	}

	const T vrs;
	const T vro;
	const T dt;
	const T ct;
    const T initialVariance;
	const unsigned knn;

	//! Constructor, uses parameter interface
	explicit SymmetryDataPointsFilter(const Parameters& params = Parameters());
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
    void symmetrySampling(std::vector<std::shared_ptr<Distribution<T>>>& distributions);
    void overlapSampling(std::vector<std::shared_ptr<Distribution<T>>>& distributions);
    std::vector<std::shared_ptr<Distribution<T>>> getDistributionsFromCloud(DataPoints& cloud);
    DataPoints getCloudFromDistributions(const DataPoints& in_cloud, std::vector<std::shared_ptr<Distribution<T>>>& distributions);
    DataPoints getPointsFromDistributions(std::vector<std::shared_ptr<Distribution<T>>>& distributions);
    void mergeTimesDescriptors(std::vector<std::shared_ptr<Distribution<T>>>& distributions, const std::vector<unsigned>& indexesToMerge);
};

#endif //LIBPOINTMATCHER_SYMMETRY_H

//
// Created by Matěj Boxan on 2023-10-09.
//

#ifndef LIBPOINTMATCHER_SYMMETRY_H
#define LIBPOINTMATCHER_SYMMETRY_H

#pragma once

#include "Eigen/Eigenvalues"
#include "PointMatcher.h"

//! TODO
template<typename T>
struct SymmetryDataPointsFilter : public PointMatcher<T>::DataPointsFilter
{

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

	using Matrix33 = Eigen::Matrix<T, 3, 3>;

	inline static const std::string description()
	{
		return "TODO";
	}
	inline static const ParametersDoc availableParameters()
	{
		return {
			{"vrs", "volume ratio for symmetry sampling", "5", "0", "inf", &P::Comp<T>},
			{"vro", "volume ratio for overlap sampling", "1.025", "0", "inf", &P::Comp<T>},
			{"dt", "distance threshold [m] for symmetry sampling", "0.05", "0", "inf", &P::Comp<T>},
			{"ct", "compressions tolerance [%]", "0.95", "0", "1", &P::Comp<T>},
			{"knn", "number of nearest neighbors to consider, including the point itself", "5", "3", "2147483647", &P::Comp<unsigned>},
			};
	}

	const T vrs;
	const T vro;
	const T dt;
	const T ct;
	const unsigned knn;

	//! Constructor, uses parameter interface
	SymmetryDataPointsFilter(const Parameters& params = Parameters());
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
    void symmetrySampling(DataPoints& cloud);
    void overlapSampling(DataPoints& cloud);
};

template<typename T>
struct Distribution {

	typedef typename PointMatcher<T>::Vector Vector;
	typedef typename PointMatcher<T>::Matrix Matrix;

    Vector point;
    T omega;
    Matrix deviation;
    T volume;

    Distribution(Vector point, T  omega, Matrix deviation);
    static Distribution<T> combineDistros(Distribution<T> distro1, Distribution<T> distro2)
    {
        T omega_12 = distro1.omega + distro2.omega;
        T omega_12_inv = 1. / omega_12;
        Vector delta = distro1.point - distro2.point;

        Vector mu_12 = distro2.point + omega_12_inv * distro1.omega * delta;
        Matrix deviation_12 = distro1.deviation + distro2.deviation
                + omega_12_inv * distro1.omega * distro2.omega * (delta * delta.transpose());
        return Distribution<T>(mu_12, omega_12, deviation_12);
    }

    void computeVolume()
    {
        auto covariance = deviation / omega;
        const Eigen::EigenSolver<Matrix> solver(covariance);
        auto eigenVa = solver.eigenvalues().real();
        volume = (2. * 1.73 * eigenVa.cwiseSqrt()).prod();
    }

public: T getVolume()
    {
        if(volume == -1) {
            computeVolume();
        }
        return volume;
    }
};

#endif //LIBPOINTMATCHER_SYMMETRY_H

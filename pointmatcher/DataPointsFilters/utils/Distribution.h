#ifndef LIBPOINTMATCHER_DISTRIBUTION_H
#define LIBPOINTMATCHER_DISTRIBUTION_H

#include "pointmatcher/PointMatcher.h"
#include <Eigen/Core>

template<typename T>
class Distribution
{
private:
	typedef PointMatcher<T> PM;

	typename PM::Vector mean;

	typename PM::Matrix weightSum;
	typename PM::Matrix omega;
	typename PM::Matrix covariance;
	bool isCovarianceCached;
	typename PM::Vector covarianceEigenValues;
	typename PM::Matrix covarianceEigenVectors;
	bool isEigenDecompositionCached;
public:

	explicit Distribution(const typename PM::Vector& point);
	Distribution(const typename PM::Vector& point, const typename PM::Matrix& covariance);
	Distribution(const typename PM::Vector& mean, const typename PM::Matrix& covariance, const typename PM::Matrix& weightSum);
	Distribution(const typename PM::Matrix& omega, const typename PM::Matrix& weightSum, const typename PM::Vector& mean);
	Distribution combine(const Distribution& otherDistribution) const;
	typename PM::Vector getMean() const;
	typename PM::Matrix getCovariance();
	typename PM::Matrix getWeightSum() const;
	typename PM::Matrix getOmega() const;
	typename PM::Vector getCovarianceEigenValues();
	typename PM::Matrix getCovarianceEigenVectors();

};

#endif

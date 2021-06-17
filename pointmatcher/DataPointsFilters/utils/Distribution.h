#ifndef LIBPOINTMATCHER_DISTRIBUTION_H
#define LIBPOINTMATCHER_DISTRIBUTION_H

#include "pointmatcher/PointMatcher.h"
#include <Eigen/Core>

template<typename T>
class Distribution
{
private:
	typedef PointMatcher<T> PM;

	Distribution(const typename PM::Matrix& omega, const typename PM::Matrix& weightSum, const typename PM::Vector& mean);

	typename PM::Vector mean;
	typename PM::Matrix omega;
	typename PM::Matrix weightSum;

public:
	Distribution(const typename PM::Vector& point);
	Distribution(const typename PM::Vector& point, const typename PM::Matrix& covariance);
	Distribution(const typename PM::Vector& mean, const typename PM::Matrix& covariance, const typename PM::Matrix& weightSum);
	Distribution combine(const Distribution& otherDistribution);
	typename PM::Vector getMean();
	typename PM::Matrix getCovariance();
	typename PM::Matrix getWeightSum();
};

#endif

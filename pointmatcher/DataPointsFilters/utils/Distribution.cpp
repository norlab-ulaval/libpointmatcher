#include "Distribution.h"

template<typename T>
Distribution<T>::Distribution(const typename PM::Vector& point):
		mean(point),
		omega(PM::Matrix::Identity(point.rows(), point.rows())),
		weightSum(PM::Matrix::Zero(point.rows(), point.rows()))
{
}

template<typename T>
Distribution<T>::Distribution(const typename PM::Vector& point, const typename PM::Matrix& covariance):
		mean(point),
		omega(covariance.inverse()),
		weightSum(PM::Matrix::Zero(point.rows(), point.rows()))
{
}

template<typename T>
Distribution<T>::Distribution(const typename PM::Vector& mean, const typename PM::Matrix& covariance, const typename PM::Matrix& weightSum)
{
	this->mean = mean;

	this->weightSum = weightSum;

	if(weightSum.isZero())
	{
		omega = covariance.inverse();
	}
	else
	{
		omega = weightSum * covariance.inverse();
	}
}

template<typename T>
Distribution<T>::Distribution(const typename PM::Matrix& omega, const typename PM::Matrix& weightSum, const typename PM::Vector& mean):
		mean(mean),
		omega(omega),
		weightSum(weightSum)
{
}

template<typename T>
Distribution<T> Distribution<T>::combine(const Distribution& otherDistribution)
{
	typename PM::Vector mu_A = mean;
	typename PM::Matrix omega_A = omega;
	typename PM::Matrix S_A = weightSum;

	typename PM::Vector mu_B = otherDistribution.mean;
	typename PM::Matrix omega_B = otherDistribution.omega;
	typename PM::Matrix S_B = otherDistribution.weightSum;

	typename PM::Matrix omega_AB = omega_A + omega_B;
	typename PM::Matrix omega_AB_inv = omega_AB.inverse();
	typename PM::Vector delta = mu_A - mu_B;

	typename PM::Vector mu_AB = mu_B + omega_A * omega_AB_inv * delta;

	typename PM::Matrix S_AB = S_A + S_B + omega_A * omega_B * omega_AB_inv * delta * delta.transpose();

	return Distribution<T>(omega_AB, S_AB, mu_AB);
}

template<typename T>
typename Distribution<T>::PM::Vector Distribution<T>::getMean()
{
	return mean;
}

template<typename T>
typename Distribution<T>::PM::Matrix Distribution<T>::getCovariance()
{
	typename PM::Matrix covariance;
	if(weightSum.isZero())
	{
		covariance = omega.inverse();
	}
	else
	{
		covariance = omega.inverse() * weightSum;
	}
	return covariance;
}

template<typename T>
typename Distribution<T>::PM::Matrix Distribution<T>::getWeightSum()
{
	return weightSum;
}

template
struct Distribution<float>;
template
struct Distribution<double>;

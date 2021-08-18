#include <Eigen/Eigenvalues>
#include "Distribution.h"
#include "utils.h"

template<typename T>
Distribution<T>::Distribution(const typename PM::Vector& point):
		mean(point),
		weightSum(PM::Matrix::Zero(point.rows(), point.rows())),
		omega(PM::Matrix::Identity(point.rows(), point.rows())),
		isCovarianceCached(false),
		isEigenDecompositionCached(false)
{
}

template<typename T>
Distribution<T>::Distribution(const typename PM::Vector& point, const typename PM::Matrix& covariance):
		mean(point),
		weightSum(PM::Matrix::Zero(point.rows(), point.rows())),
		covariance(covariance),
		isCovarianceCached(true),
		isEigenDecompositionCached(false)
{
	omega = PointMatcherSupport::inverseCovariance<T>(covariance, getCovarianceEigenValues());
}

template<typename T>
Distribution<T>::Distribution(const typename PM::Vector& mean, const typename PM::Matrix& covariance, const typename PM::Matrix& weightSum):
		mean(mean),
		weightSum(weightSum),
		covariance(covariance),
		isCovarianceCached(true),
		isEigenDecompositionCached(false)
{
	omega = PointMatcherSupport::inverseCovariance<T>(covariance, getCovarianceEigenValues());
}

template<typename T>
Distribution<T>::Distribution(const typename PM::Matrix& omega, const typename PM::Matrix& weightSum, const typename PM::Vector& mean):
		mean(mean),
		weightSum(weightSum),
		omega(omega),
		isCovarianceCached(false),
		isEigenDecompositionCached(false)
{
}

template<typename T>
Distribution<T> Distribution<T>::combine(const Distribution& otherDistribution) const
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
typename Distribution<T>::PM::Vector Distribution<T>::getMean() const
{
	return mean;
}

template<typename T>
typename Distribution<T>::PM::Matrix Distribution<T>::getCovariance()
{
	if(!isCovarianceCached)
	{
		if(weightSum.isZero())
		{
			covariance = omega.inverse();
		}
		else
		{
			covariance = omega.inverse() * weightSum;
		}
		isCovarianceCached = true;
	}

	return covariance;
}

template<typename T>
typename Distribution<T>::PM::Matrix Distribution<T>::getWeightSum() const
{
	return weightSum;
}

template<typename T>
typename Distribution<T>::PM::Matrix Distribution<T>::getOmega() const
{
	return omega;
}

template<typename T>
typename Distribution<T>::PM::Vector Distribution<T>::getCovarianceEigenValues()
{
	if(!isEigenDecompositionCached)
	{
		const Eigen::EigenSolver<typename PM::Matrix> eigenSolver(getCovariance());
		covarianceEigenValues = eigenSolver.eigenvalues().real();
		covarianceEigenVectors = eigenSolver.eigenvectors().real();
		isEigenDecompositionCached = true;
	}

	return covarianceEigenValues;
}

template<typename T>
typename Distribution<T>::PM::Matrix Distribution<T>::getCovarianceEigenVectors()
{
	if(!isEigenDecompositionCached)
	{
		const Eigen::EigenSolver<typename PM::Matrix> eigenSolver(getCovariance());
		covarianceEigenValues = eigenSolver.eigenvalues().real();
		covarianceEigenVectors = eigenSolver.eigenvectors().real();
		isEigenDecompositionCached = true;
	}

	return covarianceEigenVectors;
}

template
class Distribution<float>;

template
class Distribution<double>;

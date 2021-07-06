#include "Eigen/Eigenvalues"
#include "Uncompression.h"
#include "Functions.h"

template<typename T>
UncompressionDataPointsFilter<T>::UncompressionDataPointsFilter(const Parameters& params):
		PointMatcher<T>::DataPointsFilter("UncompressionDataPointsFilter", UncompressionDataPointsFilter::availableParameters(), params)
{
	try
	{
		seed = this->template get<size_t>("seed");
	}
	catch(const Parametrizable::InvalidParameter&)
	{
		seed = static_cast<size_t>(1); // rand default seed number
	}
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints UncompressionDataPointsFilter<T>::filter(const typename PM::DataPoints& input)
{
	typename PM::DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void UncompressionDataPointsFilter<T>::inPlaceFilter(typename PM::DataPoints& cloud)
{
	if(!cloud.descriptorExists("covariance"))
	{
		throw typename PM::DataPoints::InvalidField("UncompressionDataPointsFilter: Error, cannot find covariance in descriptors.");
	}
	if(!cloud.descriptorExists("nbPoints"))
	{
		throw typename PM::DataPoints::InvalidField("UncompressionDataPointsFilter: Error, cannot find number of points in descriptors.");
	}

	std::srand(seed);
	if(cloud.descriptorExists("weightSum"))
	{
		cloud.removeDescriptor("weightSum");
	}

	unsigned nbDim = cloud.getEuclideanDim();
	typename PM::DataPoints compressedCloud = cloud;

	const auto& covarianceVectors = compressedCloud.getDescriptorViewByName("covariance");
	const auto& nbPoints = compressedCloud.getDescriptorViewByName("nbPoints");

	cloud.conservativeResize(nbPoints.sum());

	int processedPoints = 0;
	for(unsigned i = 0; i < compressedCloud.getNbPoints(); ++i)
	{
		typename PM::Matrix covariance = PM::Matrix::Zero(nbDim, nbDim);
		for(unsigned j = 0; j < nbDim; ++j)
		{
			covariance.col(j) = covarianceVectors.block(j * nbDim, i, nbDim, 1);
		}

		const Eigen::EigenSolver<typename PM::Matrix> solver(covariance);
		typename PM::Vector eigenValues = solver.eigenvalues().real().unaryExpr([](T element)
																				{ return PointMatcherSupport::anyabs(element) < 1e-6 ? T(0) : element; });
		typename PM::Matrix eigenVectors = solver.eigenvectors().real();

		for(unsigned j = 0; j < nbPoints(0, i); ++j)
		{
			cloud.setColFrom(processedPoints, compressedCloud, i);
			typename PM::Vector sampledPoint = std::sqrt(3) * eigenValues.array().sqrt() * PM::Vector::Random(nbDim).array();
			cloud.features.col(processedPoints).topRows(nbDim) = compressedCloud.features.col(i).topRows(nbDim) + (eigenVectors * sampledPoint);
			++processedPoints;
		}
	}

	cloud.removeDescriptor("covariance");
	cloud.removeDescriptor("nbPoints");
}

template
struct UncompressionDataPointsFilter<float>;
template
struct UncompressionDataPointsFilter<double>;

#include "Eigen/Eigenvalues"
#include "Uncompression.h"
#include "Functions.h"

template<typename T>
UncompressionDataPointsFilter<T>::UncompressionDataPointsFilter(const Parameters& params):
		PointMatcher<T>::DataPointsFilter("UncompressionDataPointsFilter",
										  UncompressionDataPointsFilter::availableParameters(), params),
		maxDensity(Parametrizable::get<T>("maxDensity"))
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
typename PointMatcher<T>::DataPoints UncompressionDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void UncompressionDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	if(!cloud.descriptorExists("covariance"))
	{
		throw InvalidField("UncompressionDataPointsFilter: Error, cannot find covariance in descriptors.");
	}
	if(!cloud.descriptorExists("nbPoints"))
	{
		throw InvalidField("UncompressionDataPointsFilter: Error, cannot find number of points in descriptors.");
	}

	std::srand(seed);

	if(cloud.descriptorExists("initialPosition"))
	{
		cloud.removeDescriptor("initialPosition");
	}

	if(cloud.descriptorExists("weightSum"))
	{
		cloud.removeDescriptor("weightSum");
	}

	unsigned nbDim = cloud.getEuclideanDim();
	DataPoints compressedCloud = cloud;

	const auto& covarianceVectors = compressedCloud.getDescriptorViewByName("covariance");
	const auto& nbPoints = compressedCloud.getDescriptorViewByName("nbPoints");

	cloud.conservativeResize(nbPoints.sum());

	unsigned processedPoints = 0;

	for(unsigned i = 0; i < compressedCloud.getNbPoints(); ++i)
	{
		Matrix covariance = Matrix::Zero(nbDim, nbDim);

		for(unsigned j = 0; j < nbDim; ++j)
		{
			covariance.col(j) = covarianceVectors.block(j * nbDim, i, nbDim, 1);
		}

		const Eigen::EigenSolver<Matrix> solver(covariance);
		Vector eigenValues = solver.eigenvalues().real().unaryExpr([](T element)
																   { return element < T(1e-6) ? T(1e-6) : element; });
		Matrix eigenVectors = solver.eigenvectors().real();

		T currentVolume = 1.0;
		for(unsigned j = 0; j < nbDim; ++j)
		{
			currentVolume *= 2.0 * std::sqrt(3.0 * eigenValues(j));
		}

		unsigned nbPointsToUncompress = std::min(nbPoints(0, i), maxDensity * currentVolume);

		for(unsigned j = 0; j < nbPointsToUncompress; ++j)
		{
			cloud.setColFrom(processedPoints, compressedCloud, i);
			Vector sampledPoint = std::sqrt(3) * eigenValues.array().sqrt() * Vector::Random(nbDim).array();
			cloud.features.col(processedPoints).topRows(nbDim) = compressedCloud.features.col(i).topRows(nbDim) + (eigenVectors * sampledPoint);
			++processedPoints;
		}
	}

	cloud.conservativeResize(processedPoints);

	cloud.removeDescriptor("covariance");
	cloud.removeDescriptor("nbPoints");
}

template
struct UncompressionDataPointsFilter<float>;
template
struct UncompressionDataPointsFilter<double>;

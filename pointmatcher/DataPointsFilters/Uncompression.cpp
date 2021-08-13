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
	if(!cloud.descriptorExists("omega"))
	{
		throw InvalidField("UncompressionDataPointsFilter: Error, cannot find omega in descriptors.");
	}
	if(!cloud.descriptorExists("weightSum"))
	{
		throw InvalidField("UncompressionDataPointsFilter: Error, cannot find weight sum in descriptors.");
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

	unsigned nbDim = cloud.getEuclideanDim();
	DataPoints compressedCloud = cloud;

	const auto& omegaVectors = compressedCloud.getDescriptorViewByName("omega");
	const auto& weightSumVectors = compressedCloud.getDescriptorViewByName("weightSum");
	const auto& nbPoints = compressedCloud.getDescriptorViewByName("nbPoints");

	cloud.conservativeResize(nbPoints.sum());

	unsigned processedPoints = 0;

	for(unsigned i = 0; i < compressedCloud.getNbPoints(); ++i)
	{
		Matrix omega = Matrix::Zero(nbDim, nbDim);
		Matrix weightSum = Matrix::Zero(nbDim, nbDim);
		for(unsigned j = 0; j < nbDim; ++j)
		{
			omega.col(j) = omegaVectors.block(j * nbDim, i, nbDim, 1);
			weightSum.col(j) = weightSumVectors.block(j * nbDim, i, nbDim, 1);
		}
		Matrix covariance = omega.inverse() * weightSum;

		const Eigen::EigenSolver<Matrix> solver(covariance);
		Vector eigenValues = solver.eigenvalues().real().cwiseAbs().unaryExpr([](T element)
																			  { return element < T(1e-6) ? T(1e-6) : element; });
		Matrix eigenVectors = solver.eigenvectors().real();

		const T currentVolume = (2.0 * std::sqrt(3.0) * eigenValues.cwiseSqrt()).prod();

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

	cloud.removeDescriptor("omega");
	cloud.removeDescriptor("weightSum");
	cloud.removeDescriptor("nbPoints");
}

template
struct UncompressionDataPointsFilter<float>;
template
struct UncompressionDataPointsFilter<double>;

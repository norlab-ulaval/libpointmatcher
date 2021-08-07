#include "Compression.h"
#include "MatchersImpl.h"
#include "utils/Distribution.h"
#include "utils/utils.h"
#include <boost/optional.hpp>

template<typename T>
CompressionDataPointsFilter<T>::CompressionDataPointsFilter(const Parameters& params) :
		PointMatcher<T>::DataPointsFilter("CompressionDataPointsFilter", CompressionDataPointsFilter::availableParameters(), params),
		knn(Parametrizable::get<unsigned>("knn")),
		maxDist(Parametrizable::get<T>("maxDist")),
		epsilon(Parametrizable::get<T>("epsilon")),
		maxIterationCount(Parametrizable::get<unsigned>("maxIterationCount")),
		initialVariance(Parametrizable::get<T>("initialVariance")),
		maxDeviation(Parametrizable::get<T>("maxDeviation")),
		maxVolumeRatio(Parametrizable::get<T>("maxVolumeRatio")),
		keepNormals(Parametrizable::get<bool>("keepNormals")),
		keepEigenValues(Parametrizable::get<bool>("keepEigenValues")),
		keepEigenVectors(Parametrizable::get<bool>("keepEigenVectors")),
		sortEigen(Parametrizable::get<bool>("sortEigen"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints CompressionDataPointsFilter<T>::filter(const typename PM::DataPoints& input)
{
	typename PM::DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void CompressionDataPointsFilter<T>::inPlaceFilter(typename PM::DataPoints& cloud)
{
	unsigned featDim = cloud.getEuclideanDim();

	std::vector<Distribution<T>> distributions;
	distributions.reserve(cloud.getNbPoints());
	if(!cloud.descriptorExists("initialPosition") || !cloud.descriptorExists("covariance") || !cloud.descriptorExists("weightSum") || !cloud.descriptorExists("nbPoints"))
	{
		for(unsigned i = 0; i < cloud.getNbPoints(); ++i)
		{
			distributions.emplace_back(cloud.features.col(i).topRows(featDim), initialVariance * PM::Matrix::Identity(featDim, featDim));
		}
	}
	else
	{
		const auto& covarianceVectors = cloud.getDescriptorViewByName("covariance");
		const auto& weightSumVectors = cloud.getDescriptorViewByName("weightSum");
		for(unsigned i = 0; i < cloud.getNbPoints(); ++i)
		{
			typename PM::Matrix covariance = PM::Matrix::Zero(featDim, featDim);
			typename PM::Matrix weightSum = PM::Matrix::Zero(featDim, featDim);
			for(unsigned j = 0; j < featDim; ++j)
			{
				covariance.col(j) = covarianceVectors.block(j * featDim, i, featDim, 1);
				weightSum.col(j) = weightSumVectors.block(j * featDim, i, featDim, 1);
			}
			distributions.emplace_back(cloud.features.col(i).topRows(featDim), covariance, weightSum);
		}
	}

	if(!cloud.descriptorExists("initialPosition"))
	{
		cloud.addDescriptor("initialPosition", cloud.features.topRows(featDim));
	}
	if(!cloud.descriptorExists("covariance"))
	{
		typename PM::Matrix covariances = PM::Matrix::Zero(std::pow(featDim, 2), cloud.getNbPoints());
		if(featDim == 2)
		{
			covariances.row(0) = PM::Matrix::Constant(1, cloud.getNbPoints(), initialVariance);
			covariances.row(3) = PM::Matrix::Constant(1, cloud.getNbPoints(), initialVariance);
		}
		else
		{
			covariances.row(0) = PM::Matrix::Constant(1, cloud.getNbPoints(), initialVariance);
			covariances.row(4) = PM::Matrix::Constant(1, cloud.getNbPoints(), initialVariance);
			covariances.row(8) = PM::Matrix::Constant(1, cloud.getNbPoints(), initialVariance);
		}
		cloud.addDescriptor("covariance", covariances);
	}
	if(!cloud.descriptorExists("weightSum"))
	{
		cloud.addDescriptor("weightSum", PM::Matrix::Zero(std::pow(featDim, 2), cloud.getNbPoints()));
	}
	if(!cloud.descriptorExists("nbPoints"))
	{
		cloud.addDescriptor("nbPoints", PM::Matrix::Ones(1, cloud.getNbPoints()));
	}

	unsigned currentNbPoints = cloud.getNbPoints();
	unsigned iterationCount = 0;
	typename PM::DataPoints tempCloud;
	while(tempCloud.getNbPoints() != cloud.getNbPoints() && iterationCount++ < maxIterationCount)
	{
		tempCloud = cloud;
		Eigen::Matrix<bool, 1, Eigen::Dynamic> masks = Eigen::Matrix<bool, 1, Eigen::Dynamic>::Constant(1, tempCloud.getNbPoints(), true);

		unsigned nbNeighbors = std::min(knn, tempCloud.getNbPoints());
		Parameters params{{"knn",     PointMatcherSupport::toParam(nbNeighbors)},
						  {"maxDist", PointMatcherSupport::toParam(maxDist)},
						  {"epsilon", PointMatcherSupport::toParam(epsilon)}};
		typename MatchersImpl<T>::KDTreeMatcher matcher(params);
		matcher.init(tempCloud);
		typename PM::Matches matches(typename PM::Matches::Dists(nbNeighbors, tempCloud.getNbPoints()), typename PM::Matches::Ids(nbNeighbors, tempCloud.getNbPoints()));
		matches = matcher.findClosests(tempCloud);

		for(unsigned i = 0; i < tempCloud.getNbPoints(); ++i)
		{
			if(masks(0, i))
			{
				Distribution<T> neighborhoodDistribution(distributions[matches.ids(0, i)]);

				Vector eigenValues = distributions[matches.ids(0, i)].getCovarianceEigenValues().cwiseAbs().unaryExpr([](T element)
																													  { return element < T(1e-6) ? T(1e-6) : element; });
				T sumOfVolumes = (2.0 * std::sqrt(3.0) * eigenValues.cwiseSqrt()).prod();
				for(unsigned j = 1; j < nbNeighbors; ++j)
				{
					if(matches.ids(j, i) != PM::Matches::InvalidId && masks(0, matches.ids(j, i)))
					{
						neighborhoodDistribution = neighborhoodDistribution.combine(distributions[matches.ids(j, i)]);

						eigenValues = distributions[matches.ids(j, i)].getCovarianceEigenValues().cwiseAbs().unaryExpr([](T element)
																													   { return element < T(1e-6) ? T(1e-6) : element; });
						sumOfVolumes += (2.0 * std::sqrt(3.0) * eigenValues.cwiseSqrt()).prod();
					}
				}

				typename PM::Vector delta = neighborhoodDistribution.getMean() - tempCloud.getDescriptorViewByName("initialPosition").col(i);
				T mahalanobisDistance = std::sqrt(delta.transpose() * distributions[i].getCovariance() * delta);

				eigenValues = distributions[i].getCovarianceEigenValues().cwiseAbs().unaryExpr([](T element)
																							   { return element < T(1e-6) ? T(1e-6) : element; });
				const T neighborhoodDistributionVolume = (2.0 * std::sqrt(3.0) * eigenValues.cwiseSqrt()).prod();
				const T volumeRatio = neighborhoodDistributionVolume / sumOfVolumes;

				if(mahalanobisDistance <= maxDeviation && volumeRatio <= maxVolumeRatio)
				{
					distributions[i] = neighborhoodDistribution;
					tempCloud.features.col(i).topRows(featDim) = neighborhoodDistribution.getMean();
					for(unsigned j = 0; j < featDim; ++j)
					{
						tempCloud.getDescriptorViewByName("covariance").block(j * featDim, i, featDim, 1) = neighborhoodDistribution.getCovariance().col(j);
						tempCloud.getDescriptorViewByName("weightSum").block(j * featDim, i, featDim, 1) = neighborhoodDistribution.getWeightSum().col(j);
					}
					for(unsigned j = 1; j < nbNeighbors; ++j)
					{
						if(matches.ids(j, i) != PM::Matches::InvalidId && masks(0, matches.ids(j, i)))
						{
							tempCloud.getDescriptorViewByName("nbPoints")(0, i) += tempCloud.getDescriptorViewByName("nbPoints")(0, matches.ids(j, i));
							masks(0, matches.ids(j, i)) = false;
							--currentNbPoints;
						}
					}
				}
			}
		}

		cloud.conservativeResize(currentNbPoints);
		unsigned addedElements = 0;
		for(unsigned i = 0; i < tempCloud.getNbPoints(); ++i)
		{
			if(masks(0, i))
			{
				cloud.setColFrom(addedElements++, tempCloud, i);
			}
			else
			{
				distributions.erase(distributions.begin() + addedElements);
			}
		}
	}

	if(keepNormals || keepEigenValues || keepEigenVectors)
	{
		boost::optional<View> normals;
		boost::optional<View> eigenValues;
		boost::optional<View> eigenVectors;

		Labels cloudLabels;
		if(keepNormals)
			cloudLabels.emplace_back("normals", featDim);
		if(keepEigenValues)
			cloudLabels.emplace_back("eigValues", featDim);
		if(keepEigenVectors)
			cloudLabels.emplace_back("eigVectors", featDim * featDim);

		cloud.allocateDescriptors(cloudLabels);

		if(keepNormals)
			normals = cloud.getDescriptorViewByName("normals");
		if(keepEigenValues)
			eigenValues = cloud.getDescriptorViewByName("eigValues");
		if(keepEigenVectors)
			eigenVectors = cloud.getDescriptorViewByName("eigVectors");

		for(unsigned i = 0; i < cloud.getNbPoints(); ++i)
		{
			const Matrix pointCovariance(distributions[i].getCovariance());
			Vector pointEigenValues = Vector::Zero(featDim);
			Matrix pointEigenVectors = Matrix::Zero(featDim, featDim);

			if(pointCovariance.fullPivHouseholderQr().rank() + 1 >= featDim)
			{
				pointEigenValues = distributions[i].getCovarianceEigenValues();
				pointEigenVectors = distributions[i].getCovarianceEigenVectors();

				if(sortEigen)
				{
					const std::vector<size_t> sortedIndexes = PointMatcherSupport::sortIndexes<T>(pointEigenValues);
					const unsigned sortedIndexesSize = sortedIndexes.size();
					pointEigenValues = PointMatcherSupport::sortEigenValues<T>(pointEigenValues);
					Matrix eigenVectorsCopy = pointEigenVectors;

					for(unsigned k = 0; k < sortedIndexesSize; ++k)
						pointEigenVectors.col(k) = eigenVectorsCopy.col(sortedIndexes[k]);
				}
			}

			if(keepNormals)
			{
				if(sortEigen)
					normals->col(i) = pointEigenVectors.col(0);
				else
					normals->col(i) = PointMatcherSupport::computeNormal<T>(pointEigenValues, pointEigenVectors);

				// clamp normals to [-1,1] to handle approximation errors
				normals->col(i) = normals->col(i).cwiseMax(-1.0).cwiseMin(1.0);
			}

			if(keepEigenValues)
				eigenValues->col(i) = pointEigenValues;

			if(keepEigenVectors)
				eigenVectors->col(i) = PointMatcherSupport::serializeEigVec<T>(pointEigenVectors);
		}
	}
}

template
struct CompressionDataPointsFilter<float>;
template
struct CompressionDataPointsFilter<double>;

#include "Compression.h"
#include "MatchersImpl.h"
#include "utils/Distribution.h"

template<typename T>
CompressionDataPointsFilter<T>::CompressionDataPointsFilter(const Parameters& params) :
		PointMatcher<T>::DataPointsFilter("CompressionDataPointsFilter", CompressionDataPointsFilter::availableParameters(), params),
		knn(Parametrizable::get<int>("knn")),
		maxDist(Parametrizable::get<T>("maxDist")),
		epsilon(Parametrizable::get<T>("epsilon")),
		maxDeviation(Parametrizable::get<T>("maxDeviation"))
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
	int nbDim = cloud.getEuclideanDim();

	std::vector<Distribution<T>> distributions;
	if(!cloud.descriptorExists("covariance") || !cloud.descriptorExists("weightSum"))
	{
		for(int i = 0; i < cloud.getNbPoints(); ++i)
		{
			distributions.emplace_back(Distribution<T>(cloud.features.col(i).topRows(nbDim)));
		}
	}
	else
	{
		const auto& covarianceVectors = cloud.getDescriptorViewByName("covariance");
		const auto& weightSumVectors = cloud.getDescriptorViewByName("weightSum");
		for(int i = 0; i < cloud.getNbPoints(); ++i)
		{
			typename PM::Matrix covariance = PM::Matrix::Zero(nbDim, nbDim);
			typename PM::Matrix weightSum = PM::Matrix::Zero(nbDim, nbDim);
			for(int j = 0; j < nbDim; ++j)
			{
				covariance.col(j) = covarianceVectors.block(j * nbDim, i, nbDim, 1);
				weightSum.col(j) = weightSumVectors.block(j * nbDim, i, nbDim, 1);
			}
			distributions.emplace_back(Distribution<T>(cloud.features.col(i).topRows(nbDim), covariance, weightSum));
		}
	}

	Parameters params;
	params["knn"] = PointMatcherSupport::toParam(knn);
	params["maxDist"] = PointMatcherSupport::toParam(maxDist);
	params["epsilon"] = PointMatcherSupport::toParam(epsilon);
	typename MatchersImpl<T>::KDTreeMatcher matcher(params);
	matcher.init(cloud);
	typename PM::Matches matches(typename PM::Matches::Dists(knn, cloud.getNbPoints()), typename PM::Matches::Ids(knn, cloud.getNbPoints()));
	matches = matcher.findClosests(cloud);

	Eigen::Matrix<bool, 1, Eigen::Dynamic> masks = Eigen::Matrix<bool, 1, Eigen::Dynamic>::Constant(1, cloud.getNbPoints(), true);
	int lastNbPoints = -1, currentNbPoints = cloud.getNbPoints();
	while(currentNbPoints != lastNbPoints)
	{
		lastNbPoints = currentNbPoints;
		for(int i = 0; i < cloud.getNbPoints(); ++i)
		{
			if(!masks(0, i))
			{
				continue;
			}

			Distribution<T> neighborhoodDistribution = distributions[matches.ids(0, i)];
			for(int j = 1; j < knn; ++j)
			{
				if(!masks(0, matches.ids(j, i)))
				{
					continue;
				}
				neighborhoodDistribution = neighborhoodDistribution.combine(distributions[matches.ids(j, i)]);
			}
			typename PM::Vector delta = neighborhoodDistribution.getMean() - cloud.features.col(i).topRows(nbDim);
			float mahalanobisDistance = std::sqrt(delta.transpose() * distributions[i].getCovariance() * delta);

			if(mahalanobisDistance <= maxDeviation)
			{
				cloud.features.col(i).topRows(nbDim) = neighborhoodDistribution.getMean();
				distributions[i] = neighborhoodDistribution;
				for(int j = 1; j < knn; ++j)
				{
					if(masks(0, matches.ids(j, i)))
					{
						masks(0, matches.ids(j, i)) = false;
						--currentNbPoints;
					}
				}
			}
		}
	}

	if(!cloud.descriptorExists("covariance"))
	{
		cloud.addDescriptor("covariance", PM::Matrix::Zero(std::pow(nbDim, 2), cloud.getNbPoints()));
	}
	if(!cloud.descriptorExists("weightSum"))
	{
		cloud.addDescriptor("weightSum", PM::Matrix::Zero(std::pow(nbDim, 2), cloud.getNbPoints()));
	}

	auto covarianceVectors = cloud.getDescriptorViewByName("covariance");
	auto weightSumVectors = cloud.getDescriptorViewByName("weightSum");
	int nbPoints = 0;
	for(int i = 0; i < cloud.getNbPoints(); ++i)
	{
		if(masks(0, i))
		{
			typename PM::Matrix covariance = distributions[i].getCovariance();
			typename PM::Matrix weightSum = distributions[i].getWeightSum();
			for(int j = 0; j < nbDim; ++j)
			{
				covarianceVectors.block(j * nbDim, i, nbDim, 1) = covariance.col(j);
				weightSumVectors.block(j * nbDim, i, nbDim, 1) = weightSum.col(j);
			}

			cloud.setColFrom(nbPoints++, cloud, i);
		}
	}
	cloud.conservativeResize(nbPoints);
}

template
struct CompressionDataPointsFilter<float>;
template
struct CompressionDataPointsFilter<double>;

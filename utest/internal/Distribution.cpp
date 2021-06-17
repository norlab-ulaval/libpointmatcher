#include "utest/utest.h"
#include "DataPointsFilters/utils/Distribution.h"

const int NB_RANDOM_TESTS = 100;
const int NB_POINTS = 100;
const float POINT_MAX_DIST = 100.0;
const float EPSILON = 1e-3;

PM::DataPoints generateRandomDataPoints(bool is3D)
{
	srand((unsigned int)time(0));

	PM::Matrix randFeat = POINT_MAX_DIST * PM::Matrix::Random(is3D ? 4 : 3, NB_POINTS);
	PM::DataPoints::Labels featLabels;
	featLabels.push_back(PM::DataPoints::Label("x", 1));
	featLabels.push_back(PM::DataPoints::Label("y", 1));
	if(is3D)
	{
		featLabels.push_back(PM::DataPoints::Label("z", 1));
	}
	featLabels.push_back(PM::DataPoints::Label("pad", 1));

	PM::DataPoints pointCloud = PM::DataPoints(randFeat, featLabels);

	return pointCloud;
}

template<typename T>
Distribution<T> computeDistributionSequentially(const PM::Matrix& points)
{
	Distribution<float> distribution(points.col(0));
	for(int j = 1; j < points.cols(); j++)
	{
		distribution = distribution.combine(Distribution<float>(points.col(j)));
	}
	return distribution;
}

template<typename T>
Distribution<T> computeDistributionBySubsets(const PM::Matrix& points)
{
	if(points.cols() == 1)
	{
		return Distribution<T>(points.col(0));
	}

	return computeDistributionBySubsets<T>(points.leftCols(points.cols() / 2)).combine(
			computeDistributionBySubsets<T>(points.rightCols(points.cols() - (points.cols() / 2))));
}

TEST(DistributionTest, MeanComputedSequentially2D)
{
	for(int i = 0; i < NB_RANDOM_TESTS; i++)
	{
		PM::Matrix points = generateRandomDataPoints(false).features.topRows(2);

		Distribution<float> distribution = computeDistributionSequentially<float>(points);

		ASSERT_LE((points.rowwise().mean() - distribution.getMean()).mean(), EPSILON);
	}
}

TEST(DistributionTest, CovarianceComputedSequentially2D)
{
	for(int i = 0; i < NB_RANDOM_TESTS; i++)
	{
		PM::Matrix points = generateRandomDataPoints(false).features.topRows(2);

		Distribution<float> distribution = computeDistributionSequentially<float>(points);

		PM::Matrix computedCovariance = distribution.getCovariance();

		const PM::Matrix deviations = points.colwise() - points.rowwise().mean();
		const PM::Matrix expectedCovariance((1.0 / points.cols()) * deviations * deviations.transpose());

		ASSERT_LE((expectedCovariance - computedCovariance).mean(), EPSILON);
	}
}

TEST(DistributionTest, MeanComputedBySubsets2D)
{
	for(int i = 0; i < NB_RANDOM_TESTS; i++)
	{
		PM::Matrix points = generateRandomDataPoints(false).features.topRows(2);

		Distribution<float> distribution = computeDistributionBySubsets<float>(points);

		ASSERT_LE((points.rowwise().mean() - distribution.getMean()).mean(), EPSILON);
	}
}

TEST(DistributionTest, CovarianceComputedBySubsets2D)
{
	for(int i = 0; i < NB_RANDOM_TESTS; i++)
	{
		PM::Matrix points = generateRandomDataPoints(false).features.topRows(2);

		Distribution<float> distribution = computeDistributionBySubsets<float>(points);

		PM::Matrix computedCovariance = distribution.getCovariance();

		const PM::Matrix deviations = points.colwise() - points.rowwise().mean();
		const PM::Matrix expectedCovariance((1.0 / points.cols()) * deviations * deviations.transpose());

		ASSERT_LE((expectedCovariance - computedCovariance).mean(), EPSILON);
	}
}

TEST(DistributionTest, MeanComputedSequentially3D)
{
	for(int i = 0; i < NB_RANDOM_TESTS; i++)
	{
		PM::Matrix points = generateRandomDataPoints(true).features.topRows(3);

		Distribution<float> distribution = computeDistributionSequentially<float>(points);

		ASSERT_LE((points.rowwise().mean() - distribution.getMean()).mean(), EPSILON);
	}
}

TEST(DistributionTest, CovarianceComputedSequentially3D)
{
	for(int i = 0; i < NB_RANDOM_TESTS; i++)
	{
		PM::Matrix points = generateRandomDataPoints(true).features.topRows(3);

		Distribution<float> distribution = computeDistributionSequentially<float>(points);

		PM::Matrix computedCovariance = distribution.getCovariance();

		const PM::Matrix deviations = points.colwise() - points.rowwise().mean();
		const PM::Matrix expectedCovariance((1.0 / points.cols()) * deviations * deviations.transpose());

		ASSERT_LE((expectedCovariance - computedCovariance).mean(), EPSILON);
	}
}

TEST(DistributionTest, MeanComputedBySubsets3D)
{
	for(int i = 0; i < NB_RANDOM_TESTS; i++)
	{
		PM::Matrix points = generateRandomDataPoints(true).features.topRows(3);

		Distribution<float> distribution = computeDistributionBySubsets<float>(points);

		ASSERT_LE((points.rowwise().mean() - distribution.getMean()).mean(), EPSILON);
	}
}

TEST(DistributionTest, CovarianceComputedBySubsets3D)
{
	for(int i = 0; i < NB_RANDOM_TESTS; i++)
	{
		PM::Matrix points = generateRandomDataPoints(true).features.topRows(3);

		Distribution<float> distribution = computeDistributionBySubsets<float>(points);

		PM::Matrix computedCovariance = distribution.getCovariance();

		const PM::Matrix deviations = points.colwise() - points.rowwise().mean();
		const PM::Matrix expectedCovariance((1.0 / points.cols()) * deviations * deviations.transpose());

		ASSERT_LE((expectedCovariance - computedCovariance).mean(), EPSILON);
	}
}

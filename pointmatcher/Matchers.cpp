// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab

#include "Core.h"

// NullMatcher
template<typename T>
void MetricSpaceAligner<T>::NullMatcher::init(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	bool& iterate)
{
	
}

template<typename T>
typename MetricSpaceAligner<T>::Matches MetricSpaceAligner<T>::NullMatcher::findClosests(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	bool& iterate)
{
	return Matches();
}

template struct MetricSpaceAligner<float>::NullMatcher;
template struct MetricSpaceAligner<double>::NullMatcher;



// KDTreeMatcher
template<typename T>
MetricSpaceAligner<T>::KDTreeMatcher::KDTreeMatcher(const int knn, const double epsilon):
	knn(knn),
	epsilon(epsilon),
	featureNNS(0)
{
}

template<typename T>
MetricSpaceAligner<T>::KDTreeMatcher::~KDTreeMatcher()
{
	assert(featureNNS);
	delete featureNNS;
}

template<typename T>
void MetricSpaceAligner<T>::KDTreeMatcher::init(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	bool& iterate)
{
	// build and populate NNS
	if (featureNNS)
		delete featureNNS;
	featureNNS = NNS::create(filteredReference.features, filteredReference.features.rows() - 1);
}

template<typename T>
typename MetricSpaceAligner<T>::Matches MetricSpaceAligner<T>::KDTreeMatcher::findClosests(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	bool& iterate)
{
	
	const int pointsCount(filteredReading.features.cols());
	Matches matches(
		typename Matches::Dists(knn, pointsCount),
		typename Matches::Ids(knn, pointsCount)
	);
	
	featureNNS->knn(filteredReading.features, matches.ids, matches.dists, knn, epsilon, Nabo::NearestNeighbourSearch<T>::ALLOW_SELF_MATCH);
	
	return matches;
}

template struct MetricSpaceAligner<float>::KDTreeMatcher;
template struct MetricSpaceAligner<double>::KDTreeMatcher;

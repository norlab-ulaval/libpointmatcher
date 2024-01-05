// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab

#include "KMeansClustering.h"

template<typename T>
KMeansClusteringDataPointsFilter<T>::KMeansClusteringDataPointsFilter(const Parameters& params) :
        PointMatcher<T>::DataPointsFilter("KMeansClusteringDataPointsFilter",
                                          KMeansClusteringDataPointsFilter::availableParameters(), params),
        k(Parametrizable::get<std::size_t>("k")),
        iter(Parametrizable::get<std::size_t>("iter")),
        epsilon(Parametrizable::get<T>("epsilon")),
        seed(Parametrizable::get<int>("seed"))
{
}

// KMeansClusteringDataPointsFilter
template<typename T>
typename PointMatcher<T>::DataPoints KMeansClusteringDataPointsFilter<T>::filter(
        const DataPoints& input)
{
    DataPoints output(input);
    inPlaceFilter(output);
    return output;
}

// In-place filter
template<typename T>
void KMeansClusteringDataPointsFilter<T>::inPlaceFilter(
        DataPoints& cloud)
{
    // get k seed points for means clustering
    DataPoints seeds = getSeedPoints(cloud);
    std::cout << "Number of seed points: " << seeds.getNbPoints() << std::endl;
    // create new descriptor with the seed point index
    Vector cluster_index = Vector::Zero(cloud.getNbPoints());
    assignClusters(cloud, seeds, cluster_index);

    float clusters_epsilon = epsilon + 1.0;
    size_t ctr = 0;
    while(clusters_epsilon > epsilon && ctr < iter)
    {
        ctr++;

        // reset seed statistics
        seeds.features.setZero();
        seeds.features.row(3).setOnes();

        seeds.descriptors.setZero();

#pragma omp parallel for
        for(unsigned i = 0; i < cloud.getNbPoints(); ++i)
        {
            unsigned cluster = cluster_index(i);
            seeds.features.col(cluster) += cloud.features.col(i);
        }
        for(unsigned j = 0; j < seeds.getNbPoints(); ++j)
        {
            seeds.features.col(j) = seeds.features.col(j).colwise().hnormalized().colwise().homogeneous();
        }
        assignClusters(cloud, seeds, cluster_index);
    }
    cloud.addDescriptor("cluster_index", cluster_index.transpose());
}

template<typename T>
void KMeansClusteringDataPointsFilter<T>::assignClusters(const DataPoints& cloud, const DataPoints& seeds, Vector& cluster_index)
{
#pragma omp parallel for
    for(unsigned i = 0; i < cloud.getNbPoints(); ++i)
    {
        double min_distance = (cloud.features.col(i) - seeds.features.col(0)).norm();
        unsigned min_distance_index = 0;
        for(unsigned j = 1; j < seeds.getNbPoints(); ++j)
        {
            double dist = (cloud.features.col(i) - seeds.features.col(j)).norm();
            if(dist < min_distance)
            {
                min_distance = dist;
                min_distance_index = j;
            }
        }
        cluster_index(i) = min_distance_index;
    }
}

template<typename T>
typename PointMatcher<T>::DataPoints KMeansClusteringDataPointsFilter<T>::getSeedPoints(const DataPoints& cloud)
{
    double prob = double(k) / cloud.getNbPoints();
    std::shared_ptr<typename PointMatcher<T>::DataPointsFilter> randomSample =
            PM::get().DataPointsFilterRegistrar.create(
                    "RandomSamplingDataPointsFilter",
                    {{"prob", std::to_string(prob)},
                     {"seed", std::to_string(seed)}}
            );

    return randomSample->filter(cloud);
}

template
struct KMeansClusteringDataPointsFilter<float>;
template
struct KMeansClusteringDataPointsFilter<double>;

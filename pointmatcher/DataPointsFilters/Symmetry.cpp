//
// Created by Matěj Boxan on 2023-10-09.
//

#include "Symmetry.h"
#include "Eigen/Eigenvalues"
#include <unordered_map>

// Distribution
//Constructor
template<typename T>
Distribution<T>::Distribution(Distribution::Vector point, T omega, Distribution::Matrix33 deviation):
        point(point),
        omega(omega),
        deviation(deviation)
{}

// SymmetryDataPointsFilter
// Constructor
template<typename T>
SymmetryDataPointsFilter<T>::SymmetryDataPointsFilter(const Parameters& params):
        PointMatcher<T>::DataPointsFilter("SymmetryDataPointsFilter",
                                          SymmetryDataPointsFilter::availableParameters(), params),
        vrs(Parametrizable::get<T>("vrs")),
        vro(Parametrizable::get<T>("vro")),
        dt(Parametrizable::get<T>("dt")),
        ct(Parametrizable::get<T>("ct")),
        knn(Parametrizable::get<T>("knn"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints SymmetryDataPointsFilter<T>::filter(
        const DataPoints& input)
{
    DataPoints output(input);
    inPlaceFilter(output);
    return output;
}

// In-place filter
template<typename T>
void SymmetryDataPointsFilter<T>::inPlaceFilter(
        DataPoints& cloud)
{
    // TODO allow 2D case
    unsigned dim = 3;
    assert(cloud.getEuclideanDim() == dim);
    if(!cloud.descriptorExists("omega"))
    {

        Matrix omegas = Matrix::Zero(1, cloud.getNbPoints());
        omegas.setOnes();
        cloud.addDescriptor("omega", omegas);
    }
    if(!cloud.descriptorExists("deviation"))
    {
        Matrix deviations = Matrix::Zero(std::pow(dim, 2), cloud.getNbPoints());
        if(dim == 2)
        {
            deviations.row(0) = PM::Matrix::Constant(1, cloud.getNbPoints(), sigmaLaser);
            deviations.row(3) = PM::Matrix::Constant(1, cloud.getNbPoints(), sigmaLaser);
        }
        else
        {
            deviations.row(0) = PM::Matrix::Constant(1, cloud.getNbPoints(), sigmaLaser);
            deviations.row(4) = PM::Matrix::Constant(1, cloud.getNbPoints(), sigmaLaser);
            deviations.row(8) = PM::Matrix::Constant(1, cloud.getNbPoints(), sigmaLaser);
        }

        cloud.addDescriptor("deviation", deviations);
    }
    assert(cloud.getDescriptorDimension("omega") == 1);
    // TODO only store upper diagonal
    assert(cloud.getDescriptorDimension("deviation") == std::pow(dim, 2));
    int updated_ctr = 2;

    auto distributions = getDistributionsFromCloud(cloud);

    while(updated_ctr > 0)
    {
        updated_ctr -= 1;
        auto number_of_points_before_sampling = static_cast<float>(distributions.size());
        if(updated_ctr % 2 == 1) // symmetry sampling
        {
            symmetrySampling(distributions);
        }
        else // overlap sampling
        {
            overlapSampling(distributions);
        }
        auto number_of_points_after_sampling = static_cast<float>(distributions.size());
        if(number_of_points_after_sampling / number_of_points_before_sampling < ct)
        {
            if(updated_ctr == 0)
            {
                updated_ctr = 2;
            }
            std::cout << "Down to " << number_of_points_after_sampling << " points\n";
        }
        else
        {
            std::cout << "Almost no points removed\n";
        }
    }
    cloud = getCloudFromDistributions(distributions);
}

template<typename T>
void SymmetryDataPointsFilter<T>::symmetrySampling(
        std::vector<std::shared_ptr<Distribution<T>>>& distributions)
{
    std::cout << "Symmetry sampling" << std::endl;

    using namespace PointMatcherSupport;

    const int pointsCount(distributions.size());

    Parametrizable::Parameters param;
    boost::assign::insert(param)("knn", toParam(knn));

    auto cloud = getCloudFromDistributions(distributions);

    // Build kd-tree
    KDTreeMatcher matcher(param);
    matcher.init(cloud);

    Matches matches(typename Matches::Dists(knn, pointsCount), typename Matches::Ids(knn, pointsCount));
    matches = matcher.findClosests(cloud);

    Vector masks_all = Vector::Ones(pointsCount);

    for(int i = 0; i < pointsCount; ++i)
    {
        if(masks_all(i) == 0)
        {
            continue;
        }
        auto distro1 = distributions[i];
        for(int j = 1; j < int(knn); ++j) // index from 1 to skip self-match
        {
            if(matches.dists(j, i) == Matches::InvalidDist || matches.ids(j, i) == Matches::InvalidId)
            {
                continue;
            }

            unsigned m = matches.ids(j, i);

            if(masks_all(m) == 0)
            {
                continue;
            }
            auto distro2 = distributions[m];
            float volume2 = distro2->getVolume();

            bool was_merge = false;

            boost::optional<Distribution<T>> combined_distro;
            for(int k = j + 1; k < knn; ++k)
            {
                if(matches.dists(k, i) == Matches::InvalidDist || matches.ids(k, i) == Matches::InvalidId)
                {
                    continue;
                }
                unsigned neighbor_idx = matches.ids(k, i);
                if(masks_all(neighbor_idx) == 0)
                {
                    continue;
                }
                auto distro3 = distributions[neighbor_idx];
                auto point3 = cloud.features.col(neighbor_idx).head(3);
                auto delta = distro2->point - distro3->point;
                auto closest_point = point3 + (1. / (distro2->omega + distro3->omega) * distro2->omega * delta);
                float distance = (closest_point - distro1->point).norm();

                if(distance < dt)
                {
                    float volume3 = distro3->getVolume();
                    Distribution<T> distro_c = Distribution<T>::combineDistros(*distro2, *distro3);

                    float volume_c = distro_c.getVolume();
                    float sum_of_volumes = volume2 + volume3;
                    float ratio = volume_c / sum_of_volumes;

                    if(ratio < vrs)
                    {
                        masks_all(m) = 0;
                        masks_all(neighbor_idx) = 0;
                        was_merge = true;
                        combined_distro = distro_c;
                        break;
                    }
                }
            }
            if(combined_distro)
            {
                masks_all(i) = 2;
                Distribution<T> new_distro = Distribution<T>::combineDistros(*distro1, *combined_distro);
                distributions[i] = std::make_shared<Distribution<T>>(new_distro);
            }

            if(was_merge)
            {
                break;
            }
        }
    }

    std::vector<std::shared_ptr<Distribution<T>>> distributions_out;
    for(int i = 0; i < distributions.size(); ++i)
    {
        if (masks_all(i) != 0) {
            distributions_out.push_back(distributions[i]);
        }
    }
    distributions = distributions_out;
}

template<typename T>
void SymmetryDataPointsFilter<T>::overlapSampling(
        std::vector<std::shared_ptr<Distribution<T>>>& distributions)
{
    std::cout << "Overlap sampling" << std::endl;

    using namespace PointMatcherSupport;

    const int pointsCount(distributions.size());

    Parametrizable::Parameters param;
    boost::assign::insert(param)("knn", toParam(knn));
    auto cloud = getCloudFromDistributions(distributions);

//    auto cloud = getCloudFromDistributions(distributions);
    // Build kd-tree
    KDTreeMatcher matcher(param);
    matcher.init(cloud);

    Matches matches(typename Matches::Dists(knn, pointsCount), typename Matches::Ids(knn, pointsCount));
    matches = matcher.findClosests(cloud);

    Eigen::VectorXd masks_all = Eigen::VectorXd::Ones(pointsCount);

    for(int i = 0; i < pointsCount; ++i)
    {
        if(masks_all(i) == 0)
        {
            continue;
        }

        auto distro1 = distributions[i];

        bool was_overlap = false;
        for(int j = 1; j < int(knn); ++j) // index from 1 to skip self-match
        {
            if(matches.dists(j, i) == Matches::InvalidDist || matches.ids(j, i) == Matches::InvalidId)
            {
                continue;
            }

            unsigned m = matches.ids(j, i);

            if(masks_all(m) == 0)
            {
                continue;
            }
            auto distro2 = distributions[m];

            Distribution<T> distro_c = Distribution<T>::combineDistros(*distro1, *distro2);

            float volume_c = distro_c.getVolume();
            float sum_of_volumes = distro1->getVolume() + distro2->getVolume();
            float ratio = volume_c / sum_of_volumes;

            if(ratio < vro)
            {
                masks_all(m) = 0;
                was_overlap = true;
                distro1 = std::make_shared<Distribution<T>>(distro_c);
            }
        }

        if(was_overlap)
        {
            masks_all(i) = 2;
            distributions[i] = distro1;
        }
    }

    std::vector<std::shared_ptr<Distribution<T>>> distributions_out;
    for(int i = 0; i < distributions.size(); ++i)
    {
        if (masks_all(i) != 0) {
            distributions_out.push_back(distributions[i]);
        }
    }
    distributions = distributions_out;
}

template<typename T>
typename PointMatcher<T>::DataPoints SymmetryDataPointsFilter<T>::getCloudFromDistributions(
        std::vector<std::shared_ptr<Distribution<T>>>& distributions)
{
    DataPoints out;
    out.allocateFeature("x", 1);
    out.allocateFeature("y", 1);
    out.allocateFeature("z", 1);
    out.allocateFeature("pad", 1);

    out.conservativeResize(distributions.size());

    out.allocateDescriptor("omega", 1);
    out.allocateDescriptor("deviation", 9);
    BOOST_AUTO(omegas, out.getDescriptorViewByName("omega"));
    BOOST_AUTO(deviations, out.getDescriptorViewByName("deviation"));

    // TODO copy existing descriptors
    unsigned ctr = 0;
    out.features.row(3).setOnes();
    for(const auto &distro: distributions)
    {
        out.features.col(ctr).head(3) = (*distro).point;
        omegas(0, ctr) = (*distro).omega;
        deviations.col(ctr) = (*distro).deviation.reshaped(9, 1);
        ctr += 1;
    }
    return out;
}

template<typename T>
std::vector<std::shared_ptr<Distribution<T>>> SymmetryDataPointsFilter<T>::getDistributionsFromCloud(SymmetryDataPointsFilter::DataPoints& cloud)
{
    std::vector<std::shared_ptr<Distribution<T>>> distributions;
    distributions.reserve(cloud.getNbPoints());
    auto points = cloud.features;
    auto omegas = cloud.getDescriptorViewByName("omega");
    auto deviations = cloud.getDescriptorViewByName("deviation");
    for(int i = 0; i < cloud.getNbPoints(); ++i)
    {
        distributions.emplace_back(new Distribution<T>(points.col(i).head(3),
                                                       omegas(0, i),
                                                       deviations.block(0, i, 9, 1).reshaped(3, 3)));
    }
    return distributions;
}

template
struct SymmetryDataPointsFilter<float>;
template
struct SymmetryDataPointsFilter<double>;
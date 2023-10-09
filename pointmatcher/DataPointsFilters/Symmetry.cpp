//
// Created by Matěj Boxan on 2023-10-09.
//

#include "Symmetry.h"
#include "MatchersImpl.h"
#include "Eigen/Eigenvalues"

// Distribution
//Constructor
template<typename T>
Distribution<T>::Distribution(Distribution::Vector point, T omega, Distribution::Matrix deviation):
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
    assert(cloud.getEuclideanDim() == 3);
    assert(cloud.descriptorExists("omega"));
    assert(cloud.descriptorExists("deviation"));
    assert(cloud.getDescriptorDimension("omega") == 1);
    // TODO only store upper diagonal
    assert(cloud.getDescriptorDimension("deviation") == 9);
    int updated_ctr = 2;
    while(updated_ctr > 0)
    {
        updated_ctr -= 1;
        int number_of_points_before_sampling = cloud.getNbPoints();
        if(updated_ctr % 2 == 1) // symmetry sampling
        {
            symmetrySampling(cloud);
        }
        else // overlap sampling
        {
            overlapSampling(cloud);
        }
        int number_of_points_after_sampling = cloud.getNbPoints();
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
}

template<typename T>
void SymmetryDataPointsFilter<T>::symmetrySampling(
        DataPoints& cloud)
{
    // Force omegas and deviations to be present
	if (!cloud.descriptorExists("omega"))
	{
		throw InvalidField("SymmetryDataPointsFilter: Error, no omega found in descriptors.");
	}
	if (!cloud.descriptorExists("deviation"))
	{
		throw InvalidField("SymmetryDataPointsFilter: Error, no deviation found in descriptors.");
	}

	typedef typename MatchersImpl<T>::KDTreeMatcher KDTreeMatcher;
	typedef typename PointMatcher<T>::Matches Matches;

	using namespace PointMatcherSupport;

    const int pointsCount(cloud.getNbPoints());

	Parametrizable::Parameters param;
	boost::assign::insert(param) ( "knn", toParam(knn) );

    // Build kd-tree
	KDTreeMatcher matcher(param);
	matcher.init(cloud);

	Matches matches(typename Matches::Dists(knn, pointsCount), typename Matches::Ids(knn, pointsCount));
	matches = matcher.findClosests(cloud);

    std::unordered_map<unsigned, Distribution<T>*> distros_all;

    std::vector<Distribution<T>> distros_out;
    Eigen::VectorXd masks_all = Eigen::VectorXd::Ones(pointsCount);

    for(int i = 0; i < pointsCount; ++i)
    {
        if(masks_all(i) == 0) {
            continue;
        }
        auto it = distros_all.find(i);
        Distribution<T>* distro1 = nullptr;
        if(it == distros_all.end()) {
            auto point1 = cloud.features.col(i);
            auto omega1 = cloud.getDescriptorViewByName("omega")(0, i);
            Matrix33 deviation1 = cloud.getDescriptorRowViewByName("deviation", i).reshaped(3, 3);

            distro1 = new Distribution<T>(point1, omega1, deviation1);
            distros_all[i] = distro1;
        } else {
            distro1 = it->second;
        }
        for(int j = 1; j < int(knn); ++j) // TODO index from 1 or 0?
        {
            if(matches.dists(j, i) == Matches::InvalidDist || matches.ids(j, i) == Matches::InvalidId) // TODO add mask
            {
                continue;
            }

            unsigned m = matches.ids(j, i);

            if(masks_all(m) == 0) {
                continue;
            }

            auto it2 = distros_all.find(m);
            Distribution<T>* distro2 = nullptr;
            if(it2 == distros_all.end()) {
                auto point2 = cloud.features.col(m);
                auto omega2 = cloud.getDescriptorViewByName("omega")(0, m);
                Matrix33 deviation2 = cloud.getDescriptorRowViewByName("deviation", m).reshaped(3, 3);

                distro2 = new Distribution<T>(point2, omega2, deviation2);
                distros_all[m] = distro2;
            } else {
                distro2 = it2->second;
            }
            float volume2 = distro2->getVolume();

            bool was_merge = false;

            Distribution<T>* combined_distro = nullptr;
            for(int k = j+1; k < knn; ++k)
            {
                if(matches.dists(k, i) == Matches::InvalidDist || matches.ids(k, i) == Matches::InvalidId) // TODO add mask
                {
                    continue;
                }
                unsigned neighbor_idx = matches.ids(k, i);
                if(masks_all(neighbor_idx) == 0) {
                    continue;
                }
                auto point3 = cloud.features.col(neighbor_idx);
                auto delta = distro2->point - point3;
                auto omega3 = cloud.getDescriptorViewByName("omega")(0, neighbor_idx);
                auto closest_point = point3 + (1. / distro2->omega + omega3) * distro2->omega * delta;
                float distance = (closest_point - distro1->point).norm();

                if(distance < dt) {

                    auto it3 = distros_all.find(i);
                    Distribution<T>* distro3 = nullptr;
                    if(it3 == distros_all.end()) {
                        Matrix33 deviation3 = cloud.getDescriptorRowViewByName("deviation", neighbor_idx).reshaped(3, 3);

                        distro3 = new Distribution<T>(point3, omega3, deviation3);
                        distros_all[neighbor_idx] = distro3;
                    } else {
                        distro3 = it3->second;
                    }
                    float volume3 = distro3->getVolume();

                    Distribution<T> distro_c = Distribution<T>::combineDistros(*distro2, *distro3);

                    float volume_c = distro_c.getVolume();
                    float sum_of_volumes = volume2 + volume3;
                    float ratio = volume_c / sum_of_volumes;

                    if (ratio < vrs) {
                        masks_all(m) = 0;
                        masks_all(neighbor_idx) = 0;
                        was_merge = true;
                        combined_distro = &distro_c;
                        break;
                    }
                }
            }
            if (combined_distro != nullptr) {
                masks_all(i) = 0;
                Distribution<T> new_distro = Distribution<T>::combineDistros(*distro1, *combined_distro);
                distros_out.push_back(new_distro);
            }

            if (was_merge) {
                break;
            }
        }
    }
    unsigned unused_distros_count = masks_all.sum();
    unsigned count_points_out = unused_distros_count + distros_out.size();

    DataPoints out;
    out.allocateFeature("x", 1);
    out.allocateFeature("y", 1);
    out.allocateFeature("z", 1);
    out.allocateFeature("pad", 1);

    out.allocateDescriptor("omega", 1);
    out.allocateDescriptor("deviations", 9);

    out.conservativeResize(count_points_out);

    unsigned ctr = 0;
    for(Distribution<T> distro:distros_out)
    {
        out.features(0, ctr) = distro.point(0);
        out.features(1, ctr) = distro.point(1);
        out.features(2, ctr) = distro.point(2);
        out.features(3, ctr) = 1.0;

        out.descriptors(0, ctr) = distro.omega;
        out.descriptors.block(1, ctr, 9, 1) = distro.deviation.reshaped().transpose();
        ctr += 1;
    }

    for(int i = 0; i < pointsCount; ++i)
    {
        if(masks_all(i) == 1) {
            out.features(0, ctr) = cloud.features(0, i);
            out.features(1, ctr) = cloud.features(1, i);
            out.features(2, ctr) = cloud.features(2, i);
            out.features(3, ctr) = 1.0;

            out.descriptors(0, ctr) = cloud.descriptors(0, i);;
            out.descriptors.block(1, ctr, 9, 1) = cloud.descriptors.block(1, i, 9, 1);
            ctr += 1;
        }
    }
    cloud = out;
}

template<typename T>
void SymmetryDataPointsFilter<T>::overlapSampling(
        DataPoints& cloud)
{
    // Force omegas and deviations to be present
	if (!cloud.descriptorExists("omega"))
	{
		throw InvalidField("SymmetryDataPointsFilter: Error, no omega found in descriptors.");
	}
	if (!cloud.descriptorExists("deviation"))
	{
		throw InvalidField("SymmetryDataPointsFilter: Error, no deviation found in descriptors.");
	}
	typedef typename MatchersImpl<T>::KDTreeMatcher KDTreeMatcher;
	typedef typename PointMatcher<T>::Matches Matches;

	using namespace PointMatcherSupport;

    const int pointsCount(cloud.getNbPoints());

	Parametrizable::Parameters param;
	boost::assign::insert(param) ( "knn", toParam(knn) );

    // Build kd-tree
	KDTreeMatcher matcher(param);
	matcher.init(cloud);

	Matches matches(typename Matches::Dists(knn, pointsCount), typename Matches::Ids(knn, pointsCount));
	matches = matcher.findClosests(cloud);

    std::unordered_map<unsigned, Distribution<T>*> distros_all;

    std::vector<Distribution<T>> distros_out;
    Eigen::VectorXd masks_all = Eigen::VectorXd::Ones(pointsCount);

    for(int i = 0; i < pointsCount; ++i)
    {
        if(masks_all(i) == 0) {
            continue;
        }

        auto it = distros_all.find(i);
        Distribution<T>* distro1 = nullptr;
        if(it == distros_all.end()) {
            auto point1 = cloud.features.col(i);
            auto omega1 = cloud.getDescriptorViewByName("omega")(0, i);
            Matrix33 deviation1 = cloud.getDescriptorRowViewByName("deviation", i).reshaped(3, 3);

            distro1 = new Distribution<T>(point1, omega1, deviation1);
            distros_all[i] = distro1;
        } else {
            distro1 = it->second;
        }

        bool was_overlap = false;
        for(int j = 1; j < int(knn); ++j) // TODO index from 1 or 0?
        {
            if(matches.dists(j, i) == Matches::InvalidDist || matches.ids(j, i) == Matches::InvalidId) // TODO add mask
            {
                continue;
            }

            unsigned m = matches.ids(j, i);

            if(masks_all(m) == 0) {
                continue;
            }
            auto it2 = distros_all.find(m);
            Distribution<T>* distro2 = nullptr;
            if(it2 == distros_all.end()) {
                auto point2 = cloud.features.col(m);
                auto omega2 = cloud.getDescriptorViewByName("omega")(0, m);
                Matrix33 deviation2 = cloud.getDescriptorRowViewByName("deviation", m).reshaped(3, 3);

                distro2 = new Distribution<T>(point2, omega2, deviation2);
                distros_all[m] = distro2;
            } else {
                distro2 = it2->second;
            }

            Distribution<T> distro_c = Distribution<T>::combineDistros(*distro1, *distro2);

            float volume_c = distro_c.getVolume();
            float sum_of_volumes = distro1->getVolume() + distro2->getVolume();
            float ratio = volume_c / sum_of_volumes;

            if (ratio < vro) {
                masks_all(m) = 0;
                was_overlap = true;
                distro1 = &distro_c;
            }

            if (was_overlap) {
                masks_all(i) = 0;
                distros_out.push_back(*distro1);
            }
        }
    }
    unsigned unused_distros_count = masks_all.sum();
    unsigned count_points_out = unused_distros_count + distros_out.size();

    DataPoints out;
    out.allocateFeature("x", 1);
    out.allocateFeature("y", 1);
    out.allocateFeature("z", 1);
    out.allocateFeature("pad", 1);

    out.allocateDescriptor("omega", 1);
    out.allocateDescriptor("deviations", 9);

    out.conservativeResize(count_points_out);

    unsigned ctr = 0;
    for(Distribution<T> distro:distros_out)
    {
        out.features(0, ctr) = distro.point(0);
        out.features(1, ctr) = distro.point(1);
        out.features(2, ctr) = distro.point(2);
        out.features(3, ctr) = 1.0;

        out.descriptors(0, ctr) = distro.omega;
        out.descriptors.block(1, ctr, 9, 1) = distro.deviation.reshaped().transpose();
        ctr += 1;
    }

    for(int i = 0; i < pointsCount; ++i)
    {
        if(masks_all(i) == 1) {
            out.features(0, ctr) = cloud.features(0, i);
            out.features(1, ctr) = cloud.features(1, i);
            out.features(2, ctr) = cloud.features(2, i);
            out.features(3, ctr) = 1.0;

            out.descriptors(0, ctr) = cloud.descriptors(0, i);;
            out.descriptors.block(1, ctr, 9, 1) = cloud.descriptors.block(1, i, 9, 1);
            ctr += 1;
        }
    }
    cloud = out;
}

template
struct SymmetryDataPointsFilter<float>;
template
struct SymmetryDataPointsFilter<double>;
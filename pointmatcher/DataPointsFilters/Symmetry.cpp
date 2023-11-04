//
// Created by Matěj Boxan on 2023-10-09.
//

#include "Symmetry.h"
#include "Eigen/Eigenvalues"
#include <unordered_map>

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
    unsigned dim = 3;
    assert(cloud.getEuclideanDim() == dim);
    if(! cloud.descriptorExists("omega")) {

        Matrix omegas = Matrix::Zero(1, cloud.getNbPoints());
        omegas.setOnes();
        cloud.addDescriptor("omega", omegas);
    }
    if(!cloud.descriptorExists("deviation")) {
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
    while(updated_ctr > 0)
    {
        updated_ctr -= 1;
        auto number_of_points_before_sampling = static_cast<float>(cloud.getNbPoints());
        if(updated_ctr % 2 == 1) // symmetry sampling
        {
            symmetrySampling(cloud);
        }
        else // overlap sampling
        {
            overlapSampling(cloud);
        }
        auto number_of_points_after_sampling = static_cast<float>(cloud.getNbPoints());
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

    std::cout << "Symmetry sampling" << std::endl;

	using namespace PointMatcherSupport;

    const int pointsCount(cloud.getNbPoints());

	Parametrizable::Parameters param;
	boost::assign::insert(param) ( "knn", toParam(knn) );

    // Build kd-tree
	KDTreeMatcher matcher(param);
	matcher.init(cloud);

	Matches matches(typename Matches::Dists(knn, pointsCount), typename Matches::Ids(knn, pointsCount));
	matches = matcher.findClosests(cloud);

    std::unordered_map<unsigned, std::shared_ptr<Distribution<T>>> distros_all;

    std::vector<Distribution<T>> distros_out;
    Eigen::VectorXd masks_all = Eigen::VectorXd::Ones(pointsCount);

    for(int i = 0; i < pointsCount; ++i)
    {
        if(masks_all(i) == 0) {
            continue;
        }
        auto it = distros_all.find(i);
        std::shared_ptr<Distribution<T>> distro1;
        if(it == distros_all.end()) {
            auto point1 = cloud.features.col(i).head(3);
            auto omega1 = cloud.getDescriptorViewByName("omega")(0, i);
            Matrix33 deviation1 = cloud.getDescriptorViewByName("deviation").block(0, i, 9, 1).reshaped(3, 3);

            distro1 = std::make_shared<Distribution<T>>(Distribution<T>(point1, omega1, deviation1));
            distros_all[i] = distro1;
        } else {
            distro1 = it->second;
        }
        for(int j = 1; j < int(knn); ++j) // index from 1 to skip self-match
        {
            if(matches.dists(j, i) == Matches::InvalidDist || matches.ids(j, i) == Matches::InvalidId)
            {
                continue;
            }

            unsigned m = matches.ids(j, i);

            if(masks_all(m) == 0) {
                continue;
            }

            auto it2 = distros_all.find(m);
            std::shared_ptr<Distribution<T>> distro2;
            if(it2 == distros_all.end()) {
                auto point2 = cloud.features.col(m).head(3);
                auto omega2 = cloud.getDescriptorViewByName("omega")(0, m);
                Matrix33 deviation2 = cloud.getDescriptorViewByName("deviation").block(0, m, 9, 1).reshaped(3, 3);

                distro2 = std::make_shared<Distribution<T>>(Distribution<T>(point2, omega2, deviation2));
                distros_all[m] = distro2;
            } else {
                distro2 = it2->second;
            }
            float volume2 = distro2->getVolume();

            bool was_merge = false;

            boost::optional<Distribution<T>> combined_distro;
            for(int k = j+1; k < knn; ++k)
            {
                if(matches.dists(k, i) == Matches::InvalidDist || matches.ids(k, i) == Matches::InvalidId)
                {
                    continue;
                }
                unsigned neighbor_idx = matches.ids(k, i);
                if(masks_all(neighbor_idx) == 0) {
                    continue;
                }
                auto point3 = cloud.features.col(neighbor_idx).head(3);
                auto delta = distro2->point - point3;
                auto omega3 = cloud.getDescriptorViewByName("omega")(0, neighbor_idx);
                auto closest_point = point3 + (1. / (distro2->omega + omega3) * distro2->omega * delta);
                float distance = (closest_point - distro1->point).norm();

                if(distance < dt) {

                    auto it3 = distros_all.find(neighbor_idx);
                    std::shared_ptr<Distribution<T>> distro3;
                    if(it3 == distros_all.end()) {
                        Matrix33 deviation3 = cloud.getDescriptorViewByName("deviation").block(0, neighbor_idx, 9, 1).reshaped(3, 3);

                        distro3 = std::make_shared<Distribution<T>>(Distribution<T>(point3, omega3, deviation3));
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
                        combined_distro = distro_c;
                        break;
                    }
                }
            }
            if (combined_distro) {
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

    out.conservativeResize(count_points_out);

    out.allocateDescriptor("omega", 1);
    out.allocateDescriptor("deviation", 9);
	BOOST_AUTO(omegas, out.getDescriptorViewByName("omega"));
	BOOST_AUTO(deviations, out.getDescriptorViewByName("deviation"));

    // TODO copy existing descriptors
    unsigned ctr = 0;
    out.features.row(3).setOnes();
    for(const Distribution<T>& distro:distros_out)
    {
        out.features.col(ctr).head(3) = distro.point;
        omegas(0, ctr) = distro.omega;
        deviations.col(ctr) = distro.deviation;
        ctr += 1;
    }

    for(int i = 0; i < pointsCount; ++i)
    {
        if(masks_all(i) == 1) {
            out.features.col(ctr) = cloud.features.col(i);
            out.descriptors.col(ctr) = cloud.descriptors.col(i);
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
    std::cout << "Overlap sampling" << std::endl;

	using namespace PointMatcherSupport;

    const int pointsCount(cloud.getNbPoints());

	Parametrizable::Parameters param;
	boost::assign::insert(param) ( "knn", toParam(knn) );

    // Build kd-tree
	KDTreeMatcher matcher(param);
	matcher.init(cloud);

	Matches matches(typename Matches::Dists(knn, pointsCount), typename Matches::Ids(knn, pointsCount));
	matches = matcher.findClosests(cloud);

    std::unordered_map<unsigned, std::shared_ptr<Distribution<T>>> distros_all;

    std::vector<Distribution<T>> distros_out;
    Eigen::VectorXd masks_all = Eigen::VectorXd::Ones(pointsCount);

    for(int i = 0; i < pointsCount; ++i)
    {
        if(masks_all(i) == 0) {
            continue;
        }

        auto it = distros_all.find(i);
        std::shared_ptr<Distribution<T>> distro1;
        if(it == distros_all.end()) {
            auto point1 = cloud.features.col(i).head(3);
            auto omega1 = cloud.getDescriptorViewByName("omega")(0, i);
            Matrix33 deviation1 = cloud.getDescriptorViewByName("deviation").block(0, i, 9, 1).reshaped(3, 3);

            distro1 = std::make_shared<Distribution<T>>(Distribution<T>(point1, omega1, deviation1));
            distros_all[i] = distro1;
        } else {
            distro1 = it->second;
        }

        bool was_overlap = false;
        for(int j = 1; j < int(knn); ++j) // index from 1 to skip self-match
        {
            if(matches.dists(j, i) == Matches::InvalidDist || matches.ids(j, i) == Matches::InvalidId)
            {
                continue;
            }

            unsigned m = matches.ids(j, i);

            if(masks_all(m) == 0) {
                continue;
            }
            auto it2 = distros_all.find(m);
            std::shared_ptr<Distribution<T>> distro2;
            if(it2 == distros_all.end()) {
                auto point2 = cloud.features.col(m).head(3);
                auto omega2 = cloud.getDescriptorViewByName("omega")(0, m);
                Matrix33 deviation2 = cloud.getDescriptorViewByName("deviation").block(0, m, 9, 1).reshaped(3, 3);

                distro2 = std::make_shared<Distribution<T>>(Distribution<T>(point2, omega2, deviation2));
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
                distro1 = std::make_shared<Distribution<T>>(distro_c);
            }
        }

        if (was_overlap) {
            masks_all(i) = 0;
            distros_out.push_back(*distro1);
        }
    }
    unsigned unused_distros_count = masks_all.sum();
    unsigned count_points_out = unused_distros_count + distros_out.size();

    DataPoints out;
    out.allocateFeature("x", 1);
    out.allocateFeature("y", 1);
    out.allocateFeature("z", 1);
    out.allocateFeature("pad", 1);

    out.conservativeResize(count_points_out);

    out.allocateDescriptor("omega", 1);
    out.allocateDescriptor("deviation", 9);
	BOOST_AUTO(omegas, out.getDescriptorViewByName("omega"));
	BOOST_AUTO(deviations, out.getDescriptorViewByName("deviation"));
    // TODO copy existing descriptors

    unsigned ctr = 0;
    out.features.row(3).setOnes();
    for(const Distribution<T>& distro:distros_out)
    {
        out.features.col(ctr).head(3) = distro.point;
        omegas(0, ctr) = distro.omega;
        deviations.col(ctr) = distro.deviation;
        ctr += 1;
    }

    for(int i = 0; i < pointsCount; ++i)
    {
        if(masks_all(i) == 1) {
            out.features.col(ctr) = cloud.features.col(i);
            out.descriptors.col(ctr) = cloud.descriptors.col(i);
            ctr += 1;
        }
    }
    cloud = out;
}

template
struct SymmetryDataPointsFilter<float>;
template
struct SymmetryDataPointsFilter<double>;
#include "VoxelHashMap.h"
#include <unordered_map>
#include <cassert>




template <typename T>
VoxelHashMapDataPointsFilter<T>::VoxelHashMapDataPointsFilter(const Parameters& params) :
	voxelSize(Parametrizable::get<T>("voxelSize")),
	pointsPerVoxel(Parametrizable::get<size_t>("pointsPerVoxel"))
{
}

template <typename T>
typename PointMatcher<T>::DataPoints
VoxelHashMapDataPointsFilter<T>::filter(const DataPoints& input) {
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

template <typename T>
void VoxelHashMapDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud) {
	std::unordered_map<Voxel, std::vector<Index>> hashMap;


	const int featDim(cloud.features.rows());
	assert(featDim == 3 || featDim == 4);

	int insertedPointsCount = 0;

    for (int i = 0; i < cloud.getNbPoints(); ++i) {
        Voxel voxel = PointToVoxel(cloud.features.col(i), voxelSize);
        auto search = hashMap.find(voxel);

        if (search != hashMap.end()) {
            std::vector<Index>& voxel_points = search->second;
            if (voxel_points.size() < pointsPerVoxel) {
                voxel_points.emplace_back(i);

                cloud.setColFrom(insertedPointsCount, cloud, i);
                insertedPointsCount++;
            }
        } else {
            std::vector<Index> voxel_points;
            voxel_points.reserve(pointsPerVoxel);
            voxel_points.emplace_back(i);
            hashMap.insert({voxel, std::move(voxel_points)});

            cloud.setColFrom(insertedPointsCount, cloud, i);
            insertedPointsCount++;
        }
    }

    cloud.conservativeResize(insertedPointsCount);
}


template struct VoxelHashMapDataPointsFilter<float>;
template struct VoxelHashMapDataPointsFilter<double>;
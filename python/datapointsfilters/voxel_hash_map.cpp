#include "voxel_hash_map.h"

#include "DataPointsFilters/VoxelHashMap.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindVoxelHashMap(py::module& p_module)
		{
			using VoxelHashMapDataPointsFilter = VoxelHashMapDataPointsFilter<ScalarType>;
			py::class_<VoxelHashMapDataPointsFilter, std::shared_ptr<VoxelHashMapDataPointsFilter>, DataPointsFilter>(p_module, "VoxelHashMapDataPointsFilter", "Voxel hash map data points filter")

				.def_static("description", &VoxelHashMapDataPointsFilter::description)
				.def_static("availableParameters", &VoxelHashMapDataPointsFilter::availableParameters)

				.def_readonly("voxelSize", &VoxelHashMapDataPointsFilter::voxelSize)
				.def_readonly("pointsPerVoxel", &VoxelHashMapDataPointsFilter::pointsPerVoxel)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &VoxelHashMapDataPointsFilter::filter)
				.def("inPlaceFilter", &VoxelHashMapDataPointsFilter::inPlaceFilter);
		}
	}
}

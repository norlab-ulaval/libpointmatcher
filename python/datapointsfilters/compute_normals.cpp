#include "compute_normals.h"

#include "DataPointsFilters/ComputeNormals.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindComputeNormals(py::module& p_module)
		{
			using ComputeNormalsDataPointsFilter = ComputeNormalsDataPointsFilter<ScalarType>;
			py::class_<ComputeNormalsDataPointsFilter, std::shared_ptr<ComputeNormalsDataPointsFilter>, DataPointsFilter>(p_module, "ComputeNormalsDataPointsFilter")
				.def_static("description", &ComputeNormalsDataPointsFilter::description)
				.def(py::init<>())
				.def("filter", &ComputeNormalsDataPointsFilter::filter)
				.def("inPlaceFilter", &ComputeNormalsDataPointsFilter::inPlaceFilter);
		}
	}
}

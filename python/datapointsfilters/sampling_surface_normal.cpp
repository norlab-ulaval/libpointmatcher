#include "sampling_surface_normal.h"

#include "DataPointsFilters/SamplingSurfaceNormal.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindSamplingSurfaceNormal(py::module& p_module)
		{
			using SamplingSurfaceNormalDataPointsFilter = SamplingSurfaceNormalDataPointsFilter<ScalarType>;
			py::class_<SamplingSurfaceNormalDataPointsFilter, std::shared_ptr<SamplingSurfaceNormalDataPointsFilter>, DataPointsFilter>(p_module, "SamplingSurfaceNormalDataPointsFilter")

				.def_static("description", &SamplingSurfaceNormalDataPointsFilter::description)
				.def_static("availableParameters", &SamplingSurfaceNormalDataPointsFilter::availableParameters)

				.def_readonly("ratio", &SamplingSurfaceNormalDataPointsFilter::ratio)
				.def_readonly("knn", &SamplingSurfaceNormalDataPointsFilter::knn)
				.def_readonly("samplingMethod", &SamplingSurfaceNormalDataPointsFilter::samplingMethod)
				.def_readonly("maxBoxDim", &SamplingSurfaceNormalDataPointsFilter::maxBoxDim)
				.def_readonly("averageExistingDescriptors", &SamplingSurfaceNormalDataPointsFilter::averageExistingDescriptors)
				.def_readonly("keepNormals", &SamplingSurfaceNormalDataPointsFilter::keepNormals)
				.def_readonly("keepDensities", &SamplingSurfaceNormalDataPointsFilter::keepDensities)
				.def_readonly("keepEigenValues", &SamplingSurfaceNormalDataPointsFilter::keepEigenValues)
				.def_readonly("keepEigenVectors", &SamplingSurfaceNormalDataPointsFilter::keepEigenVectors)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &SamplingSurfaceNormalDataPointsFilter::filter)
				.def("inPlaceFilter", &SamplingSurfaceNormalDataPointsFilter::inPlaceFilter);
		}
	}
}

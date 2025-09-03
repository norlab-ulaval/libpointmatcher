#include "surface_normal.h"

#include "DataPointsFilters/SurfaceNormal.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindSurfaceNormal(py::module& p_module)
		{
			using SurfaceNormalDataPointsFilter = SurfaceNormalDataPointsFilter<ScalarType>;
			py::class_<SurfaceNormalDataPointsFilter, std::shared_ptr<SurfaceNormalDataPointsFilter>, DataPointsFilter>(p_module, "SurfaceNormalDataPointsFilter")

				.def_static("description", &SurfaceNormalDataPointsFilter::description)
				.def_static("availableParameters", &SurfaceNormalDataPointsFilter::availableParameters)

				.def_readonly("knn", &SurfaceNormalDataPointsFilter::knn)
				.def_readonly("maxDist", &SurfaceNormalDataPointsFilter::maxDist)
				.def_readonly("epsilon", &SurfaceNormalDataPointsFilter::epsilon)
				.def_readonly("keepNormals", &SurfaceNormalDataPointsFilter::keepNormals)
				.def_readonly("keepDensities", &SurfaceNormalDataPointsFilter::keepDensities)
				.def_readonly("keepEigenValues", &SurfaceNormalDataPointsFilter::keepEigenValues)
				.def_readonly("keepEigenVectors", &SurfaceNormalDataPointsFilter::keepEigenVectors)
				.def_readonly("keepMatchedIds", &SurfaceNormalDataPointsFilter::keepMatchedIds)
				.def_readonly("keepMeanDist", &SurfaceNormalDataPointsFilter::keepMeanDist)
				.def_readonly("sortEigen", &SurfaceNormalDataPointsFilter::sortEigen)
				.def_readonly("smoothNormals", &SurfaceNormalDataPointsFilter::smoothNormals)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &SurfaceNormalDataPointsFilter::filter)
				.def("inPlaceFilter", &SurfaceNormalDataPointsFilter::inPlaceFilter);
		}
	}
}

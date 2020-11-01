#include "gestalt.h"

#include "DataPointsFilters/Gestalt.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindGestalt(py::module& p_module)
		{
			using GestaltDataPointsFilter = GestaltDataPointsFilter<ScalarType>;
			py::class_<GestaltDataPointsFilter, std::shared_ptr<GestaltDataPointsFilter>, DataPointsFilter>(p_module, "GestaltDataPointsFilter")
				.def_static("description", &GestaltDataPointsFilter::description)
				.def_static("availableParameters", &GestaltDataPointsFilter::availableParameters)

				.def_readonly("ratio", &GestaltDataPointsFilter::ratio)
				.def_readonly("radius", &GestaltDataPointsFilter::radius)
				.def_readonly("knn", &GestaltDataPointsFilter::knn)
				.def_readonly("vSizeX", &GestaltDataPointsFilter::vSizeX)
				.def_readonly("vSizeY", &GestaltDataPointsFilter::vSizeY)
				.def_readonly("vSizeZ", &GestaltDataPointsFilter::vSizeZ)
				.def_readonly("maxBoxDim", &GestaltDataPointsFilter::maxBoxDim)
				.def_readonly("maxTimeWindow", &GestaltDataPointsFilter::maxTimeWindow)
				.def_readonly("keepMeans", &GestaltDataPointsFilter::keepMeans)
				.def_readonly("averageExistingDescriptors", &GestaltDataPointsFilter::averageExistingDescriptors)
				.def_readonly("keepNormals", &GestaltDataPointsFilter::keepNormals)
				.def_readonly("keepEigenValues", &GestaltDataPointsFilter::keepEigenValues)
				.def_readonly("keepEigenVectors", &GestaltDataPointsFilter::keepEigenVectors)
				.def_readonly("keepCovariances", &GestaltDataPointsFilter::keepCovariances)
				.def_readonly("keepGestaltFeatures", &GestaltDataPointsFilter::keepGestaltFeatures)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &GestaltDataPointsFilter::filter, py::arg("input"))
				.def("inPlaceFilter", &GestaltDataPointsFilter::inPlaceFilter, py::arg("cloud"))
				.def("serializeGestaltMatrix", &GestaltDataPointsFilter::serializeGestaltMatrix, py::arg("gestaltFeatures"))
				.def("calculateAngles", &GestaltDataPointsFilter::calculateAngles, py::arg("points"), py::arg("keyPoint"))
				.def("calculateRadii", &GestaltDataPointsFilter::calculateRadii, py::arg("points"), py::arg("keyPoint"));
		}
	}
}

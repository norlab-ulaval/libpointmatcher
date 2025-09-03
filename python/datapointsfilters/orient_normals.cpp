#include "orient_normals.h"

#include "DataPointsFilters/OrientNormals.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindOrientNormals(py::module& p_module)
		{
			using OrientNormalsDataPointsFilter = OrientNormalsDataPointsFilter<ScalarType>;
			py::class_<OrientNormalsDataPointsFilter, std::shared_ptr<OrientNormalsDataPointsFilter>, DataPointsFilter>(p_module, "OrientNormalsDataPointsFilter", "Reorientation of normals")

				.def_static("description", &OrientNormalsDataPointsFilter::description)
				.def_static("availableParameters", &OrientNormalsDataPointsFilter::availableParameters)

				.def_readonly("towardCenter", &OrientNormalsDataPointsFilter::towardCenter)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &OrientNormalsDataPointsFilter::filter)
				.def("inPlaceFilter", &OrientNormalsDataPointsFilter::inPlaceFilter);
		}
	}
}

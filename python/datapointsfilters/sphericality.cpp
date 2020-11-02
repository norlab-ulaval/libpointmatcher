#include "sphericality.h"

#include "DataPointsFilters/Sphericality.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindSphericality(py::module& p_module)
		{
			using SphericalityDataPointsFilter = SphericalityDataPointsFilter<ScalarType>;
			py::class_<SphericalityDataPointsFilter, std::shared_ptr<SphericalityDataPointsFilter>, DataPointsFilter>(p_module, "SphericalityDataPointsFilter")

				.def_static("description", &SphericalityDataPointsFilter::description)
				.def_static("availableParameters", &SphericalityDataPointsFilter::availableParameters)

				.def_readonly("keepUnstructureness", &SphericalityDataPointsFilter::keepUnstructureness)
				.def_readonly("keepStructureness", &SphericalityDataPointsFilter::keepStructureness)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &SphericalityDataPointsFilter::filter)
				.def("inPlaceFilter", &SphericalityDataPointsFilter::inPlaceFilter);
		}
	}
}

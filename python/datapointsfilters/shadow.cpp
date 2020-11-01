#include "shadow.h"

#include "DataPointsFilters/Shadow.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindShadow(py::module& p_module)
		{
			using ShadowDataPointsFilter = ShadowDataPointsFilter<ScalarType>;
			py::class_<ShadowDataPointsFilter, std::shared_ptr<ShadowDataPointsFilter>, DataPointsFilter>(p_module, "ShadowDataPointsFilter", "Shadow filter, remove ghost points appearing on edges")

				.def_static("description", &ShadowDataPointsFilter::description)
				.def_static("availableParameters", &ShadowDataPointsFilter::availableParameters)

				.def_readonly("eps", &ShadowDataPointsFilter::eps)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &ShadowDataPointsFilter::filter)
				.def("inPlaceFilter", &ShadowDataPointsFilter::inPlaceFilter);
		}
	}
}

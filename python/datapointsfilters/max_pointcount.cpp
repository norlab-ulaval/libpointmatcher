#include "max_pointcount.h"

#include "DataPointsFilters/MaxPointCount.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindMaxPointCount(py::module& p_module)
		{
			using MaxPointCountDataPointsFilter = MaxPointCountDataPointsFilter<ScalarType>;
			py::class_<MaxPointCountDataPointsFilter, std::shared_ptr<MaxPointCountDataPointsFilter>, DataPointsFilter>(p_module, "MaxPointCountDataPointsFilter")
				.def_static("description", &MaxPointCountDataPointsFilter::description)
				.def_static("availableParameters", &MaxPointCountDataPointsFilter::availableParameters)

				.def_readonly("maxCount", &MaxPointCountDataPointsFilter::maxCount)
				.def_readonly("seed", &MaxPointCountDataPointsFilter::seed)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &MaxPointCountDataPointsFilter::filter)
				.def("inPlaceFilter", &MaxPointCountDataPointsFilter::inPlaceFilter);
		}
	}
}

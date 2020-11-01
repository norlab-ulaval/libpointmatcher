#include "max_quantile_on_axis.h"

#include "DataPointsFilters/MaxQuantileOnAxis.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindMaxQuantileOnAxis(py::module& p_module)
		{
			using MaxQuantileOnAxisDataPointsFilter = MaxQuantileOnAxisDataPointsFilter<ScalarType>;
			py::class_<MaxQuantileOnAxisDataPointsFilter, std::shared_ptr<MaxQuantileOnAxisDataPointsFilter>, DataPointsFilter>(p_module, "MaxQuantileOnAxisDataPointsFilter")
				.def_static("description", &MaxQuantileOnAxisDataPointsFilter::description)
				.def_static("availableParameters", &MaxQuantileOnAxisDataPointsFilter::availableParameters)

				.def_readonly("dim", &MaxQuantileOnAxisDataPointsFilter::dim)
				.def_readonly("ratio", &MaxQuantileOnAxisDataPointsFilter::ratio)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &MaxQuantileOnAxisDataPointsFilter::filter)
				.def("inPlaceFilter", &MaxQuantileOnAxisDataPointsFilter::inPlaceFilter);
		}
	}
}

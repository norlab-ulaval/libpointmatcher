#include "distance_limit.h"

#include "DataPointsFilters/DistanceLimit.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindDistanceLimit(py::module& p_module)
		{
			using DistanceLimitDataPointsFilter = DistanceLimitDataPointsFilter<ScalarType>;
			py::class_<DistanceLimitDataPointsFilter, std::shared_ptr<DistanceLimitDataPointsFilter>, DataPointsFilter>(p_module, "DistanceLimitDataPointsFilter")
				.def_static("description", &DistanceLimitDataPointsFilter::description)
				.def_static("availableParameters", &DistanceLimitDataPointsFilter::availableParameters)

				.def_readonly("dim", &DistanceLimitDataPointsFilter::dim)
				.def_readonly("dist", &DistanceLimitDataPointsFilter::dist)
				.def_readonly("removeInside", &DistanceLimitDataPointsFilter::removeInside)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &DistanceLimitDataPointsFilter::filter, py::arg("input"))
				.def("inPlaceFilter", &DistanceLimitDataPointsFilter::inPlaceFilter, py::arg("cloud"));
		}
	}
}

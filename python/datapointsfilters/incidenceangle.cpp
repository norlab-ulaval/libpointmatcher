#include "incidenceangle.h"

namespace pointmatcher
{
	void pybindIncidenceAngle(py::module& p_module)
	{
		using IncidenceAngleDataPointsFilter = IncidenceAngleDataPointsFilter<double>;
		py::class_<IncidenceAngleDataPointsFilter, std::shared_ptr<IncidenceAngleDataPointsFilter>, DataPointsFilter>
			(p_module, "IncidenceAngleDataPointsFilter")
			.def_static("description", &IncidenceAngleDataPointsFilter::description)

			.def(py::init<>())

			.def("filter", &IncidenceAngleDataPointsFilter::filter, py::arg("input"))
			.def("inPlaceFilter", &IncidenceAngleDataPointsFilter::inPlaceFilter, py::arg("cloud"));
	}
}
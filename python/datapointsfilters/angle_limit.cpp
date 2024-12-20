#include "angle_limit.h"

#include "DataPointsFilters/AngleLimit.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindAngleLimit(py::module& p_module)
		{
			using AngleLimitDataPointsFilter = AngleLimitDataPointsFilter<ScalarType>;
			py::class_<AngleLimitDataPointsFilter, std::shared_ptr<AngleLimitDataPointsFilter>, DataPointsFilter>(p_module, "AngleLimitDataPointsFilter")
				.def_static("description", &AngleLimitDataPointsFilter::description)
				.def_static("availableParameters", &AngleLimitDataPointsFilter::availableParameters)

				.def_readonly("phiMin", &AngleLimitDataPointsFilter::phiMin)
				.def_readonly("phiMax", &AngleLimitDataPointsFilter::phiMax)
				.def_readonly("thetaMin", &AngleLimitDataPointsFilter::thetaMin)
				.def_readonly("thetaMax", &AngleLimitDataPointsFilter::thetaMax)
				.def_readonly("removeInside", &AngleLimitDataPointsFilter::removeInside)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &AngleLimitDataPointsFilter::filter, py::arg("input"))
				.def("inPlaceFilter", &AngleLimitDataPointsFilter::inPlaceFilter, py::arg("cloud"));
		}
	}
}

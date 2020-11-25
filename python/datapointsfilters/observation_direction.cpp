#include "observation_direction.h"

#include "DataPointsFilters/ObservationDirection.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindObservationDirection(py::module& p_module)
		{
			using ObservationDirectionDataPointsFilter = ObservationDirectionDataPointsFilter<ScalarType>;
			py::class_<ObservationDirectionDataPointsFilter, std::shared_ptr<ObservationDirectionDataPointsFilter>, DataPointsFilter>(p_module, "ObservationDirectionDataPointsFilter")
				.def_static("description", &ObservationDirectionDataPointsFilter::description)
				.def_static("availableParameters", &ObservationDirectionDataPointsFilter::availableParameters)

				.def_readonly("centerX", &ObservationDirectionDataPointsFilter::centerX)
				.def_readonly("centerY", &ObservationDirectionDataPointsFilter::centerY)
				.def_readonly("centerZ", &ObservationDirectionDataPointsFilter::centerZ)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &ObservationDirectionDataPointsFilter::filter)
				.def("inPlaceFilter", &ObservationDirectionDataPointsFilter::inPlaceFilter);
		}
	}
}

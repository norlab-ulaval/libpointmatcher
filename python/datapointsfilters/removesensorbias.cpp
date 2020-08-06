#include "removesensorbias.h"

namespace pointmatcher
{
	void pybindRemoveSensorBias(py::module& p_module)
	{
		using RemoveSensorBiasDataPointsFilter = RemoveSensorBiasDataPointsFilter<double>;
		py::class_<RemoveSensorBiasDataPointsFilter, std::shared_ptr<RemoveSensorBiasDataPointsFilter>, DataPointsFilter>
			(p_module, "RemoveSensorBiasDataPointsFilter")

			.def_static("description", &RemoveSensorBiasDataPointsFilter::description)
			.def_static("availableParameters", &RemoveSensorBiasDataPointsFilter::availableParameters)

			.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

			.def("filter", &RemoveSensorBiasDataPointsFilter::filter)
			.def("inPlaceFilter", &RemoveSensorBiasDataPointsFilter::inPlaceFilter);
	}
}
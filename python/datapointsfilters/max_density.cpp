#include "max_density.h"

#include "DataPointsFilters/MaxDensity.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindMaxDensity(py::module& p_module)
		{
			using MaxDensityDataPointsFilter = MaxDensityDataPointsFilter<ScalarType>;
			py::class_<MaxDensityDataPointsFilter, std::shared_ptr<MaxDensityDataPointsFilter>, DataPointsFilter>(p_module, "MaxDensityDataPointsFilter")
				.def_static("description", &MaxDensityDataPointsFilter::description)
				.def_static("availableParameters", &MaxDensityDataPointsFilter::availableParameters)

				.def_readonly("maxDensity", &MaxDensityDataPointsFilter::maxDensity)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &MaxDensityDataPointsFilter::filter)
				.def("inPlaceFilter", &MaxDensityDataPointsFilter::inPlaceFilter);
		}
	}
}

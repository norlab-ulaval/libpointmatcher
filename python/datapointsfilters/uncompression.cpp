#include "uncompression.h"

#include "DataPointsFilters/Uncompression.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindUncompression(py::module& p_module)
		{
			using UncompressionDataPointsFilter = UncompressionDataPointsFilter<ScalarType>;
			py::class_<UncompressionDataPointsFilter, std::shared_ptr<UncompressionDataPointsFilter>, DataPointsFilter>(p_module, "UncompressionDataPointsFilter")
					.def_static("description", &UncompressionDataPointsFilter::description)
					.def_static("availableParameters", &UncompressionDataPointsFilter::availableParameters)

					.def_readonly("seed", &UncompressionDataPointsFilter::seed)

					.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

					.def("filter", &UncompressionDataPointsFilter::filter, py::arg("input"))
					.def("inPlaceFilter", &UncompressionDataPointsFilter::inPlaceFilter, py::arg("cloud"));
		}
	}
}

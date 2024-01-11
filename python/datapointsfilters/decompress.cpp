#include "decompress.h"

#include "DataPointsFilters/Decompress.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindDecompress(py::module& p_module)
		{
			using DecompressDataPointsFilter = DecompressDataPointsFilter<ScalarType>;
			py::class_<DecompressDataPointsFilter, std::shared_ptr<DecompressDataPointsFilter>, DataPointsFilter>(p_module, "DecompressDataPointsFilter")
				.def_static("description", &DecompressDataPointsFilter::description)
				.def_static("availableParameters", &DecompressDataPointsFilter::availableParameters)

				.def_readonly("seed", &DecompressDataPointsFilter::seed)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &DecompressDataPointsFilter::filter, py::arg("input"))
				.def("inPlaceFilter", &DecompressDataPointsFilter::inPlaceFilter, py::arg("cloud"));
		}
	}
}

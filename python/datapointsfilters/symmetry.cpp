#include "symmetry.h"

#include "DataPointsFilters/Symmetry.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindSymmetry(py::module& p_module)
		{
			using SymmetryDataPointsFilter = SymmetryDataPointsFilter<ScalarType>;
			py::class_<SymmetryDataPointsFilter, std::shared_ptr<SymmetryDataPointsFilter>, DataPointsFilter>(p_module, "SymmetryDataPointsFilter")

				.def_static("description", &SymmetryDataPointsFilter::description)
				.def_static("availableParameters", &SymmetryDataPointsFilter::availableParameters)

				.def_readonly("vrs", &SymmetryDataPointsFilter::vrs)
				.def_readonly("vro", &SymmetryDataPointsFilter::vro)
				.def_readonly("dr", &SymmetryDataPointsFilter::dr)
				.def_readonly("ct", &SymmetryDataPointsFilter::ct)
				.def_readonly("knn", &SymmetryDataPointsFilter::knn)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &SymmetryDataPointsFilter::filter)
				.def("inPlaceFilter", &SymmetryDataPointsFilter::inPlaceFilter);
		}
	}
}

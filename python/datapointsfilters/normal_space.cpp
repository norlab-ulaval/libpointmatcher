#include "normal_space.h"

#include "DataPointsFilters/NormalSpace.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindNormalSpace(py::module& p_module)
		{
			using NormalSpaceDataPointsFilter = NormalSpaceDataPointsFilter<ScalarType>;
			py::class_<NormalSpaceDataPointsFilter, std::shared_ptr<NormalSpaceDataPointsFilter>, DataPointsFilter>(p_module, "NormalSpaceDataPointsFilter")
				.def_static("description", &NormalSpaceDataPointsFilter::description)
				.def_static("availableParameters", &NormalSpaceDataPointsFilter::availableParameters)

				.def_readonly("nbSample", &NormalSpaceDataPointsFilter::nbSample)
				.def_readonly("seed", &NormalSpaceDataPointsFilter::seed)
				.def_readonly("epsilon", &NormalSpaceDataPointsFilter::epsilon)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &NormalSpaceDataPointsFilter::filter)
				.def("inPlaceFilter", &NormalSpaceDataPointsFilter::inPlaceFilter);
		}
	}
}

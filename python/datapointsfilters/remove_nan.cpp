#include "remove_nan.h"

#include "DataPointsFilters/RemoveNaN.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindRemoveNaN(py::module& p_module)
		{
			using RemoveNaNDataPointsFilter = RemoveNaNDataPointsFilter<ScalarType>;
			py::class_<RemoveNaNDataPointsFilter, std::shared_ptr<RemoveNaNDataPointsFilter>, DataPointsFilter>(p_module, "RemoveNaNDataPointsFilter", "Remove points having NaN as coordinate")

				.def_static("description", &RemoveNaNDataPointsFilter::description)

				.def(py::init<>())

				.def("filter", &RemoveNaNDataPointsFilter::filter)
				.def("inPlaceFilter", &RemoveNaNDataPointsFilter::inPlaceFilter);
		}
	}
}

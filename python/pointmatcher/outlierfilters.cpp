#include "outlierfilters.h"

namespace pointmatcher
{
	void pybindOutlierFilters(py::class_<PM>& p_class)
	{
		py::bind_vector<std::vector<std::shared_ptr<OutlierFilter>>>(p_class, "OutlierFiltersVector");

		py::class_<OutlierFilters, std::vector<std::shared_ptr<OutlierFilter>>>(p_class, "OutlierFilters")
		    .def(py::init<>())
			.def("compute", &OutlierFilters::compute, py::arg("filteredReading"), py::arg("filteredReference"), py::arg("input"));
	}
}


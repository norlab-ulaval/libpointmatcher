#include "outlier_filters.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindOutlierFilters(py::class_<PM>& p_class)
		{
			py::bind_vector<OutlierFilters>(p_class, "OutlierFilters", "A chain of OutlierFilter").def(py::init<>())
				.def("compute", &OutlierFilters::compute, py::arg("filteredReading"), py::arg("filteredReference"), py::arg("input"), "Apply outlier-detection chain");
		}
	}
}


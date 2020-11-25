#include "outlier_filter.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindOutlierFilter(py::class_<PM>& p_class)
		{
			py::class_<OutlierFilter, std::shared_ptr<OutlierFilter>, Parametrizable> pyOutlierFilter(p_class, "OutlierFilter");

			pyOutlierFilter.doc() = R"pbdoc(
An outlier filter removes or weights links between points in reading and their matched points in reference, depending on some criteria.

Criteria can be a fixed maximum authorized distance, a factor of the median distance, etc.
Points with zero weights are ignored in the subsequent minimization step.
)pbdoc";
		}
	}
}

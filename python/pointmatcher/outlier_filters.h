#ifndef PYTHON_POINTMATCHER_OUTLIER_FILTERS_H
#define PYTHON_POINTMATCHER_OUTLIER_FILTERS_H

#include "pypoint_matcher_helper.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindOutlierFilters(py::class_<PM>& p_class);
	}
}

#endif //PYTHON_POINTMATCHER_OUTLIER_FILTERS_H

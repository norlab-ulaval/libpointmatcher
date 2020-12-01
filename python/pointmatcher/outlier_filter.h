#ifndef PYTHON_POINTMATCHER_OUTLIER_FILTER_H
#define PYTHON_POINTMATCHER_OUTLIER_FILTER_H

#include "pypoint_matcher_helper.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindOutlierFilter(py::class_<PM>& p_class);
	}
}

#endif //PYTHON_POINTMATCHER_OUTLIER_FILTER_H

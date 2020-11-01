#ifndef PYTHON_MODULES_DATA_POINTS_FILTERS_MODULE_H
#define PYTHON_MODULES_DATAPOINTSFILTERS_MODULE_H

#include "pypoint_matcher_helper.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindDataPointsFilters(py::class_<PM>& p_class);
	}
}

#endif //PYTHON_MODULES_DATA_POINTS_FILTERS_MODULE_H

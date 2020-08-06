#ifndef PYTHON_MODULES_DATAPOINTSFILTERS_MODULE_H
#define PYTHON_MODULES_DATAPOINTSFILTERS_MODULE_H

#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindDataPointsFilters(py::class_<PM>& p_class);
}

#endif //PYTHON_MODULES_DATAPOINTSFILTERS_MODULE_H

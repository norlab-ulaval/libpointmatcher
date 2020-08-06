#ifndef PYTHON_POINTMATCHER_OUTLIERFILTERS_H
#define PYTHON_POINTMATCHER_OUTLIERFILTERS_H

#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindOutlierFilters(py::class_<PM>& p_class);
}

#endif //PYTHON_POINTMATCHER_OUTLIERFILTERS_H

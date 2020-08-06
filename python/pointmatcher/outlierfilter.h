#ifndef PYTHON_POINTMATCHER_OUTLIERFILTER_H
#define PYTHON_POINTMATCHER_OUTLIERFILTER_H

#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindOutlierFilter(py::class_<PM>& p_class);
}

#endif //PYTHON_POINTMATCHER_OUTLIERFILTER_H

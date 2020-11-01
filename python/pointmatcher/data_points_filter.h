#ifndef PYTHON_POINTMATCHER_DATA_POINTS_FILTER_H
#define PYTHON_POINTMATCHER_DATA_POINTS_FILTER_H

#include "pypoint_matcher_helper.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindDataPointsFilter(py::class_<PM>& p_class);
	}
}

#endif //PYTHON_POINTMATCHER_DATA_POINTS_FILTER_H

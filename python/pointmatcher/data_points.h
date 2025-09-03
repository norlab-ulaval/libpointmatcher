#ifndef PYTHON_POINTMATCHER_DATA_POINTS_H
#define PYTHON_POINTMATCHER_DATA_POINTS_H

#include "pypoint_matcher_helper.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindDataPoints(py::class_<PM>& p_class);
	}
}

#endif //PYTHON_POINTMATCHER_DATA_POINTS_H

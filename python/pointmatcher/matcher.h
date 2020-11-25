#ifndef PYTHON_POINTMATCHER_MATCHER_H
#define PYTHON_POINTMATCHER_MATCHER_H

#include "pypoint_matcher_helper.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindMatcher(py::class_<PM>& p_class);
	}
}

#endif //PYTHON_POINTMATCHER_MATCHER_H

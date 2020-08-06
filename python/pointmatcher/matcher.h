#ifndef PYTHON_POINTMATCHER_MATCHER_H
#define PYTHON_POINTMATCHER_MATCHER_H

#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindMatcher(py::class_<PM>& p_class);
}

#endif //PYTHON_POINTMATCHER_MATCHER_H

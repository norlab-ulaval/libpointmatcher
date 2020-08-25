#ifndef PYTHON_POINTMATCHER_ERROR_MINIMIZER_H
#define PYTHON_POINTMATCHER_ERROR_MINIMIZER_H

#include "pypoint_matcher_helper.h"

namespace pointmatcher
{
	void pybindErrorMinimizer(py::class_<PM>& p_class);
}

#endif //PYTHON_POINTMATCHER_ERROR_MINIMIZER_H

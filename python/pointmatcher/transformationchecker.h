#ifndef PYTHON_POINTMATCHER_TRANSFORMATIONCHECKER_H
#define PYTHON_POINTMATCHER_TRANSFORMATIONCHECKER_H

#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindTransformationChecker(py::class_<PM>& p_class);
}

#endif //PYTHON_POINTMATCHER_TRANSFORMATIONCHECKER_H

#ifndef PYTHON_POINTMATCHER_TRANSFORMATION_CHECKER_H
#define PYTHON_POINTMATCHER_TRANSFORMATION_CHECKER_H

#include "pypoint_matcher_helper.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindTransformationChecker(py::class_<PM>& p_class);
	}
}

#endif //PYTHON_POINTMATCHER_TRANSFORMATION_CHECKER_H

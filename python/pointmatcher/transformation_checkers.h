#ifndef PYTHON_POINTMATCHER_TRANSFORMATION_CHECKERS_H
#define PYTHON_POINTMATCHER_TRANSFORMATION_CHECKERS_H

#include "pypoint_matcher_helper.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindTransformationCheckers(py::class_<PM>& p_class);
	}
}

#endif //PYTHON_POINTMATCHER_TRANSFORMATION_CHECKERS_H

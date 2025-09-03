#ifndef PYTHON_POINTMATCHER_TRANSFORMATION_H
#define PYTHON_POINTMATCHER_TRANSFORMATION_H

#include "pypoint_matcher_helper.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindTranformation(py::class_<PM>& p_class);
	}
}

#endif //PYTHON_POINTMATCHER_TRANSFORMATION_H

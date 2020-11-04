#ifndef PYTHON_POINTMATCHER_TRANSFORMATIONS_H
#define PYTHON_POINTMATCHER_TRANSFORMATIONS_H

#include "pypoint_matcher_helper.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindTranformations(py::class_<PM>& p_class);
	}
}

#endif //PYTHON_POINTMATCHER_TRANSFORMATIONS_H

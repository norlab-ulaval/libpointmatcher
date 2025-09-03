#ifndef PYTHON_POINTMATCHER_INSPECTOR_H
#define PYTHON_POINTMATCHER_INSPECTOR_H

#include "pypoint_matcher_helper.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindInspector(py::class_<PM>& p_class);
	}
}

#endif //PYTHON_POINTMATCHER_INSPECTOR_H

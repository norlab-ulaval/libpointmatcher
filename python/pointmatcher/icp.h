#ifndef PYTHON_POINTMATCHER_ICP_H
#define PYTHON_POINTMATCHER_ICP_H

#include "pypoint_matcher_helper.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindICP(py::class_<PM>& p_class);
	}
}

#endif //PYTHON_POINTMATCHER_ICP_H

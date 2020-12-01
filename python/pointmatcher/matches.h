#ifndef PYTHON_POINTMATCHER_MATCHES_H
#define PYTHON_POINTMATCHER_MATCHES_H

#include "pypoint_matcher_helper.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindMatches(py::class_<PM>& p_class);
	}
}

#endif //PYTHON_POINTMATCHER_MATCHES_H

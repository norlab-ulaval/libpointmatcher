#ifndef PYTHON_POINTMATCHER_ICP_H
#define PYTHON_POINTMATCHER_ICP_H

#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindICP(py::class_<PM>& p_class);
}

#endif //PYTHON_POINTMATCHER_ICP_H

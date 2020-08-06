#ifndef PYTHON_ERRORMINIMIZERS_POINTTOPOINT_H
#define PYTHON_ERRORMINIMIZERS_POINTTOPOINT_H

#include "pointmatcher/ErrorMinimizersImpl.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindPointToPoint(py::module& p_module);
}

#endif //PYTHON_ERRORMINIMIZERS_POINTTOPOINT_H

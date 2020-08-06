#ifndef PYTHON_ERRORMINIMIZERS_POINTTOPLANEWITHCOV_H
#define PYTHON_ERRORMINIMIZERS_POINTTOPLANEWITHCOV_H

#include "pointmatcher/ErrorMinimizersImpl.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindPointToPlaneWithCov(py::module& p_module);
}

#endif //PYTHON_ERRORMINIMIZERS_POINTTOPLANEWITHCOV_H

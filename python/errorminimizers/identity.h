#ifndef PYTHON_ERRORMINIMIZERS_IDENTITY_H
#define PYTHON_ERRORMINIMIZERS_IDENTITY_H

#include "pointmatcher/ErrorMinimizersImpl.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindIdentityEM(py::module& p_module);
}

#endif //PYTHON_ERRORMINIMIZERS_IDENTITY_H

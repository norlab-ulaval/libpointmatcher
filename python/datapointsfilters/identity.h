#ifndef PYTHON_ERRORMINIMIZERS_IDENTITY_H
#define PYTHON_ERRORMINIMIZERS_IDENTITY_H

#include "DataPointsFilters/Identity.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindIdentityDPF(py::module& p_module);
}

#endif //PYTHON_ERRORMINIMIZERS_IDENTITY_H

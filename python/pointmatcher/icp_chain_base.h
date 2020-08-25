#ifndef PYTHON_POINTMATCHER_ICP_CHAIN_BASE_H
#define PYTHON_POINTMATCHER_ICP_CHAIN_BASE_H

#include "pypoint_matcher_helper.h"

namespace pointmatcher
{
	void pybindICPChainBase(py::class_<PM>& p_class);
}

#endif //PYTHON_POINTMATCHER_ICP_CHAIN_BASE_H

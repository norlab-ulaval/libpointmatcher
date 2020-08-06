#ifndef PYTHON_MODULES_POINTMATCHERSUPPORT_MODULE_H
#define PYTHON_MODULES_POINTMATCHERSUPPORT_MODULE_H

#include "pypointmatcher_helper.h"

namespace py = pybind11;
namespace pms = PointMatcherSupport;

namespace pointmatcher
{
	void pybindPointMatcherSupportModule(py::module& p_module);
}

#endif //PYTHON_MODULES_POINTMATCHERSUPPORT_MODULE_H

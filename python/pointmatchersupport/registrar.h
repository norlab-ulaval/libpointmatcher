#ifndef PYTHON_POINTMATCHERSUPPORT_REGISTRAR_H
#define PYTHON_POINTMATCHERSUPPORT_REGISTRAR_H

#include "pypoint_matcher_helper.h"

namespace python
{
	namespace pointmatchersupport
	{
		void pybindRegistrar(py::module& p_module);
	}
}

#endif //PYTHON_POINTMATCHERSUPPORT_REGISTRAR_H

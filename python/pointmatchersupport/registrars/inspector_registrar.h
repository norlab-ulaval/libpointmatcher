#ifndef PYTHON_POINTMATCHERSUPPORT_REGISTRARS_INSPECTOR_REGISTRAR_H
#define PYTHON_POINTMATCHERSUPPORT_REGISTRARS_INSPECTOR_REGISTRAR_H

#include "pypoint_matcher_helper.h"

namespace python
{
	namespace pointmatchersupport
	{
		namespace registrars
		{
			void pybindInspectorRegistrar(py::module& p_module);
		}
	}
}

#endif //PYTHON_POINTMATCHERSUPPORT_REGISTRARS_INSPECTOR_REGISTRAR_H

#ifndef PYTHON_DATAPOINTSFILTERS_REMOVE_NAN_H
#define PYTHON_DATAPOINTSFILTERS_REMOVE_NAN_H

#include "pypoint_matcher_helper.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindRemoveNaN(py::module& p_module);
	}
}

#endif //PYTHON_DATAPOINTSFILTERS_REMOVE_NAN_H

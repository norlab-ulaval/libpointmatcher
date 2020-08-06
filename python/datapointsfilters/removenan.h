#ifndef PYTHON_DATAPOINTSFILTERS_REMOVENAN_H
#define PYTHON_DATAPOINTSFILTERS_REMOVENAN_H

#include "DataPointsFilters/RemoveNaN.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindRemoveNaN(py::module& p_module);
}

#endif //PYTHON_DATAPOINTSFILTERS_REMOVENAN_H

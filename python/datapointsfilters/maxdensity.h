#ifndef PYTHON_DATAPOINTSFILTERS_MAXDENSITY_H
#define PYTHON_DATAPOINTSFILTERS_MAXDENSITY_H

#include "DataPointsFilters/MaxDensity.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindMaxDensity(py::module& p_module);
}

#endif //PYTHON_DATAPOINTSFILTERS_MAXDENSITY_H

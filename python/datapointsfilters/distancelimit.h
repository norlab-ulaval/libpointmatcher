#ifndef PYTHON_DATAPOINTSFILTERS_DISTANCELIMIT_H
#define PYTHON_DATAPOINTSFILTERS_DISTANCELIMIT_H

#include "DataPointsFilters/DistanceLimit.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindDistanceLimit(py::module& p_module);
}

#endif //PYTHON_DATAPOINTSFILTERS_DISTANCELIMIT_H

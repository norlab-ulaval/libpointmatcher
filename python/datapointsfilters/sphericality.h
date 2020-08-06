#ifndef PYTHON_DATAPOINTSFILTERS_SPHERICALITY_H
#define PYTHON_DATAPOINTSFILTERS_SPHERICALITY_H

#include "DataPointsFilters/Sphericality.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindSphericality(py::module& p_module);
}

#endif //PYTHON_DATAPOINTSFILTERS_SPHERICALITY_H

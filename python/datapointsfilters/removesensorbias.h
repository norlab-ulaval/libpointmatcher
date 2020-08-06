#ifndef PYTHON_DATAPOINTSFILTERS_REMOVESENSORBIAS_H
#define PYTHON_DATAPOINTSFILTERS_REMOVESENSORBIAS_H

#include "DataPointsFilters/RemoveSensorBias.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindRemoveSensorBias(py::module& p_module);
}

#endif //PYTHON_DATAPOINTSFILTERS_REMOVESENSORBIAS_H

#ifndef PYTHON_DATAPOINTSFILTERS_OBSERVATIONDIRECTION_H
#define PYTHON_DATAPOINTSFILTERS_OBSERVATIONDIRECTION_H

#include "DataPointsFilters/ObservationDirection.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindObservationDirection(py::module& p_module);
}

#endif //PYTHON_DATAPOINTSFILTERS_OBSERVATIONDIRECTION_H

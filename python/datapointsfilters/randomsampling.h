#ifndef PYTHON_DATAPOINTSFILTERS_RANDOMSAMPLING_H
#define PYTHON_DATAPOINTSFILTERS_RANDOMSAMPLING_H

#include "DataPointsFilters/RandomSampling.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindRandomSampling(py::module& p_module);
}

#endif //PYTHON_DATAPOINTSFILTERS_RANDOMSAMPLING_H

#ifndef PYTHON_DATAPOINTSFILTERS_FIXSTEPSAMPLING_H
#define PYTHON_DATAPOINTSFILTERS_FIXSTEPSAMPLING_H

#include "DataPointsFilters/FixStepSampling.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindFixStepSampling(py::module& p_module);
}

#endif //PYTHON_DATAPOINTSFILTERS_FIXSTEPSAMPLING_H

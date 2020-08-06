#ifndef PYTHON_DATAPOINTSFILTERS_CUTATDESCRIPTORTHRESHOLD_H
#define PYTHON_DATAPOINTSFILTERS_CUTATDESCRIPTORTHRESHOLD_H

#include "DataPointsFilters/CutAtDescriptorThreshold.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindCutAtDescriptorThreshold(py::module& p_module);
}

#endif //PYTHON_DATAPOINTSFILTERS_CUTATDESCRIPTORTHRESHOLD_H

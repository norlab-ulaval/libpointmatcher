#ifndef PYTHON_DATAPOINTSFILTERS_BOUNDINGBOX_H
#define PYTHON_DATAPOINTSFILTERS_BOUNDINGBOX_H

#include "DataPointsFilters/BoundingBox.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindBoundingBox(py::module& p_module);
}

#endif //PYTHON_DATAPOINTSFILTERS_BOUNDINGBOX_H

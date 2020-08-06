#ifndef PYTHON_DATAPOINTSFILTERS_ORIENTNORMALS_H
#define PYTHON_DATAPOINTSFILTERS_ORIENTNORMALS_H

#include "DataPointsFilters/OrientNormals.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindOrientNormals(py::module& p_module);
}

#endif //PYTHON_DATAPOINTSFILTERS_ORIENTNORMALS_H

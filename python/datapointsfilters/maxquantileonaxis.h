#ifndef PYTHON_DATAPOINTSFILTERS_MAXQUANTILEONAXIS_H
#define PYTHON_DATAPOINTSFILTERS_MAXQUANTILEONAXIS_H

#include "DataPointsFilters/MaxQuantileOnAxis.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindMaxQuantileOnAxis(py::module& p_module);
}

#endif //PYTHON_DATAPOINTSFILTERS_MAXQUANTILEONAXIS_H

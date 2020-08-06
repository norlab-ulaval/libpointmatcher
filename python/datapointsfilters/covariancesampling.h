#ifndef PYTHON_DATAPOINTSFILTERS_COVARIANCESAMPLING_H
#define PYTHON_DATAPOINTSFILTERS_COVARIANCESAMPLING_H

#include "DataPointsFilters/CovarianceSampling.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	void pybindCovarianceSampling(py::module& p_module);
}

#endif //PYTHON_DATAPOINTSFILTERS_COVARIANCESAMPLING_H

#include "registrar.h"
#include "registrars/datapointsfilter_registrar.h"
#include "registrars/errorminimizer_registrar.h"
#include "registrars/inspector_registrar.h"
#include "registrars/logger_registrar.h"
#include "registrars/matcher_registrar.h"
#include "registrars/outlierfilter_registrar.h"
#include "registrars/transformation_registrar.h"
#include "registrars/transformationchecker_registrar.h"

namespace pointmatcher
{
	void pybindRegistrar(py::module& p_module)
	{
		pybindDataPointsFilterRegistrar(p_module);
		pybindErrorMinimizerRegistrar(p_module);
		pybindInspectorRegistrar(p_module);
		pybindLoggerRegistrar(p_module);
		pybindOutlierFilterRegistrar(p_module);
		pybindMatcherRegistrar(p_module);
		pybindTransformationRegistrar(p_module);
		pybindTransformationCheckerRegistrar(p_module);
	}
}
#include "registrar.h"
#include "registrars/data_points_filter_registrar.h"
#include "registrars/error_minimizer_registrar.h"
#include "registrars/inspector_registrar.h"
#include "registrars/logger_registrar.h"
#include "registrars/matcher_registrar.h"
#include "registrars/outlier_filter_registrar.h"
#include "registrars/transformation_registrar.h"
#include "registrars/transformation_checker_registrar.h"

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
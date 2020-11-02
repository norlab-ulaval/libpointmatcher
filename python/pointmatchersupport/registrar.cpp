#include "registrar.h"
#include "registrars/data_points_filter_registrar.h"
#include "registrars/error_minimizer_registrar.h"
#include "registrars/inspector_registrar.h"
#include "registrars/logger_registrar.h"
#include "registrars/matcher_registrar.h"
#include "registrars/outlier_filter_registrar.h"
#include "registrars/transformation_registrar.h"
#include "registrars/transformation_checker_registrar.h"

namespace python
{
	namespace pointmatchersupport
	{
		void pybindRegistrar(py::module& p_module)
		{
			registrars::pybindDataPointsFilterRegistrar(p_module);
			registrars::pybindErrorMinimizerRegistrar(p_module);
			registrars::pybindInspectorRegistrar(p_module);
			registrars::pybindLoggerRegistrar(p_module);
			registrars::pybindOutlierFilterRegistrar(p_module);
			registrars::pybindMatcherRegistrar(p_module);
			registrars::pybindTransformationRegistrar(p_module);
			registrars::pybindTransformationCheckerRegistrar(p_module);
		}
	}
}

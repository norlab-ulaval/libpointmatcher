#include "modules/pointmatchersupport_module.h"
#include "modules/pointmatcher_module.h"
#include "modules/errorminimizers_module.h"
#include "modules/datapointsfilters_module.h"

namespace pm = pointmatcher;

PYBIND11_MODULE(pypointmatcher, module)
{
	module.doc() = "Python bindings of libpointmatcher";

	pm::pybindPointMatcherSupportModule(module);
	pm::pybindPointMatcherModule(module);
	pm::pybindErrorMinimizersModule(module);
	pm::pybindDataPointsFiltersModule(module);
}

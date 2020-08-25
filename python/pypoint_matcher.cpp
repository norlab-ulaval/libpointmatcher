#include "modules/point_matcher_support_module.h"
#include "modules/point_matcher_module.h"
#include "modules/error_minimizers_module.h"
#include "modules/data_points_filters_module.h"

namespace pm = pointmatcher;

PYBIND11_MODULE(pypointmatcher, module)
{
	module.doc() = "Python bindings of libpointmatcher";

	pm::pybindPointMatcherSupportModule(module);
	pm::pybindPointMatcherModule(module);
	pm::pybindErrorMinimizersModule(module);
	pm::pybindDataPointsFiltersModule(module);
}

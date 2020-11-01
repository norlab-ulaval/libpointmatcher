#include "modules/point_matcher_support_module.h"
#include "modules/point_matcher_module.h"
#include "modules/error_minimizers_module.h"
#include "modules/data_points_filters_module.h"

PYBIND11_MODULE(pypointmatcher, module)
{
	module.doc() = "Python bindings of libpointmatcher";

	python::modules::pybindPointMatcherSupportModule(module);
	python::modules::pybindPointMatcherModule(module);
	python::modules::pybindErrorMinimizersModule(module);
	python::modules::pybindDataPointsFiltersModule(module);
}

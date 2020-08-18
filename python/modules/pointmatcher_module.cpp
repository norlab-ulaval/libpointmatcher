#include "pointmatcher_module.h"

#include "pointmatcher/pointmatcher.h"
#include "pointmatcher/impl.h"
#include "pointmatcher/io.h"

namespace pointmatcher
{
	void pybindPointMatcherModule(py::module& p_module)
	{
		py::module pointmatcherModule = p_module.def_submodule("pointmatcher");
		pybindPointMatcher(pointmatcherModule);
		pybindIO(pointmatcherModule);
		pybindImpl(pointmatcherModule);
	}
}
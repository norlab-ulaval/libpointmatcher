#include "point_matcher_module.h"

#include "pointmatcher/point_matcher.h"
#include "pointmatcher/impl.h"
#include "pointmatcher/io.h"

namespace python
{
	namespace modules
	{
		void pybindPointMatcherModule(py::module& p_module)
		{
			py::module pointmatcherModule = p_module.def_submodule("pointmatcher");

			pointmatcher::pybindPointMatcher(pointmatcherModule);
			pointmatcher::pybindIO(pointmatcherModule);
			pointmatcher::pybindImpl(pointmatcherModule);
		}
	}
}

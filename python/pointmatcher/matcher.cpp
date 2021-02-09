#include "matcher.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindMatcher(py::class_<PM>& p_class)
		{
			py::class_<Matcher, std::shared_ptr<Matcher>, Parametrizable>(p_class, "Matcher")
				.def_readwrite("visitCounter", &Matcher::visitCounter)

				.def("resetVisitCount", &Matcher::resetVisitCount).def("getVisitCount", &Matcher::getVisitCount);
		}
	}
}

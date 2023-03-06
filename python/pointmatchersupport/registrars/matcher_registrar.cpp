#include "matcher_registrar.h"

namespace python
{
	namespace pointmatchersupport
	{
		namespace registrars
		{
			void pybindMatcherRegistrar(py::module& p_module)
			{
				py::class_<MatcherRegistrar, std::shared_ptr<MatcherRegistrar>>(p_module, "MatcherRegistrar")
					.def("getDescriptor", &MatcherRegistrar::getDescriptor, py::arg("name"), "Return a descriptor following a name, throw an exception if name is invalid")
					.def("create", &MatcherRegistrar::create, py::arg("name"), py::arg("params") = Parameters(), "Create an instance")
					.def("getDescription", &MatcherRegistrar::getDescription, py::arg("name"), "Get the description of a class")
					.def("dump", [](const MatcherRegistrar& self)
					{
						std::ostringstream oss;
						self.dump(oss);
						py::print(oss.str());
					}, "Print the list of registered classes to stream");
			}
		}
	}
}

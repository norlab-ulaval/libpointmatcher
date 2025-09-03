#include "inspector_registrar.h"

namespace python
{
	namespace pointmatchersupport
	{
		namespace registrars
		{
			void pybindInspectorRegistrar(py::module& p_module)
			{
				py::class_<InspectorRegistrar, std::shared_ptr<InspectorRegistrar>>(p_module, "InspectorRegistrar")
					.def("getDescriptor", &InspectorRegistrar::getDescriptor, py::arg("name"), "Return a descriptor following a name, throw an exception if name is invalid")
					.def("create", &InspectorRegistrar::create, py::arg("name"), py::arg("params") = Parameters(), "Create an instance")
					.def("getDescription", &InspectorRegistrar::getDescription, py::arg("name"), "Get the description of a class")
					.def("dump", [](const InspectorRegistrar& self)
					{
						std::ostringstream oss;
						self.dump(oss);
						py::print(oss.str());
					}, "Print the list of registered classes to stream");
			}
		}
	}
}

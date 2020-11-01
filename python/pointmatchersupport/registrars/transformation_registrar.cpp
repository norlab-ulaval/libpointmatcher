#include "transformation_registrar.h"

namespace python
{
	namespace pointmatchersupport
	{
		namespace registrars
		{
			void pybindTransformationRegistrar(py::module& p_module)
			{
				py::class_<TransformationRegistrar, std::shared_ptr<TransformationRegistrar>>(p_module, "TransformationRegistrar")
					.def("getDescriptor", &TransformationRegistrar::getDescriptor, py::arg("name"), "Return a descriptor following a name, throw an exception if name is invalid")
					.def("create", &TransformationRegistrar::create, py::arg("name"), py::arg("params") = Parameters(), "Create an instance")
					.def("getDescription", &TransformationRegistrar::getDescription, py::arg("name"), "Get the description of a class")
					.def("dump", [](const TransformationRegistrar& self)
					{
						std::ostringstream oss;
						self.dump(oss);
						py::print(oss.str());
					}, "Print the list of registered classes to stream");
			}
		}
	}
}

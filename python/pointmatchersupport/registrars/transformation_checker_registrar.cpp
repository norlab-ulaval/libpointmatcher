#include "transformation_checker_registrar.h"

namespace python
{
	namespace pointmatchersupport
	{
		namespace registrars
		{
			void pybindTransformationCheckerRegistrar(py::module& p_module)
			{
				py::class_<TransformationCheckerRegistrar, std::shared_ptr<TransformationCheckerRegistrar>>(p_module, "TransformationCheckerRegistrar")
					.def("getDescriptor", &TransformationCheckerRegistrar::getDescriptor, py::arg("name"), "Return a descriptor following a name, throw an exception if name is invalid")
					.def("create", &TransformationCheckerRegistrar::create, py::arg("name"), py::arg("params") = Parameters(), "Create an instance")
					.def("getDescription", &TransformationCheckerRegistrar::getDescription, py::arg("name"), "Get the description of a class")
					.def("dump", [](const TransformationCheckerRegistrar& self)
					{
						std::ostringstream oss;
						self.dump(oss);
						py::print(oss.str());
					}, "Print the list of registered classes to stream");
			}
		}
	}
}

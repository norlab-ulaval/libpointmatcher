#include "logger_registrar.h"

namespace python
{
	namespace pointmatchersupport
	{
		namespace registrars
		{
			void pybindLoggerRegistrar(py::module& p_module)
			{
				py::class_<LoggerRegistrar, std::shared_ptr<LoggerRegistrar>>(p_module, "LoggerRegistrar")
					.def("getDescriptor", &LoggerRegistrar::getDescriptor, py::arg("name"), "Return a descriptor following a name, throw an exception if name is invalid")
					.def("create", &LoggerRegistrar::create, py::arg("name"), py::arg("params") = Parameters(), "Create an instance")
					.def("getDescription", &LoggerRegistrar::getDescription, py::arg("name"), "Get the description of a class")
					.def("dump", [](const LoggerRegistrar& self)
					{
						std::ostringstream oss;
						self.dump(oss);
						py::print(oss.str());
					}, "Print the list of registered classes to stream");
			}
		}
	}
}

#include "error_minimizer_registrar.h"

namespace python
{
	namespace pointmatchersupport
	{
		namespace registrars
		{
			void pybindErrorMinimizerRegistrar(py::module& p_module)
			{
				py::class_<ErrorMinimizerRegistrar, std::shared_ptr<ErrorMinimizerRegistrar>>(p_module, "ErrorMinimizerRegistrar")
					.def("getDescriptor", &ErrorMinimizerRegistrar::getDescriptor, py::arg("name"), "Return a descriptor following a name, throw an exception if name is invalid")
					.def("create", &ErrorMinimizerRegistrar::create, py::arg("name"), py::arg("params") = Parameters(), "Create an instance")
					.def("getDescription", &ErrorMinimizerRegistrar::getDescription, py::arg("name"), "Get the description of a class")
					.def("dump", [](const ErrorMinimizerRegistrar& self)
					{
						std::ostringstream oss;
						self.dump(oss);
						py::print(oss.str());
					}, "Print the list of registered classes to stream");
			}
		}
	}
}

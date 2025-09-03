#include "outlier_filter_registrar.h"

namespace python
{
	namespace pointmatchersupport
	{
		namespace registrars
		{
			void pybindOutlierFilterRegistrar(py::module& p_module)
			{
				py::class_<OutlierFilterRegistrar, std::shared_ptr<OutlierFilterRegistrar>>(p_module, "OutlierFilterRegistrar")
					.def("getDescriptor", &OutlierFilterRegistrar::getDescriptor, py::arg("name"), "Return a descriptor following a name, throw an exception if name is invalid")
					.def("create", &OutlierFilterRegistrar::create, py::arg("name"), py::arg("params") = Parameters(), "Create an instance")
					.def("getDescription", &OutlierFilterRegistrar::getDescription, py::arg("name"), "Get the description of a class")
					.def("dump", [](const OutlierFilterRegistrar& self)
					{
						std::ostringstream oss;
						self.dump(oss);
						py::print(oss.str());
					}, "Print the list of registered classes to stream");
			}
		}
	}
}

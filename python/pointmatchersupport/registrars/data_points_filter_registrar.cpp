#include "data_points_filter_registrar.h"

namespace python
{
	namespace pointmatchersupport
	{
		namespace registrars
		{
			void pybindDataPointsFilterRegistrar(py::module& p_module)
			{
				py::class_<DataPointsFilterRegistrar, std::shared_ptr<DataPointsFilterRegistrar>>(p_module, "DataPointsFilterRegistrar")
					.def("getDescriptor", &DataPointsFilterRegistrar::getDescriptor, py::arg("name"), "Return a descriptor following a name, throw an exception if name is invalid")
					.def("create", &DataPointsFilterRegistrar::create, py::arg("name"), py::arg("params") = Parameters(), "Create an instance")
					.def("getDescription", &DataPointsFilterRegistrar::getDescription, py::arg("name"), "Get the description of a class")
					.def("dump", [](const DataPointsFilterRegistrar& self)
					{
						std::ostringstream oss;
						self.dump(oss);
						py::print(oss.str());
					}, "Print the list of registered classes to stream");
			}
		}
	}
}

#include "inspector.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindInspector(py::class_<PM>& p_class)
		{
			py::class_<Inspector, std::shared_ptr<Inspector>, Parametrizable>(p_class, "Inspector", "An inspector allows to log data at the different steps, for analysis.")
				.def(py::init<>())
				.def(py::init<const std::string&, const ParametersDoc, const Parameters&>(), py::arg("className"), py::arg("paramsDoc"), py::arg("params"))

				.def("init", &Inspector::init)

				.def("addStat", &Inspector::addStat, py::arg("name"), py::arg("data"))
				.def("dumpStats", [](Inspector& self)
				{
					std::ostringstream oss;
					self.dumpStats(oss);
					py::print(oss.str());
				}).def("dumpStatsHeader", [](Inspector& self)
				{
					std::ostringstream oss;
					self.dumpStatsHeader(oss);
					py::print(oss.str());
				})

				.def("dumpIteration", &Inspector::dumpIteration, py::arg("iterationNumber"), py::arg("parameters"), py::arg("filteredReference"), py::arg("reading"), py::arg("matches"), py::arg("outlierWeights"), py::arg("transformationCheckers"))
				.def("finish", &Inspector::finish, py::arg("iterationCount"));
		}
	}
}

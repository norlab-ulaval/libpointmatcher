#include "inspectors_impl.h"

#include "pointmatcher/InspectorsImpl.h"

namespace python
{
	namespace pointmatcher
	{
		namespace impls
		{
			void pybindInspectorsImpl(py::module& p_module)
			{
				using InspectorsImpl = InspectorsImpl<ScalarType>;
				py::class_<InspectorsImpl, std::shared_ptr<InspectorsImpl>> pyInspectorsImpl(p_module, "InspectorsImpl");

				using NullInspector = InspectorsImpl::NullInspector;
				py::class_<NullInspector, std::shared_ptr<NullInspector>, Inspector>(pyInspectorsImpl, "NullInspector", "Clearer name when no inspector is required")
					.def(py::init<>(), "This constructor is not required. It make the attribute className == \"NullInspector\" instead of 'unknown'")
					.def_static("description", &NullInspector::description);

				using PerformanceInspector = InspectorsImpl::PerformanceInspector;
				py::class_<PerformanceInspector, std::shared_ptr<PerformanceInspector>, Inspector>(pyInspectorsImpl, "PerformanceInspector")
					.def(py::init<const std::string&, const ParametersDoc, const Parameters&>(), py::arg("className"), py::arg("paramsDoc"), py::arg("params"))
					.def(py::init<const Parameters&>(), py::arg("params"))
					.def_static("description", &PerformanceInspector::description)
					.def_static("availableParameters", &PerformanceInspector::availableParameters)
					.def_readonly("baseFileName", &PerformanceInspector::baseFileName)
					.def_readonly("bDumpPerfOnExit", &PerformanceInspector::bDumpPerfOnExit)
					.def_readonly("bDumpStats", &PerformanceInspector::bDumpStats)
					.def("addStat", &PerformanceInspector::addStat).def("dumpStats", [](PerformanceInspector& self)
					{
						std::ostringstream oss;
						self.dumpStats(oss);
						py::print(oss.str());
					}).def("dumpStatsHeader", [](PerformanceInspector& self)
					{
						std::ostringstream oss;
						self.dumpStatsHeader(oss);
						py::print(oss.str());
					});

				using AbstractVTKInspector = InspectorsImpl::AbstractVTKInspector;
				py::class_<AbstractVTKInspector, std::shared_ptr<AbstractVTKInspector>, PerformanceInspector>(pyInspectorsImpl, "AbstractVTKInspector")
					.def("init", &AbstractVTKInspector::init)
					.def("dumpDataPoints", (void (AbstractVTKInspector::*)(const DataPoints&, const std::string&)) &AbstractVTKInspector::dumpDataPoints, py::arg("cloud"), py::arg("name"))
					.def("dumpMeshNodes", (void (AbstractVTKInspector::*)(const DataPoints&, const std::string&)) &AbstractVTKInspector::dumpMeshNodes, py::arg("cloud"), py::arg("name"))
					.def("dumpIteration", &AbstractVTKInspector::dumpIteration)
					.def("finish", &AbstractVTKInspector::finish);

				using VTKFileInspector = InspectorsImpl::VTKFileInspector;
				py::class_<VTKFileInspector, std::shared_ptr<VTKFileInspector>, AbstractVTKInspector>(pyInspectorsImpl, "VTKFileInspector")
					.def_static("description", &VTKFileInspector::description)
					.def_static("availableParameters", &VTKFileInspector::availableParameters)

					.def_readonly("baseFileName", &VTKFileInspector::baseFileName)
					.def_readonly("bDumpIterationInfo", &VTKFileInspector::bDumpIterationInfo)
					.def_readonly("bDumpDataLinks", &VTKFileInspector::bDumpDataLinks)
					.def_readonly("bDumpReading", &VTKFileInspector::bDumpReading)
					.def_readonly("bDumpReference", &VTKFileInspector::bDumpReference)

					.def(py::init<const Parameters&>(), py::arg("params") = Parameters())

					.def("init", &VTKFileInspector::init).def("finish", &VTKFileInspector::finish);
			}
		}
	}
}

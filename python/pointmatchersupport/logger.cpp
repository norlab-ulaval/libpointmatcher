#include "logger.h"

namespace python
{
	namespace pointmatchersupport
	{
		void pybindLogger(py::module& p_module)
		{
			py::class_<Logger, std::shared_ptr<Logger>, Parametrizable>(p_module, "Logger", "The logger interface, used to output warnings and informations")
				.def(py::init<>(), "Construct without parameter")
				.def(py::init<const std::string&, const ParametersDoc, const Parameters&>(), py::arg("className"), py::arg("paramsDoc"), py::arg("params"), "Construct with parameters")

				.def("hasInfoChannel", &Logger::hasInfoChannel, "Return whether this logger provides the info channel")
				.def("beginInfoEntry", &Logger::beginInfoEntry, py::arg("file"), py::arg("line"), py::arg("func"), "Start a new entry into the info channel")
//			.def("infoStream", &Logger::infoStream, py::return_value_policy::reference, "Return the info stream, 0 if hasInfoChannel() returns false") // FIXME
				.def("finishInfoEntry", &Logger::finishInfoEntry, "Finish the entry into the info channel")
				.def("hasWarningChannel", &Logger::hasWarningChannel, "Return whether this logger provides the warning channel")
				.def("beginWarningEntry", &Logger::beginWarningEntry, "Start a new entry into the warning channel")
//			.def("warningStream", &Logger::warningStream, py::return_value_policy::reference, "Return the warning stream, 0 if hasWarningChannel() returns false") // FIXME
				.def("finishWarningEntry", &Logger::finishWarningEntry, "Finish the entry into the warning channel");
		}
	}

}

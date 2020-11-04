#include "logger_impl.h"

#include "pointmatcher/LoggerImpl.h"

namespace python
{
	namespace pointmatchersupport
	{
		void pybindLoggerImpl(py::module& p_module)
		{
			using NullLogger = pms::NullLogger;
			py::class_<pms::NullLogger, std::shared_ptr<NullLogger>, Logger>(p_module, "NullLogger")
				.def_static("description", &NullLogger::description)

				.def(py::init<>());

			using FileLogger = pms::FileLogger;
			py::class_<FileLogger, std::shared_ptr<FileLogger>, Logger>(p_module, "FileLogger")
				.def_static("description", &FileLogger::description)
				.def_static("availableParameters", &FileLogger::availableParameters)

				.def_readonly("infoFileName", &FileLogger::infoFileName)
				.def_readonly("warningFileName", &FileLogger::warningFileName)
				.def_readonly("displayLocation", &FileLogger::displayLocation)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters())

				.def("hasInfoChannel", &FileLogger::hasInfoChannel, "Return whether this logger provides the info channel")
				.def("beginInfoEntry", &FileLogger::beginInfoEntry, py::arg("file"), py::arg("line"), py::arg("func"), "Start a new entry into the info channel")
//			.def("infoStream", &FileLogger::infoStream, py::return_value_policy::reference, "Return the info stream, 0 if hasInfoChannel() returns false") // FIXME
				.def("finishInfoEntry", &FileLogger::finishInfoEntry, "Finish the entry into the info channel")
				.def("hasWarningChannel", &FileLogger::hasWarningChannel, "Return whether this logger provides the warning channel")
				.def("beginWarningEntry", &FileLogger::beginWarningEntry, "Start a new entry into the warning channel")
//			.def("warningStream", &FileLogger::warningStream, py::return_value_policy::reference, "Return the warning stream, 0 if hasWarningChannel() returns false") // FIXME
				.def("finishWarningEntry", &FileLogger::finishWarningEntry, "Finish the entry into the warning channel");
		}
	}
}

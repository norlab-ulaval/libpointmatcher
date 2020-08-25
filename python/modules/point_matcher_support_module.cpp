#include "point_matcher_support_module.h"

#include "pointmatchersupport/bibliography.h"
#include "pointmatchersupport/logger.h"
#include "pointmatchersupport/logger_impl.h"
#include "pointmatchersupport/parametrizable.h"
#include "pointmatchersupport/registrar.h"

namespace pointmatcher
{
	void pybindPointMatcherSupportModule(py::module& p_module)
	{
		py::module pointmatchersupportModule = p_module.def_submodule("pointmatchersupport");

		using InvalidModuleType = pms::InvalidModuleType;
		py::register_exception<InvalidModuleType>(pointmatchersupportModule, "InvalidModuleType");

		using TransformationError = pms::TransformationError;
		py::register_exception<TransformationError>(pointmatchersupportModule, "TransformationError");

		using ConfigurationError = pms::ConfigurationError;
		py::register_exception<ConfigurationError>(pointmatchersupportModule, "ConfigurationError");

		using InvalidElement = pms::InvalidElement;
		py::register_exception<InvalidElement>(pointmatchersupportModule, "InvalidElement");

		pybindBibliography(pointmatchersupportModule);
		pybindParametrizable(pointmatchersupportModule);
		pybindLogger(pointmatchersupportModule);
		pybindLoggerImpl(pointmatchersupportModule);
		pybindRegistrar(pointmatchersupportModule);

		pointmatchersupportModule.def("setLogger", &pms::setLogger, py::arg("newLogger"),
									  "Set a new logger, protected by a mutex");
		pointmatchersupportModule.def("validateFile", &pms::validateFile, py::arg("fileName"),
									  "Throw a runtime_error exception if fileName cannot be opened");

		using CsvElements = pms::CsvElements;
		py::bind_map<CsvElements>(pointmatchersupportModule, "CsvElements");
	}
}
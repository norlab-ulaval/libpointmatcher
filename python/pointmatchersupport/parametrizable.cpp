#include "parametrizable.h"

namespace python
{
	namespace pointmatchersupport
	{
		void pybindParametrizable(py::module& p_module)
		{
			using Parametrizable = pms::Parametrizable;
			py::class_<Parametrizable, std::shared_ptr<Parametrizable>> pyParametrizable(p_module, "Parametrizable");

			pyParametrizable.doc() = R"pbdoc(
The superclass of classes that are constructed using generic parameters.
This class provides the parameter storage and fetching mechanism
)pbdoc";

			using InvalidParameter = Parametrizable::InvalidParameter;
			py::register_exception<InvalidParameter>(pyParametrizable, "InvalidParameter");

			py::class_<ParameterDoc>(pyParametrizable, "ParameterDoc", "The documentation of a parameter")
				.def_readwrite("name", &ParameterDoc::name, "name")
				.def_readwrite("doc", &ParameterDoc::doc, "short documentation")
				.def_readwrite("defaultValue", &ParameterDoc::defaultValue, "default value")
				.def_readwrite("minValue", &ParameterDoc::minValue, "if bounds are checked, minimum value")
				.def_readwrite("maxValue", &ParameterDoc::maxValue, "if bounds are checked, maxmimu value")
//			.def_readwrite("comp", &ParameterDoc::comp, "pointer to comparison function") // FIXME Get this error when compiling -> error: invalid conversion from ‘bool (*)(std::__cxx11::basic_string<char>, std::__cxx11::basic_string<char>)’ to ‘const void*’ [-fpermissive]

				.def(py::init<const std::string&, const std::string&, const std::string&, const std::string&, const std::string&, Parametrizable::LexicalComparison>(), py::arg("name"), py::arg("doc"), py::arg("defaultValue"), py::arg("minValue"), py::arg("maxValue"), py::arg("comp"))
				.def(py::init<const std::string&, const std::string&, const std::string&>(), py::arg("name"), py::arg("doc"), py::arg("defaultValue"))

				.def("__str__", [](const ParameterDoc& self)
				{
					std::ostringstream oss;
					oss << self;
					return oss.str();
				});

			using ParametersDoc = Parametrizable::ParametersDoc;
			py::bind_vector<ParametersDoc>(pyParametrizable, "ParametersDoc", "The documentation of all parameters");

			using Parameters = Parametrizable::Parameters;
			py::bind_map<Parameters>(pyParametrizable, "Parameters", "The documentation of all parameters", py::module_local())
				.def("clear", &Parameters::clear, "Remove all items from D.");

			pyParametrizable.def_readonly("className", &Parametrizable::className, "name of the class")
				.def_readonly("parametersDoc", &Parametrizable::parametersDoc, "documentation of parameters")
				.def_readwrite("parameters", &Parametrizable::parameters, "parameters with their values encoded in string")
				.def_readwrite("parametersUsed", &Parametrizable::parametersUsed, "parameters whose value has actually been read")

				.def(py::init<>())
				.def(py::init<const std::string&, const ParametersDoc, Parameters&>(), py::arg("className"), py::arg("paramsDoc"), py::arg("params"))

				.def("getParamValueString", &Parametrizable::getParamValueString)
				.def("__str__", [](const Parametrizable& self)
				{
					std::ostringstream oss;
					oss << self;
					return oss.str();
				});

			p_module.def("__str__", [](const ParametersDoc& self)
			{
				std::ostringstream oss;
				oss << self;
				return oss.str();
			});
		}
	}
}

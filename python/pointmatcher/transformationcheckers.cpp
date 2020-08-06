#include "transformationcheckers.h"

namespace pointmatcher
{
	void pybindTransformationCheckers(py::class_<PM>& p_class)
	{
		py::bind_vector<std::vector<std::shared_ptr<TransformationChecker>>>(p_class, "TransformationCheckersVector");

		py::class_<TransformationCheckers, std::vector<std::shared_ptr<TransformationChecker>>>(p_class, "TransformationCheckers")
		    .def(py::init<>())
		    .def("init", &TransformationCheckers::init, py::arg("parameters"), py::arg("iterate"))
		    .def("check", &TransformationCheckers::check, py::arg("parameters"), py::arg("iterate"));
	}
}
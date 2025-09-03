#include "transformation_checkers.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindTransformationCheckers(py::class_<PM>& p_class)
		{
			py::bind_vector<TransformationCheckers>(p_class, "TransformationCheckers", "A chain of TransformationChecker")
				.def(py::init<>())
				.def("init", &TransformationCheckers::init, py::arg("parameters"), py::arg("iterate"), "Init all transformation checkers, set iterate to false if iteration should stop")
				.def("check", &TransformationCheckers::check, py::arg("parameters"), py::arg("iterate"), "Check using all transformation checkers, set iterate to false if iteration should stop");
		}
	}
}

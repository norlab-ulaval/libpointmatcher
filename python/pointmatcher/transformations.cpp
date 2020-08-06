#include "transformations.h"

namespace pointmatcher
{
	void pybindTranformations(py::class_<PM>& p_class)
	{
		py::bind_vector<std::vector<std::shared_ptr<Transformation>>>(p_class, "TransformationsVector");

		py::class_<Transformations, std::vector<std::shared_ptr<Transformation>>>(p_class, "Transformations", "A chain of Transformation")
			.def(py::init<>())
			.def("apply", &Transformations::apply, py::arg("cloud"), py::arg("parameters"));
	}
}
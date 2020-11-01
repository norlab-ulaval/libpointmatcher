#include "transformations.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindTranformations(py::class_<PM>& p_class)
		{
			py::bind_vector<Transformations>(p_class, "Transformations", "A chain of Transformation").def(py::init<>())
				.def("apply", &Transformations::apply, py::arg("cloud"), py::arg("parameters"), "Apply this chain to cloud, using parameters, mutates cloud");
		}
	}
}

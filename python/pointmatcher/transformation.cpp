#include "transformation.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindTranformation(py::class_<PM>& p_class)
		{
			py::class_<Transformation, std::shared_ptr<Transformation>, Parametrizable>(p_class, "Transformation", "A function that transforms points and their descriptors given a transformations matrix");
		}
	}
}

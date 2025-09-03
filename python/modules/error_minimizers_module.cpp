#include "error_minimizers_module.h"

#include "errorminimizers/identity.h"
#include "errorminimizers/point_to_plane.h"
#include "errorminimizers/point_to_plane_with_cov.h"
#include "errorminimizers/point_to_point.h"
#include "errorminimizers/point_to_point_similarity.h"
#include "errorminimizers/point_to_point_with_cov.h"

namespace python
{
	namespace modules
	{
		void pybindErrorMinimizersModule(py::module& p_module)
		{
			py::module errorminizersModule = p_module.def_submodule("errorminimizers");

			errorminimizers::pybindIdentity(errorminizersModule);
			errorminimizers::pybindPointToPlane(errorminizersModule);
			errorminimizers::pybindPointToPlaneWithCov(errorminizersModule);
			errorminimizers::pybindPointToPoint(errorminizersModule);
			errorminimizers::pybindPointToPointSimilarity(errorminizersModule);
			errorminimizers::pybindPointToPointWithCov(errorminizersModule);
		}
	}
}

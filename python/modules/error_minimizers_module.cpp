#include "error_minimizers_module.h"

#include "errorminimizers/identity.h"
#include "errorminimizers/point_to_plane.h"
#include "errorminimizers/point_to_plane_with_cov.h"
#include "errorminimizers/point_to_point.h"
#include "errorminimizers/point_to_point_similarity.h"
#include "errorminimizers/point_to_point_with_cov.h"

namespace pointmatcher
{
	void pybindErrorMinimizersModule(py::module& p_module)
	{
		py::module errorminizersModule = p_module.def_submodule("errorminimizers");

		pybindIdentityEM(errorminizersModule);
		pybindPointToPlane(errorminizersModule);
		pybindPointToPlaneWithCov(errorminizersModule);
		pybindPointToPoint(errorminizersModule);
		pybindPointToPointSimilarity(errorminizersModule);
		pybindPointToPointWithCov(errorminizersModule);
	}
}

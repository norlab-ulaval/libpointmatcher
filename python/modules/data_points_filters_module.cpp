#include "data_points_filters_module.h"

#include "datapointsfilters/bounding_box.h"
#include "datapointsfilters/covariance_sampling.h"
#include "datapointsfilters/cut_at_descriptor_threshold.h"
#include "datapointsfilters/distance_limit.h"
#include "datapointsfilters/ellipsoids.h"
#include "datapointsfilters/fix_step_sampling.h"
#include "datapointsfilters/gestalt.h"
#include "datapointsfilters/identity.h"
#include "datapointsfilters/incidence_angle.h"
#include "datapointsfilters/max_density.h"
#include "datapointsfilters/max_pointcount.h"
#include "datapointsfilters/max_quantile_on_axis.h"
#include "datapointsfilters/normal_space.h"
#include "datapointsfilters/observation_direction.h"
#include "datapointsfilters/octree_grid.h"
#include "datapointsfilters/orient_normals.h"
#include "datapointsfilters/random_sampling.h"
#include "datapointsfilters/remove_nan.h"
#include "datapointsfilters/remove_sensor_bias.h"
#include "datapointsfilters/sampling_surface_normal.h"
#include "datapointsfilters/shadow.h"
#include "datapointsfilters/simple_sensor_noise.h"
#include "datapointsfilters/sphericality.h"
#include "datapointsfilters/surface_normal.h"

namespace python
{
	namespace modules
	{
		void pybindDataPointsFiltersModule(py::module& p_module)
		{
			py::module datapointsfilterModule = p_module.def_submodule("datapointsfilters");

			datapointsfilters::pybindBoundingBox(datapointsfilterModule);
			datapointsfilters::pybindCovarianceSampling(datapointsfilterModule);
			datapointsfilters::pybindCutAtDescriptorThreshold(datapointsfilterModule);
			datapointsfilters::pybindDistanceLimit(datapointsfilterModule);
			datapointsfilters::pybindEllipsoids(datapointsfilterModule);
			datapointsfilters::pybindFixStepSampling(datapointsfilterModule);
			datapointsfilters::pybindGestalt(datapointsfilterModule);
			datapointsfilters::pybindIdentity(datapointsfilterModule);
			datapointsfilters::pybindIncidenceAngle(datapointsfilterModule);
			datapointsfilters::pybindMaxDensity(datapointsfilterModule);
			datapointsfilters::pybindMaxPointCount(datapointsfilterModule);
			datapointsfilters::pybindMaxQuantileOnAxis(datapointsfilterModule);
			datapointsfilters::pybindNormalSpace(datapointsfilterModule);
			datapointsfilters::pybindObservationDirection(datapointsfilterModule);
			datapointsfilters::pybindOctreeGrid(datapointsfilterModule);
			datapointsfilters::pybindOrientNormals(datapointsfilterModule);
			datapointsfilters::pybindRandomSampling(datapointsfilterModule);
			datapointsfilters::pybindRemoveNaN(datapointsfilterModule);
			datapointsfilters::pybindRemoveSensorBias(datapointsfilterModule);
			datapointsfilters::pybindSamplingSurfaceNormal(datapointsfilterModule);
			datapointsfilters::pybindShadow(datapointsfilterModule);
			datapointsfilters::pybindSimpleSensorNoise(datapointsfilterModule);
			datapointsfilters::pybindSphericality(datapointsfilterModule);
			datapointsfilters::pybindSurfaceNormal(datapointsfilterModule);
		}
	}
}

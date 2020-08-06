import numpy as np

from math import sqrt
from pypointmatcher import pointmatcher, pointmatchersupport
from examples.python.utils import parse_translation, parse_rotation

PM = pointmatcher.PointMatcher
DP = PM.DataPoints
Parameters = pointmatchersupport.Parametrizable.ParametersMap

#  Code example for ICP taking 2 points clouds (2D or 3D) relatively close
#  and computing the transformation between them.
#
#  This code is more complete than icp_simple. It can load parameter files and has more options.
config_file = "../data/default.yaml"
output_base_file = "tests_output/icp_advance_api/"
is_3D = True  # (toggle to switch between 2D and 3D clouds)
init_translation = "1,1,1" if is_3D else "1,1"
init_rotation = "1,0,0;0,1,0;0,0,1" if is_3D else "1,0;0,1"

if is_3D:
    # 3D point clouds
    ref = DP.load('../data/car_cloud400.csv')
    data = DP.load('../data/car_cloud401.csv')
    test_base = "3D"
else:
    # 2D point clouds
    ref = DP.load('../data/2D_twoBoxes.csv')
    data = DP.load('../data/2D_oneBox.csv')
    test_base = "2D"

icp = PM.ICP()

# icp.setDefault()
icp.loadFromYaml(config_file)

cloud_dimension = ref.getEuclideanDim()

assert cloud_dimension == 2 or cloud_dimension == 3, "Invalid input point clouds dimension"

translation = parse_translation(init_translation, cloud_dimension)
rotation = parse_rotation(init_rotation, cloud_dimension)
init_transfo = np.matmul(translation, rotation)

rigid_trans = PM.get().TransformationRegistrar.create("RigidTransformation")

if not rigid_trans.checkParameters(init_transfo):
    print("Initial transformations is not rigid, identiy will be used")
    init_transfo = np.identity(cloud_dimension + 1)

initialized_data = rigid_trans.compute(data, init_transfo)

T = icp(initialized_data, ref)
match_ratio = icp.errorMinimizer.getWeightedPointUsedRatio()
print(f"match ratio: {match_ratio:.6}")
print("\n------------------")

data_out = DP(initialized_data)
icp.transformations.apply(data_out, T)

#  START demo 1
#  Test for retrieving Haussdorff distance (with outliers). We generate new matching module
#  specifically for this purpose.
#
#  INPUTS:
#  ref: point cloud used as reference
#  data_out: aligned point cloud (using the transformation outputted by icp)
#  icp: icp object used to aligned the point clouds

params = Parameters()
params["knn"] = "1"
params["epsilon"] = "0"

matcher_Hausdorff = PM.get().MatcherRegistrar.create("KDTreeMatcher", params)

# max. distance from reading to reference
matcher_Hausdorff.init(ref)
matches = matcher_Hausdorff.findClosests(data_out)
max_dist1 = matches.getDistsQuantile(1.0)
max_dist_robust1 = matches.getDistsQuantile(0.85)

# max. distance from reference to reading
matcher_Hausdorff.init(data_out)
matches = matcher_Hausdorff.findClosests(ref)
max_dist2 = matches.getDistsQuantile(1.0)
max_dist_robust2 = matches.getDistsQuantile(0.85)

haussdorff_dist = max(max_dist1, max_dist2)
haussdorff_quantile_dist = max(max_dist_robust1, max_dist_robust2)

print(f"Haussdorff distance: {sqrt(haussdorff_dist):.6} m")
print(f"Haussdorff quantile distance: {sqrt(haussdorff_quantile_dist):.6} m")

#  START demo 2
#  Test for retrieving paired point mean distance without outliers.
#  We reuse the same module used for the icp object.
#
#  INPUTS:
#  ref: point cloud used as reference
#  data_out: aligned point cloud (using the transformation outputted by icp)
#  icp: icp object used to aligned the point clouds

# initiate the matching with unfiltered point cloud
icp.matcher.init(ref)

# extract closest points
matches = icp.matcher.findClosests(data_out)

# weight paired points
outlier_weights = icp.outlierFilters.compute(data_out, ref, matches)

# generate tuples of matched points and remove pairs with zero weight
matched_points = PM.ErrorMinimizer.ErrorElements(data_out, ref, outlier_weights, matches)

# extract relevant information for convenience
dim = matched_points.reading.getEuclideanDim()
nb_matched_points = matched_points.reading.getNbPoints()
matched_read = matched_points.reading.features[:dim]
matched_ref = matched_points.reference.features[:dim]

# compute mean distance
dist = np.linalg.norm(matched_read - matched_ref, axis=0)
mean_dist = dist.sum() / nb_matched_points

print(f"Robust mean distance: {mean_dist:.6} m")
print("------------------\n")

# End demo

ref.save(output_base_file + f"{test_base}_test_ref.vtk")
data.save(output_base_file + f"{test_base}_test_data_in.vtk")
data_out.save(output_base_file + f"{test_base}_test_data_out.vtk")

print(f"{test_base} ICP transformation:\n{T}".replace("[", " ").replace("]", " "))

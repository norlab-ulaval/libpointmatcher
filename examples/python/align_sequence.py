# Code example for ICP taking a sequence of point clouds relatively close
# and build a map with them.
# It assumes that: 3D point clouds are used, they were recorded in sequence
# and they are express in sensor frame.

import numpy as np

from pypointmatcher import pointmatcher as pm, pointmatchersupport as pms

PM = pm.PointMatcher
PMIO = pm.PointMatcherIO
DP = PM.DataPoints
params = pms.Parametrizable.Parameters()

# Path of output directory (default: tests/align_sequence/)
# The output directory must already exist
# Leave empty to save in the current directory
output_base_directory = "tests/align_sequence/"

# Name of output files (default: align_sequence)
output_file_name = "align_sequence"

# Rigid transformation
rigid_trans = PM.get().TransformationRegistrar.create("RigidTransformation")

# Create filters manually to clean the global map
params["knn"] = "10"
params["epsilon"] = "5"
params["keepNormals"] = "0"
params["keepDensities"] = "1"
density_filter = PM.get().DataPointsFilterRegistrar.create("SurfaceNormalDataPointsFilter", params)
params.clear()

params["maxDensity"] = "30"
max_density_subsample = PM.get().DataPointsFilterRegistrar.create("MaxDensityDataPointsFilter",
                                                                  params)
params.clear()

# Main algorithm definition
icp = PM.ICP()

# load YAML config
config_file = "../data/default.yaml"
pms.validateFile(config_file)
icp.loadFromYaml(config_file)

# Loading the list of files
# file_info_list = PMIO.FileInfoVector("../data/carCloudList.csv", "../data/")
# or
file_info_list = PMIO.FileInfoVector("../data/cloudList.csv", "../data/")

map_point_cloud = DP()
new_cloud = DP()

T_to_map_from_new = np.identity(4)  # assumes 3D

for i in range(len(file_info_list)):
    print(f"---------------------\nLoading: {file_info_list[i].readingFileName}")

    # It is assume that the point cloud is express in sensor frame
    new_cloud = DP.load(file_info_list[i].readingFileName)

    if map_point_cloud.getNbPoints() == 0:
        map_point_cloud = new_cloud
        continue

    # call ICP
    try:
        # We use the last transformation as a prior
        # this assumes that the point clouds were recorded in
        # sequence.
        prior = T_to_map_from_new

        T_to_map_from_new = icp(new_cloud, map_point_cloud, prior)

    except PM.ConvergenceError as CE:
        print(f"ERROR PM.ICP failed to converge: \n\t{CE}\n\n")
        continue

    # This is not necessary in this example, but could be
    # useful if the same matrix is composed in the loop.
    T_to_map_from_new = rigid_trans.correctParameters(T_to_map_from_new)

    # Move the new point cloud in the map reference
    new_cloud = rigid_trans.compute(new_cloud, T_to_map_from_new)

    # Merge point clouds to map
    map_point_cloud.concatenate(new_cloud)

    # Clean the map
    map_point_cloud = density_filter.filter(map_point_cloud)
    map_point_cloud = max_density_subsample.filter(map_point_cloud)

    # Save the map at each iteration
    output_file_name_iter = f"{output_file_name}_{i}.vtk"
    print(f"outputFileName: {output_file_name_iter}")
    map_point_cloud.save(f"{output_base_directory}{output_file_name_iter}")

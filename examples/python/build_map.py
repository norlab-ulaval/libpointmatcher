# Code example for DataFilter taking a sequence of point clouds with
# their global coordinates and build a map with a fix (manageable) number of points.
# The example shows how to generate filters in the source code.
# For an example generating filters using yaml configuration, see demo_cmake/convert.cpp
# For an example with a registration solution, see icp.cpp

import numpy as np

from pypointmatcher import pointmatcher as pm, pointmatchersupport as pms

PM = pm.PointMatcher
PMIO = pm.PointMatcherIO
DP = PM.DataPoints
params = pms.Parametrizable.Parameters()

# Loading the list of files
file_info_list = PMIO.FileInfoVector("../data/carCloudList.csv", "../data/")
total_point_count = 30000

# Path of output directory (default: tests/build_map/)
# The output directory must already exist
# Leave empty to save in the current directory
output_base_directory = "tests/build_map/"

# Name of output file: file_name.{vtk,csv,ply} (default: test.vtk)
output_file_name = "test.vtk"

pms.setLogger(PM.get().LoggerRegistrar.create("FileLogger"))

map_cloud = DP()
last_cloud = DP()
new_cloud = DP()

T = np.identity(4)

# Define transformation chain
transformation = PM.get().TransformationRegistrar.create("RigidTransformation")

# This filter will remove a sphere of 1 m radius. Easy way to remove the sensor self-scanning.
params["minDist"] = "1.0"
remove_scanner = PM.get().DataPointsFilterRegistrar.create("MinDistDataPointsFilter", params)
params.clear()

# This filter will randomly remove 35% of the points.
params["prob"] = "0.65"
rand_subsample = PM.get().DataPointsFilterRegistrar.create("RandomSamplingDataPointsFilter", params)
params.clear()

# For a complete description of filter, see
# https://github.com/ethz-asl/libpointmatcher/blob/master/doc/Datafilters.md
params["knn"] = "10"
params["epsilon"] = "5"
params["keepNormals"] = "1"
params["keepDensities"] = "0"
normal_filter = PM.get().DataPointsFilterRegistrar.create("SurfaceNormalDataPointsFilter", params)
params.clear()

params["knn"] = "10"
params["epsilon"] = "5"
params["keepDensities"] = "1"
params["keepNormals"] = "0"
density_filter = PM.get().DataPointsFilterRegistrar.create("SurfaceNormalDataPointsFilter", params)
params.clear()

observation_direction_filter = PM.get().DataPointsFilterRegistrar.create("ObservationDirectionDataPointsFilter")

params["towardCenter"] = "1"
orien_normal_filter = PM.get().DataPointsFilterRegistrar.create("OrientNormalsDataPointsFilter", params)
params.clear()

params["maxDensity"] = "30"
uniform_subsample = PM.get().DataPointsFilterRegistrar.create("MaxDensityDataPointsFilter", params)
params.clear()

shadow_filter = PM.get().DataPointsFilterRegistrar.create("ShadowDataPointsFilter")

for i in range(len(file_info_list)):
    print("\n-----------------------------")
    print(f"Loading {file_info_list[i].readingFileName} ", end="")

    new_cloud = DP.load(file_info_list[i].readingFileName)

    print(f"found {new_cloud.getNbPoints()} points.")

    if file_info_list[i].groundTruthTransformation.shape[0] != 0:
        T = file_info_list[i].groundTruthTransformation
    else:
        print("ERROR: the field gTXX (ground truth) is required")
        exit()

    # Remove the scanner
    new_cloud = remove_scanner.filter(new_cloud)

    # Accelerate the process and dissolve lines
    new_cloud = rand_subsample.filter(new_cloud)

    # Build filter to remove shadow points and down-sample
    new_cloud = normal_filter.filter(new_cloud)
    new_cloud = observation_direction_filter.filter(new_cloud)
    new_cloud = orien_normal_filter.filter(new_cloud)
    new_cloud = shadow_filter.filter(new_cloud)

    # Transforme pointCloud
    print(f"Transformation matrix:\n{T}\n".replace("[", " ").replace("]", " "), end="")
    new_cloud = transformation.compute(new_cloud, T)

    if i == 0:
        map_cloud = new_cloud
    else:
        map_cloud.concatenate(new_cloud)

        # Control point cloud size
        prob_to_keep = total_point_count / map_cloud.features.shape[1]

        if prob_to_keep < 1:
            map_cloud = density_filter.filter(map_cloud)
            map_cloud = uniform_subsample.filter(map_cloud)

            prob_to_keep = total_point_count / map_cloud.features.shape[1]

            if prob_to_keep < 1:
                print(f"Randomly keep {prob_to_keep * 100}% points")

                rand_subsample = PM.get().DataPointsFilterRegistrar.create(
                    "RandomSamplingDataPointsFilter",
                    {"prob": f"{prob_to_keep}"})

                map_cloud = rand_subsample.filter(map_cloud)

    map_cloud.save(f"{output_base_directory + output_file_name[:-4]}_{i}.vtk")

map_cloud = density_filter.filter(map_cloud)
map_cloud = uniform_subsample.filter(map_cloud)
map_cloud = density_filter.filter(map_cloud)

print("\n-----------------------------"*2)
print(f"Final number of points in the map: {map_cloud.getNbPoints()}")

map_cloud.save(f"{output_base_directory + output_file_name}")

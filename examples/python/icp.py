# Code example for ICP taking 2 points clouds (2D or 3D) relatively close
# and computing the transformation between them.
#
# This code is more complete than icp_simple. It can load parameter files and
# has more options.

import numpy as np

from pypointmatcher import pointmatcher as pm
from utils import parse_translation, parse_rotation

PM = pm.PointMatcher
DP = PM.DataPoints

# Save transformation matrix in three different files:
#     - BASEFILENAME_inti_transfo.txt
#     - BASEFILENAME_icp_transfo.txt
#     - BASEFILENAME_complete_transfo.txt
#       (default: false)
is_transfo_saved = False

# Be more verbose (info logging to the console)
is_verbose = True

# Load the config from a YAML file (default: default.yaml)
# Leave empty to set the ICP default configuration
config_file = "../data/default.yaml"

# Path of output directory (default: tests/icp/)
# The output directory must already exist
# Leave empty to save in the current directory
output_base_directory = "tests/icp/"

# Name of output files (default: test)
output_base_file = "test"

# Toggle to switch between 2D and 3D point clouds
is_3D = True

# Add an initial 3D translation before applying ICP (default: 0,0,0)
# Add an initial 2D translation before applying ICP (default: 0,0)
init_translation = "0,0,0" if is_3D else "0,0"
# Add an initial 3D rotation before applying ICP (default: 1,0,0;0,1,0;0,0,1)
# Add an initial 2D rotation before applying ICP (default: 1,0;0,1)
init_rotation = "1,0,0;0,1,0;0,0,1" if is_3D else "1,0;0,1"

if is_3D:
    # Load 3D point clouds
    ref = DP(DP.load('../data/car_cloud400.csv'))
    data = DP(DP.load('../data/car_cloud401.csv'))
    test_base = "3D"
else:
    # Load 2D point clouds
    ref = DP(DP.load('../data/2D_twoBoxes.csv'))
    data = DP(DP.load('../data/2D_oneBox.csv'))
    test_base = "2D"

# Create the default ICP algorithm
icp = PM.ICP()

if len(config_file) == 0:
    # See the implementation of setDefault() to create a custom ICP algorithm
    icp.setDefault()
else:
    # load YAML config
    icp.loadFromYaml(config_file)

cloud_dimension = ref.getEuclideanDim()

assert cloud_dimension == 2 or cloud_dimension == 3, "Invalid input point clouds dimension"

# Parse the translation and rotation to be used to compute the initial transformation
translation = parse_translation(init_translation, cloud_dimension)
rotation = parse_rotation(init_rotation, cloud_dimension)

init_transfo = np.matmul(translation, rotation)

rigid_trans = PM.get().TransformationRegistrar.create("RigidTransformation")

if not rigid_trans.checkParameters(init_transfo):
    print("Initial transformations is not rigid, identiy will be used")
    init_transfo = np.identity(cloud_dimension + 1)

initialized_data = rigid_trans.compute(data, init_transfo)

# Compute the transformation to express data in ref
T = icp(initialized_data, ref)

if is_verbose:
    print(f"match ratio: {icp.errorMinimizer.getWeightedPointUsedRatio():.6}")

# Transform data to express it in ref
data_out = DP(initialized_data)
icp.transformations.apply(data_out, T)

# Save files to see the results
ref.save(f"{output_base_directory + test_base}_{output_base_file}_ref.vtk")
data.save(f"{output_base_directory + test_base}_{output_base_file}_data_in.vtk")
data_out.save(f"{output_base_directory + test_base}_{output_base_file}_data_out.vtk")

if is_transfo_saved:
    init_file_name = f"{output_base_directory + test_base}_{output_base_file}_init_transfo.txt"
    icp_file_name = f"{output_base_directory + test_base}_{output_base_file}_icp.transfo.txt"
    complete_file_name = f"{output_base_directory + test_base}_{output_base_file}_complete_transfo.txt"

    with open(init_file_name, "w") as f:
        f.write(f"{init_transfo}".replace("[", " ").replace("]", " "))

    with open(icp_file_name, "w") as f:
        f.write(f"{T}".replace("[", " ").replace("]", " "))

    with open(complete_file_name, "w") as f:
        f.write(f"{np.matmul(T, init_transfo)}".replace("[", " ").replace("]", " "))

else:
    if is_verbose:
        print(f"{test_base} ICP transformation:\n{T}".replace("[", " ").replace("]", " "))

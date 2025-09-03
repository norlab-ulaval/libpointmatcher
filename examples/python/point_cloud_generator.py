# Code example for ICP generating different shapes using the built-in interface
import os.path

import numpy as np
from pypointmatcher import pointmatcher as pm

PM = pm.PointMatcher
DP = PM.DataPoints
Generator = pm.PointCloudGenerator

# Path of output directory (default: tests/icp_simple/)
# The output directory must already exist
# Leave empty to save in the current directory
output_base_directory = "tests/generator/"

# How many points will each generated shape contain
number_of_points = 10000

# Toggle to switch between 2D and 3D clouds
# Only 3D is currently supported
is_3D = True

if is_3D:
    # Load 3D point clouds
    ref = DP(DP.load('../data/car_cloud400.csv'))
    data = DP(DP.load('../data/car_cloud401.csv'))
    test_base = "3D"
    translation = np.array([[0], [0], [0]])
    rotation = np.array([1, 0, 0, 0], dtype=np.float32)
else:
    raise Exception("The Point Cloud Generator only supports 3D shapes")

box = Generator.generateUniformlySampledBox(1.0, 2.0, 3.0, number_of_points, translation, rotation)
circle = Generator.generateUniformlySampledCircle(1.0, number_of_points, translation, rotation)
cylinder = Generator.generateUniformlySampledCylinder(1.0, 2.0, number_of_points, translation, rotation)
plane = Generator.generateUniformlySampledPlane(np.array([1.0, 2.0, 3.0]), number_of_points, translation, rotation)
sphere = Generator.generateUniformlySampledSphere(1.0, number_of_points, translation, rotation)


# Save files to see the results
if not os.path.exists(output_base_directory):
    os.makedirs(output_base_directory)

box.save(f"{output_base_directory}box.vtk")
circle.save(f"{output_base_directory}circle.vtk")
cylinder.save(f"{output_base_directory}cylinder.vtk")
plane.save(f"{output_base_directory}plane.vtk")
sphere.save(f"{output_base_directory}sphere.vtk")

print(f"Saved generated shapes into {output_base_directory}")

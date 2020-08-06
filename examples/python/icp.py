import numpy as np

from pypointmatcher.pointmatcher import PointMatcher
from examples.python.utils import parse_translation, parse_rotation

PM = PointMatcher
DP = PM.DataPoints

config_file = "../data/default.yaml"
output_base_file = "tests_output/icp/"
is_3D = True
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

icp.loadFromYaml(config_file)
# icp.setDefault()

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

data_out = DP(initialized_data)
icp.transformations.apply(data_out, T)

ref.save(output_base_file + f"{test_base}_test_ref.vtk")
data.save(output_base_file + f"{test_base}_test_data_in.vtk")
data_out.save(output_base_file + f"{test_base}_test_data_out.vtk")

print(f"{test_base} ICP transformation:\n{T}".replace("[", " ").replace("]", " "))

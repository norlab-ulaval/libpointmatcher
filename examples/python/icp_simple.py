# Code example for ICP taking 2 points clouds (2D or 3D) relatively close
# and computing the transformation between them.

from pypointmatcher import pointmatcher as pm

PM = pm.PointMatcher
DP = PM.DataPoints

# Path of output directory (default: tests/icp_simple/)
# The output directory must already exist
# Leave empty to save in the current directory
output_base_directory = "tests/icp_simple/"

# Name of output files (default: test)
output_base_file = "test"

# Toggle to switch between 2D and 3D clouds
is_3D = True

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

# See the implementation of setDefault() to create a custom ICP algorithm
icp.setDefault()

# Compute the transformation to express data in ref
T = icp(data, ref)

# Transform data to express it in ref
data_out = DP(data)
icp.transformations.apply(data_out, T)

# Save files to see the results
ref.save(f"{output_base_directory + test_base}_{output_base_file}_ref.vtk")
data.save(f"{output_base_directory + test_base}_{output_base_file}_data_in.vtk")
data_out.save(f"{output_base_directory + test_base}_{output_base_file}_data_out.vtk")

print(f"Final {test_base} transformations:\n{T}\n".replace("[", " ").replace("]", " "))

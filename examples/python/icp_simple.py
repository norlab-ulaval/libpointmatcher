from pypointmatcher.pointmatcher import PointMatcher

PM = PointMatcher
DP = PM.DataPoints
output_base_file = "tests_output/icp_simple/"
is_3D = True  # (toggle to switch between 2D and 3D clouds)

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

icp.setDefault()

TP = icp(ref, data)

data_out = DP(data)

icp.transformations.apply(data_out, TP)

ref.save(output_base_file + f"{test_base}_test_ref.vtk")
data.save(output_base_file + f"{test_base}_test_data_in.vtk")
data_out.save(output_base_file + f"{test_base}_data_out.vtk")

print(f"Final {test_base} transformations:\n{TP}\n".replace("[", " ").replace("]", " "))

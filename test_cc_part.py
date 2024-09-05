import open3d as o3d
import os
from engine import cloudcompare_part as proc
from engine import agisoft_part as agi

agisoft_pc_path = r'D:\Python2024\OrthoDetect\output\agisoft_outputs\point_cloud.ply'
cloudcomp_out_dir = r'D:\Python2024\OrthoDetect\output\cloudcompare_outputs'

"""
°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
*******************************  PREPROCESSING  ******************************************
All the operations that always occur when importing a point cloud
°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
"""
# 1. BASIC DATA ____________________________________________________________________________________________________

# read full high definition point cloud (using open3d)
sub_sampled = False
print('Reading the point cloud!')
pc_load = o3d.io.read_point_cloud(agisoft_pc_path)
print('Pc loaded!')

# compute some very basic properties
bound_pc_path = os.path.join(cloudcomp_out_dir, "pc_limits.ply")
bound, bound_points, center, dim, density, n_points = proc.compute_basic_properties(pc_load, save_bound_pc = True, output_path_bound_pc = bound_pc_path)

print(f'The point cloud density is: {density:.3f}')

# 2. RANSAC DETECTION __________________________________________________________________________________
print('Launching RANSAC detection...')

# create RANSAC directories
ransac_cloud_folder = os.path.join(cloudcomp_out_dir, 'RANSAC_pc')
proc.new_dir(ransac_cloud_folder)

ransac_obj_folder = os.path.join(cloudcomp_out_dir, 'RANSAC_meshes')
proc.new_dir(ransac_obj_folder)

# fixing RANSAC Parameters
min_points = n_points / 200

n_planes = proc.preproc_ransac_short(agisoft_pc_path, min_points, 0.05, ransac_obj_folder, ransac_cloud_folder)


# create full plan list
plane_list = proc.generate_list('obj', ransac_obj_folder, exclude='merged')
# print all plane data
for plane in plane_list:
    plane_data = proc.plane_data_light(plane)
    normal = [plane_data[5], plane_data[6], plane_data[7]]
    print(f'Plane:{plane}, Normal: {normal}')

# find vertical planes

ver_planes = proc.filter_planes(plane_list, 'vertical', 5)

# get normals direction
normals_list = []
for plane in ver_planes:
    plane_data = proc.plane_data_light(plane)
    normal = [plane_data[5], plane_data[6], plane_data[7]]
    normals_list.append(normal)
    print(f'Plane:{plane}, Normal: {normal}, Vertical!')
print(normals_list)

# run agisoft for ortho creation
agi.run_agisoft_rgb_only_part2(r'D:\Python2024\OrthoDetect\output\agisoft_outputs\agisoft.psx', normals_list)


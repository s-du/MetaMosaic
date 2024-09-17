import statistics
import numpy as np
import open3d as o3d
import os
import subprocess
import math
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import find_peaks



# PATHS FUNCTIONS_______________________________________________________________________________________________________
cc_path = os.path.join("C:\\", "Program Files", "CloudCompare", "CloudCompare")  # path to cloudcompare exe


def generate_list(file_format, dir, exclude='text_to_exclude', include=''):
    """
    Function that generates the list of file with a specific extension in a folder
    :param file_format: (str) the extension to look for
    :param dir: (str) the folder to look into
    :param exclude: (str) an optional parameter to exclude files that include some text in their name
    :param include: (str) an optional parameter to specifically include files with some text in their name
    :return: (list) the list of detected files
    """
    file_list = []
    for file in os.listdir(dir):
        fileloc = os.path.join(dir, file)
        if file.endswith(file_format):
            if exclude not in file:
                if include in file:
                    file_list.append(fileloc)
    return file_list


def new_dir(dir_path):
    """
    Simple function to verify if a directory exists and if not creating it
    :param dir_path: (str) the path to check
    :return:
    """
    if not os.path.exists(dir_path):
        os.mkdir(dir_path)


def find_substring(substring, folder):
    """
    Function that finds a file with a specific substring in a folder, and return its path
    @ parameters:
        substring -- substring to be looked for (string)
        folder -- input folder (string)
    """
    for file in os.listdir(folder):
        if substring in file:
            path = os.path.join(folder, file)
    return path


def find_substring_new_path(substring, new_path, folder):
    """
    Function that finds a file with a specific substring in a folder, and move it to a new location
    @ parameters:
        substring -- substring to be looked for (string)
        new_path -- new_path where to move the found file (string)
        folder -- input folder (string)
    """
    # rename and place in right folder

    for file in os.listdir(folder):
        if substring in file:
            os.rename(os.path.join(folder, file), new_path)


def find_substring_delete(substring, folder):
    for file in os.listdir(folder):
        if substring in file:
            os.remove(os.path.join(folder, file))


# OPEN3D FUNCTIONS______________________________________________________________________________________________________
def get_average_normal(pcd):
    # Check if the point cloud has normals
    if not pcd.has_normals():
        print("The point cloud does not have normals.")
        exit()

    # Access the normals as a numpy array
    normals = np.asarray(pcd.normals)

    # Compute the average normal vector
    average_normal = np.mean(normals, axis=0)

    # Normalize the average normal (optional, but often useful)
    average_normal /= np.linalg.norm(average_normal)

    # Print the result
    print("Average Normal Vector: ", average_normal)

    return average_normal

def create_box_limit(pcd, output_path):
    bound = pcd.get_oriented_bounding_box()
    points_bb = bound.get_box_points()
    points_bb_np = np.asarray(points_bb)

    pcd = o3d.geometry.PointCloud()
    pcd.points = points_bb
    pcd = pcd.paint_uniform_color([0.5, 0.5, 0.5])

    # write point cloud
    o3d.io.write_point_cloud(output_path, pcd)


def compute_basic_properties(pc_load, save_bound_pc=False, output_path_bound_pc='', compute_full_density=True):
    """
    A function that compiles all the basic properties of the point cloud
    :param pc_load: (open3D pc) the point cloud object, loaded into open3D
    :param save_bound_pc: (bool) Whether to save the bounding box as a point cloud
    :param output_path_bound_pc: (str) the path where to save the bounding box point cloud
    :param compute_density: (bool) whether to compute the density on the whole the point cloud
    :return:bound = the bounding box, center= the center of the bounding box, dim= the extend of the bounding box, n_points= the number of points
    """

    # Bounding box
    bound = pc_load.get_axis_aligned_bounding_box()
    center = bound.get_center()
    dim = bound.get_extent()
    points_bb = bound.get_box_points()

    # Debug prints
    print(f'Bounding box center: {center}')
    print(f'Bounding box dimensions: {dim}')
    print(f'Type of points_bb: {type(points_bb)}')
    print(f'Number of bounding box points: {len(points_bb)}')

    points_bb_np = np.asarray(points_bb)
    print('Bounding box points converted to numpy array.')

    # Create point cloud from bounding box
    pcd = o3d.geometry.PointCloud()
    print('Generic point cloud object created')
    pcd.points = points_bb
    print('Bounding box points assigned to pcd.points.')

    # Optionally save the bounding box point cloud
    if save_bound_pc:
        o3d.io.write_point_cloud(output_path_bound_pc, pcd)
        print(f'Saved bounding box point cloud to: {output_path_bound_pc}')

    # Return values
    n_points = len(pc_load.points)

    # write point cloud
    if save_bound_pc:
        o3d.io.write_point_cloud(output_path_bound_pc, pcd)

    # output the number of points of the point cloud
    points_list = np.asarray(pc_load.points)

    # compute density
    density = []
    dim_x = dim[0]
    dim_y = dim[1]
    dim_z = dim[2]

    if not compute_full_density:
        pt1 = [center[0] - dim_x / 8, center[1] - dim_y / 8, center[2] - dim_z / 2]
        pt2 = [center[0] + dim_x / 8, center[1] + dim_y / 8, center[2] + dim_z / 2]
        np_points = [pt1, pt2]
        points = o3d.utility.Vector3dVector(np_points)

        crop_box = o3d.geometry.AxisAlignedBoundingBox
        crop_box = crop_box.create_from_points(points)

        point_cloud_crop = pc_load.crop(crop_box)
        dist = point_cloud_crop.compute_nearest_neighbor_distance()

        if dist:
            density = statistics.mean(dist)
        else:
            density = 0.01  # TODO: adapting if no points found. Idea : pick some small sample of the point cloud
    else:  # density on the whole point cloud
        dist = pc_load.compute_nearest_neighbor_distance()
        density = statistics.mean(dist)

    print(bound, points_bb_np, center, dim, density, n_points)
    return bound, points_bb_np, center, dim, density, n_points


def compute_density(pc_load, center, dim_z):
    """
    Function that uses open3D to compute the density of the point cloud
    :param pc_load: the point cloud object, loaded into open3D
    :param center: the center point
    :param dim_z:
    :return:
    """
    density = []
    pt1 = [center[0] - 2, center[1] - 2, center[2] - dim_z / 2]
    pt2 = [center[0] + 2, center[1] + 2, center[2] + dim_z / 2]
    pt3 = [center[0] - 2, center[1] + 2, center[2] + dim_z / 2]
    np_points = [pt1, pt2, pt3]
    points = o3d.utility.Vector3dVector(np_points)

    crop_box = o3d.geometry.AxisAlignedBoundingBox
    crop_box = crop_box.create_from_points(points)

    point_cloud_crop = pc_load.crop(crop_box)
    dist = point_cloud_crop.compute_nearest_neighbor_distance()

    if dist:
        density = statistics.mean(dist)
    return density


# CLOUDCOMPARE FUNCTIONS________________________________________________________________________________________________
def cc_function(dest_dir, function_name, fun_txt):
    """
    Function that creates a bat file to launch CloudCompare CLI
    :param dest_dir: the folder where the bat file will be created and executed
    :param function_name: a name for the function to be executed
    :param fun_txt: the actual content of the bat file
    :return:
    """
    batpath = os.path.join(dest_dir, function_name + ".bat")
    with open(batpath, 'w') as OPATH:
        OPATH.writelines(fun_txt)
    subprocess.call([batpath])
    os.remove(batpath)


def cc_rotate_from_matrix(cloud_path, rot_matrix):
    """
    Function to apply a rotation to a point cloud, using a rotation matrix
    @param cloud_path:
    @param rot_matrix: as a numpy array (3,3)
    """
    (cloud_folder, cloud_file) = os.path.split(cloud_path)
    cc_cloud = '"' + cloud_path + '"'
    # create the text file including the transformation matrix
    text = ''
    for row in rot_matrix:
        text = text + str(row[0]) + ' ' + str(row[1]) + ' ' + str(row[2]) + ' 0' + '\n'
    text = text + '0 0 0 1'

    file_name = 'rotation.txt'
    txt_path = os.path.join(cloud_folder, file_name)
    # Write the file out again
    with open(txt_path, 'w') as file:
        file.write(text)
    cc_txt_path = '"' + txt_path + '"'

    function_name = 'rotated'
    function = ' -APPLY_TRANS ' + str(cc_txt_path)

    # prepare CloudCompare fonction
    fun_txt = 'SET MY_PATH="' + cc_path + '" \n' + '%MY_PATH% -SILENT -NO_TIMESTAMP -C_EXPORT_FMT PLY -O ' + cc_cloud + function
    cc_function(cloud_folder, function_name, fun_txt)

    os.remove(txt_path)


def cc_recolor(cloud_path, color):
    pass


def cc_sphericity_linesub(cloud_path, radius, filter_min, filter_max):
    """
    A function to compute the sphericity of the point cloud, filter it according to values, and assign a color to the remaining points
    """

    (cloud_folder, cloud_file) = os.path.split(cloud_path)
    cc_cloud = '"' + cloud_path + '"'
    function_name = 'sph'
    function = ' -FEATURE SPHERICITY ' + str(radius) + ' -FILTER_SF ' + str(float(filter_min)) + ' ' + str(
        float(filter_max))

    # Prepare CloudCompare function
    fun_txt = 'SET MY_PATH="' + cc_path + '" \n' + '%MY_PATH% -SILENT -C_EXPORT_FMT PLY -O ' + \
              cc_cloud + function
    cc_function(cloud_folder, function_name, fun_txt)


# PREPROCESSING FUNCTIONS_______________________________________________________________________________________________
def preproc_ransac_short(cloud_path, min_points, dist, obj_out_dir, cloud_out_dir):
    """
    Function that detects the main planes accross the point cloud
    :param cloud_path: (str) the path to the point cloud
    :param min_points: (int) the minimum points necessary to define a plane
    :param dist: (float) the maximum distance of a point to a candidate plane to be considered as a support for that plane
    :param obj_out_dir: (str) otutput directory for obj files
    :param cloud_out_dir: (str) output directory for ply files
    :return: (int) the number of detected planes
    """
    (cloud_dir, cloud_file) = os.path.split(cloud_path)
    cc_cloud = '"' + cloud_path + '"'
    function_name = 'ransac'
    function = ' -RANSAC SUPPORT_POINTS ' + str(int(min_points)) + ' EPSILON_ABSOLUTE ' + str(
        dist) + ' OUTPUT_INDIVIDUAL_PAIRED_CLOUD_PRIMITIVE -M_EXPORT_FMT OBJ -SAVE_MESHES -C_EXPORT_FMT PLY -SAVE_CLOUDS -MERGE_MESHES ' \
                '-SAVE_MESHES -MERGE_CLOUDS -SAVE_CLOUDS '

    # Prepare cloudcompare fonction
    fun_txt = 'SET MY_PATH="' + cc_path + '" \n' + '%MY_PATH% -SILENT -NO_TIMESTAMP -AUTO_SAVE OFF -O ' + cc_cloud + function
    cc_function(cloud_dir, function_name, fun_txt)

    # Sort files
    i = 0
    j = 0  # number of planes
    for file in os.listdir(cloud_dir):
        if file != cloud_file:
            if file.endswith('.ply'):
                if not 'MERGED' in file:
                    if 'PLANE' in file:
                        i += 1
                        name = 'plane_' + str(i) + '.ply'
                        os.rename(os.path.join(cloud_dir, file), os.path.join(cloud_out_dir, name))
                else:
                    name = 'plane_merged' + '.ply'
                    os.rename(os.path.join(cloud_dir, file), os.path.join(cloud_out_dir, name))
            if file.endswith('.obj'):
                if not 'MERGED' in file:
                    if 'PLANE' in file:
                        j += 1
                        name = 'plane_' + str(j) + '.obj'
                        os.rename(os.path.join(cloud_dir, file), os.path.join(obj_out_dir, name))
                else:
                    name = 'plane_merged' + '.obj'
                    os.rename(os.path.join(cloud_dir, file), os.path.join(obj_out_dir, name))
            if file.endswith('.bin'):
                to_rem = os.path.join(cloud_dir, file)
                os.remove(to_rem)

    return j


def preproc_floors_from_ransac_merged(ransac_cloud_path, bin_height=0.1):
    pcd = o3d.io.read_point_cloud(ransac_cloud_path)
    points_list = np.asarray(pcd.points)

    # take the z coordinate
    dist = points_list[:, 2]

    # some statistics
    nb_pts = len(dist)
    min_z = np.min(dist)
    max_z = np.max(dist)
    range_z = max_z - min_z  # range of height
    print(f'Minimum z: {min_z} m, Maximum z: {max_z} m, Range: {range_z} m')

    # histogram computation
    nb_bins = round(range_z) / bin_height
    print(f'Number of bins: {nb_bins}')
    density, bins = np.histogram(dist, bins=int(nb_bins))
    mean_peak = np.mean(density)
    print(f'Mean population: {mean_peak} points')

    # plot histogram
    _ = plt.hist(dist, bins=int(nb_bins))
    plt.axhline(1.6 * mean_peak, color='k', linestyle='dashed', linewidth=1)
    plt.show()

    loc = find_peaks(density, height=(1.6 * mean_peak, nb_pts))
    positions = []

    winner_list = loc[0]
    print(f'Winner list: {winner_list}')
    print(density)

    density_length = len(density)
    print(range(density_length))

    for winner in winner_list:
        for k in range(len(density)):
            if k == winner:
                print(k)

    plt.plot(positions)
    plt.show()


def preproc_align_cloud(cloud_path, ransac_obj_folder, ransac_cloud_folder, exclude_txt='MERGED'):
    def compute_rot_matrix(vector):
        a = vector
        b = [1, 0, 0]
        v = np.cross(a, b)
        s = np.linalg.norm(v)
        c = np.dot(a, b)
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        r = np.eye(3) + vx + np.dot(vx, vx) * (1 - c) / (s ** 2)
        return r

    obj_list = generate_list('obj', ransac_obj_folder, exclude=exclude_txt)
    results_vert = find_planes(obj_list, ransac_cloud_folder, orientation='ver_all')
    obj_list_2 = results_vert[2]

    all_x_normals = []
    all_y_normals = []
    all_z_normals = []
    # reminder : plane_data = [cx, cy, cz, w, h, vn[0], vn[1], vn[2]]

    for num, file in enumerate(obj_list_2):
        plane_data = plane_data_light(file)
        all_x_normals.append(plane_data[5])
        all_y_normals.append(plane_data[6])
        all_z_normals.append(plane_data[7])

    hist_x = np.histogram(all_x_normals, bins=200)
    number = hist_x[0]

    m = max(number)
    index = [i for i, j in enumerate(number) if j == m]
    index = index[0]

    bins = hist_x[1]
    vx_to_find = bins[index]

    index_in_initial_list = [i for i, j in enumerate(all_x_normals) if vx_to_find - 0.1 < j < vx_to_find + 0.1]
    index_in_initial_list = index_in_initial_list[0]

    vx = all_x_normals[index_in_initial_list]
    vy = all_y_normals[index_in_initial_list]
    vector = [vx, vy, 0]
    r = compute_rot_matrix(vector)
    print(r)

    cc_rotate_from_matrix(cloud_path, r)
    return r


# NORMAL FUNCTIONS____________________________________________________________________________________________
# Function to normalize a vector
def normalize(vector):
    return vector / np.linalg.norm(vector)


# Function to check if two normals are close
def are_normals_close(normal1, normal2, threshold=0.996):  # Threshold for dot product similarity (cosine similarity)
    normal1_normalized = normalize(np.array(normal1))
    normal2_normalized = normalize(np.array(normal2))

    dot_product = np.dot(normal1_normalized, normal2_normalized)

    # If the dot product is close to 1, the vectors are almost in the same direction
    return dot_product > threshold


# Remove duplicate normals based on closeness
def remove_duplicate_normals(normals_list, threshold=0.996):
    unique_normals = []

    for normal in normals_list:
        if not any(are_normals_close(normal, unique_normal, threshold) for unique_normal in unique_normals):
            unique_normals.append(normal)

    return unique_normals


# 2D RENDERING FUNCTIONS________________________________________________________________________________________________
def raster_single_bound(cloud_path, grid_step, dir_cc, bound_pc, xray=True):
    # File names and paths
    (cloud_dir, cloudname) = os.path.split(cloud_path)
    cc_cloud = '"' + cloud_path + '"'
    cc_cloud_lim = '"' + bound_pc + '"'
    proj = ''
    if not xray:
        proj = ' -SF_PROJ MAX -PROJ MAX'

    function_name = 'raster'

    function = ' -AUTO_SAVE OFF -NO_TIMESTAMP -MERGE_CLOUDS -RASTERIZE' + proj + ' -VERT_DIR ' + str(
        dir_cc) + ' -GRID_STEP ' \
               + str(grid_step) + ' -OUTPUT_RASTER_RGB '

    # Prepare CloudCompare function
    fun_txt = 'SET MY_PATH="' + cc_path + '" \n' + '%MY_PATH% -SILENT -O ' + cc_cloud + ' -O ' + cc_cloud_lim + function
    cc_function(cloud_dir, function_name, fun_txt)


def render_plane_in_cloud(plane_cloud_path, cloud_path, grid_step):
    # File names and paths
    (cloud_dir, cloudname) = os.path.split(cloud_path)
    cc_cloud = '"' + cloud_path + '"'
    cc_mesh = '"' + plane_cloud_path + '"'

    function_name = 'raster'
    function = ' -AUTO_SAVE OFF -SF_CONVERT_TO_RGB FALSE -MERGE_CLOUDS -RASTERIZE -GRID_STEP ' + str(
        grid_step) + ' -OUTPUT_RASTER_RGB'

    # Prepare CloudCompare function
    fun_txt = 'SET MY_PATH="' + cc_path + '" \n' + '%MY_PATH% -SILENT -O ' + cc_cloud + ' -REMOVE_ALL_SFS -O  ' + cc_mesh + function
    cc_function(cloud_dir, function_name, fun_txt)


def create_pcv(cloud_path):
    # File names and paths
    (cloud_dir, cloudname) = os.path.split(cloud_path)
    cc_cloud = '"' + cloud_path + '"'

    function_name = 'pcv'
    function = ' -AUTO_SAVE OFF -C_EXPORT_FMT PLY -PCV -SF_CONVERT_TO_RGB FALSE -SAVE_CLOUDS'

    # Prepare CloudCompare function
    fun_txt = 'SET MY_PATH="' + cc_path + '" \n' + '%MY_PATH% -SILENT -O ' + cc_cloud + function
    cc_function(cloud_dir, function_name, fun_txt)


def render_planar_segment(cloud_path, grid_step):
    # File names and paths
    (cloud_dir, cloudname) = os.path.split(cloud_path)
    cc_cloud = '"' + cloud_path + '"'
    proj = ' -SF_PROJ MAX -PROJ MAX'

    function_name = 'raster'

    function = ' -AUTO_SAVE OFF -BEST_FIT_PLANE -MAKE_HORIZ -SF_CONVERT_TO_RGB FALSE -RASTERIZE' + proj + ' -GRID_STEP ' \
               + str(grid_step) + ' -OUTPUT_RASTER_RGB'

    # Prepare CloudCompare function
    fun_txt = 'SET MY_PATH="' + cc_path + '" \n' + '%MY_PATH% -SILENT -O ' + cc_cloud + function
    cc_function(cloud_dir, function_name, fun_txt)


def raster_all_bound(cloud_path, grid_step, bound_pc, xray=True, sf=False):
    """
    Function for generating images from a point cloud, from all possible points of view
    :param cloud_path: (str) path to the point cloud to render
    :param grid_step: (float) size of one pixel, in m
    :param bound_pc: (str) path to the bounding box point cloud
    :param xray: (bool) whether to render a xray view
    :param sf: (bool) whether to render one of the scalar fields
    :return:
    """
    # File names and paths
    (cloud_folder, cloudname) = os.path.split(cloud_path)
    cc_cloud = '"' + cloud_path + '"'
    cc_cloud_lim = '"' + bound_pc + '"'
    proj = ''
    if not xray:  # Note: the value is AVERAGE by default --> Thus resulting in kind of an xray view
        proj = ' -SF_PROJ MAX -PROJ MAX'

    function_name = 'raster'
    # See memo for understanding views logic
    # order of generated views is : front / right / top / back / left

    function = ' -AUTO_SAVE OFF -MERGE_CLOUDS -RASTERIZE' + proj + ' -VERT_DIR 0 -GRID_STEP ' \
               + str(grid_step) + ' -OUTPUT_RASTER_RGB -RASTERIZE' + proj + ' -VERT_DIR 1 -GRID_STEP ' \
               + str(grid_step) + ' -OUTPUT_RASTER_RGB -RASTERIZE' + proj + ' -VERT_DIR 2 -GRID_STEP ' \
               + str(grid_step) + ' -OUTPUT_RASTER_RGB'
    if not xray:  # the last two views do not need to be generated for xray
        proj = ' -SF_PROJ MIN -PROJ MIN'
        function += ' -RASTERIZE' + proj + ' -VERT_DIR 0 -GRID_STEP ' + str(grid_step) + ' -OUTPUT_RASTER_RGB ' \
                                                                                         ' -RASTERIZE' + proj + ' -VERT_DIR 1 -GRID_STEP ' + str(
            grid_step) + ' -OUTPUT_RASTER_RGB '

    # Prepare CloudCompare function
    fun_txt = 'SET MY_PATH="' + cc_path + '" \n' + '%MY_PATH% -SILENT -O ' + cc_cloud + ' -O ' + cc_cloud_lim + function
    cc_function(cloud_folder, function_name, fun_txt)


# FLOORPLANS____________________________________________________________________________________________________________
def floorplan_advanced(cloud_path, bound_pc, grid_step1, floor_level, ceiling_level, method='verticality'):
    # File names and paths
    (cloud_dir, cloudname) = os.path.split(cloud_path)
    cc_cloud = '"' + cloud_path + '"'
    cc_cloud_lim = '"' + bound_pc + '"'

    function_name = 'floor'
    function = ' -AUTO_SAVE OFF -NO_TIMESTAMP '

    if method == 'verticality':
        pass

    # projection option
    proj = ' -SF_PROJ MAX -PROJ MAX'

    # OPERATION 1: rasterize and export cloud with population count
    function += '-RASTERIZE' + proj + ' -GRID_STEP ' \
                + str(grid_step1) + ' -OUTPUT_CLOUD -SAVE_CLOUDS'

    # Prepare CloudCompare function
    fun_txt = 'SET MY_PATH="' + cc_path + '" \n' + '%MY_PATH% -SILENT -O ' + cc_cloud + function
    cc_function(cloud_dir, function_name, fun_txt)


# CUT SECTIONS__________________________________________________________________________________________________________
def cut_sections_write_xml(file_name, center_x, size_x, center_y, size_y, center_z, size_z, repeat, gap, output_dir):
    # Folder params

    # get the repetition dimension and adapt for cloudcompare
    if repeat == 'x':
        param_rep = 0
    elif repeat == 'y':
        param_rep = 1
    else:
        param_rep = 2

    # Write XML file
    text = '<CloudCompare> \n <BoxThickness x="' + str(size_x) + '" y ="' + str(size_y) + '" z ="' + \
           str(size_z) + '"/> \n <BoxCenter x="' + str(center_x) + '" y ="' + str(center_y) + '" z ="' \
           + str(center_z) + '"/> \n <RepeatDim>' + str(param_rep) + '</RepeatDim> \n <RepeatGap>' + str(gap) + \
           '</RepeatGap> \n' \
           '<OutputFilePath>' + output_dir + '</OutputFilePath> \n </CloudCompare>'

    xml_path = os.path.join(output_dir, file_name)
    # Write the file out again
    with open(xml_path, 'w') as file:
        file.write(text)
    return xml_path


def cut_sections_all_dirs(cloud_path, center_x, size_x, center_y, size_y, center_z, size_z, gap,
                          output_dir_x, output_dir_y, output_dir_z):
    # Folder params
    (cloud_folder, cloud_file) = os.path.split(cloud_path)
    cc_cloud = '"' + cloud_path + '"'

    function_txt = ''
    function_name = 'cut'

    # Write XML file
    xml_files = []

    output_folders = [output_dir_x, output_dir_y, output_dir_z]
    orientation = ['x', 'y', 'z']
    for i in range(3):
        if i == 0:
            xml_path = cut_sections_write_xml('cut_x.xml', center_x, 0.1, center_y, size_y, center_z, size_z,
                                              orientation[i],
                                              gap, output_folders[i])
        if i == 1:
            xml_path = cut_sections_write_xml('cut_y.xml', center_x, size_x, center_y, 0.1, center_z, size_z,
                                              orientation[i],
                                              gap, output_folders[i])
        if i == 2:
            xml_path = cut_sections_write_xml('cut_z.xml', center_x, size_x, center_y, size_y, center_z, 0.1,
                                              orientation[i],
                                              gap, output_folders[i])

        cc_xml = '"' + xml_path + '"'
        function_txt = function_txt + ' -CROSS_SECTION ' + cc_xml
        xml_files.append(xml_path)

    fun_txt = 'SET MY_PATH="' + cc_path + '" \n' + '%MY_PATH% -SILENT -C_EXPORT_FMT PLY -NO_TIMESTAMP  -O ' \
              + cc_cloud + function_txt

    cc_function(cloud_folder, function_name, fun_txt)
    os.remove(xml_path)


# ANALYTIC FUNCTIONS____________________________________________________________________________________________________
def cc_planes_to_build_dist_list(cloud_path, obj_list, cloud_out_folder, dist=0.1, span=0.03):
    """A function that open all plane meshes in a directory, compares it to the original building cloud

    Two inputs:
        folder -- the directory containing the .obj planes
        dist -- criteria of comparison (float)

    outputs individual point planes that keep color information

    """
    (cloud_dir, cloud_file) = os.path.split(cloud_path)
    cc_cloud = '"' + cloud_path + '"'
    function_name = 'c2m'

    function_load_compare = ''
    for i, obj_file in enumerate(obj_list):
        mesh_name = obj_file
        cc_mesh = '"' + mesh_name + '"'
        function_load_compare = ' -O ' + cc_mesh + ' -C2M_DIST -MAX_DIST ' + str(dist) + ' -FILTER_SF ' + str(
            -float(dist) + span) + ' ' + str(
            float(dist) - span) + ' -SAVE_CLOUDS -CLEAR_MESHES '

        # Prepare CloudCompare fonction
        fun_txt = 'SET MY_PATH="' + cc_path + '" \n' + '%MY_PATH% -SILENT -C_EXPORT_FMT LAS -AUTO_SAVE OFF -O ' + cc_cloud + function_load_compare

        # Execute function
        cc_function(cloud_dir, function_name, fun_txt)

    # sort files
    i = 0
    for file in os.listdir(cloud_dir):
        if file.endswith('.las'):
            if 'C2M_DIST' in file:
                i += 1
                name = 'segment_' + str(i) + '.las'
                os.rename(os.path.join(cloud_dir, file), os.path.join(cloud_out_folder, name))

    return i


def plane_data_light(obj_path):
    """
    Function to get the size, orientation and position of a plane obj
    Returns an 'plane_data' array
    cx, cy, cz -- x, y ,z coord of plane center
    w, h -- width and height of the plane
    vn -- vector representing the normal of the plane
    @parameters:
        obj_path -- path to a specific plane in obj format
    """
    v = []
    vn = []
    mylines = []
    with open(obj_path, 'rt') as myfile:
        for myline in myfile:  # For each line, read it to a string
            mylines.append(myline)
            if 'v ' in myline:
                myline = myline[2:-2]
                v.append(myline.split(' '))
            if 'vn' in myline:
                myline = myline[3:-2]
                vn = myline.split(' ')

    v1 = v[0]
    v2 = v[1]
    v3 = v[2]
    v4 = v[3]

    x = [float(v1[0]), float(v2[0]), float(v3[0]), float(v4[0])]
    y = [float(v1[1]), float(v2[1]), float(v3[1]), float(v4[1])]
    z = [float(v1[2]), float(v2[2]), float(v3[2]), float(v4[2])]
    cx = (x[0] + x[2]) / 2
    cy = (y[0] + y[2]) / 2
    cz = (z[0] + z[2]) / 2
    w = math.sqrt((x[0] - x[1]) ** 2 + (y[0] - y[1]) ** 2 + (z[0] - z[1]) ** 2)
    h = math.sqrt((x[1] - x[2]) ** 2 + (y[1] - y[2]) ** 2 + (z[1] - z[2]) ** 2)
    vn = [float(vn[0]), float(vn[1]), float(vn[2])]
    plane_data = [cx, cy, cz, w, h, vn[0], vn[1], vn[2]]

    return plane_data


def plane_data_complete(obj_path, cloud_path):
    """
    Function to get the size, orientation and position of a plane obj, plus additional information gathered from the
    corresponding point cloud By default, it is expected that the point cloud has the same name as the obj
    """
    (obj_folder, obj_file) = os.path.split(obj_path)
    # Read obj plane info
    v = []
    vn = []
    mylines = []
    with open(obj_path, 'rt') as myfile:
        for myline in myfile:  # For each line, read it to a string
            mylines.append(myline)
            if 'v ' in myline:
                myline = myline[2:-2]
                v.append(myline.split(' '))
            if 'vn' in myline:
                myline = myline[3:-2]
                vn = myline.split(' ')

    v1 = v[0]
    v2 = v[1]
    v3 = v[2]
    v4 = v[3]

    x = [float(v1[0]), float(v2[0]), float(v3[0]), float(v4[0])]
    y = [float(v1[1]), float(v2[1]), float(v3[1]), float(v4[1])]
    z = [float(v1[2]), float(v2[2]), float(v3[2]), float(v4[2])]
    cx = (x[0] + x[2]) / 2
    cy = (y[0] + y[2]) / 2
    cz = (z[0] + z[2]) / 2
    dim1 = math.sqrt((x[0] - x[1]) ** 2 + (y[0] - y[1]) ** 2 + (z[0] - z[1]) ** 2)
    dim2 = math.sqrt((x[1] - x[2]) ** 2 + (y[1] - y[2]) ** 2 + (z[1] - z[2]) ** 2)
    area = dim1 * dim2
    vn = [float(vn[0]), float(vn[1]), float(vn[2])]
    plane_data = [x, y, z, cx, cy, cz, dim1, dim2, area, vn[0], vn[1], vn[2]]

    pcd = o3d.io.read_point_cloud(cloud_path)
    points_list = np.asarray(pcd.points)
    color_list = np.asarray(pcd.colors)
    n_points = len(points_list[:, 1])
    average_color = [math.sqrt(np.mean(color_list[:, 0] ** 2)), math.sqrt(np.mean(color_list[:, 1] ** 2)),
                     math.sqrt(np.mean(color_list[:, 2] ** 2))]

    plane_data.append(n_points)
    plane_data.extend(average_color)

    return plane_data


def plane_orientation(vn):
    """
    Determines the orientation of the plane based on its normal vector.
    Returns 'vertical', 'horizontal', or 'oblique'.

    @parameters:
        vn -- normal vector of the plane (a list or tuple with 3 components)
    """
    z_component = vn[2]

    if z_component >= 0.98:
        return 'horizontal'
    elif -0.06 <= z_component <= 0.06:
        return 'vertical'
    else:
        return 'oblique'


def filter_planes(plane_list, desired_orientation, min_area):
    """
    Filters a list of planes based on desired orientation and minimum area.
    Returns a sublist of planes that match the criteria.

    @parameters:
        plane_list -- list of obj file paths
        desired_orientation -- 'vertical', 'horizontal', or 'oblique'
        min_area -- minimum area of the plane
    """
    filtered_planes = []

    for obj_path in plane_list:
        plane_data = plane_data_light(obj_path)
        area = plane_data[3]*plane_data[4]
        vn = plane_data[5:8]
        print(vn)
        orientation = plane_orientation(vn)

        if orientation == desired_orientation and area >= min_area:
            filtered_planes.append(obj_path)

    return filtered_planes




# DEPRECIATED FUNCTIONS_________________________________________________________________________________________________
def find_planes(obj_list, cloud_folder, cloud_suffix='', orientation='all', size_absolute='all', size_criteria=10,
                size_comparative='all', location_relative='all', location_option='all_winners', uniformity='all'):
    """
    A central function to detect planes that respect a criterion, or a series of criteria
    """

    def get_plane_data(file):
        obj_folder, obj_file = os.path.split(file)
        ply_file_name = obj_file[:-4] + cloud_suffix + '.ply'
        ply_file_loc = os.path.join(cloud_folder, ply_file_name)
        return plane_data_complete(file, ply_file_loc)

    def orientation_filter(plane_prop):
        if orientation == 'hor' and abs(plane_prop[11]) > 0.998:
            return True
        if orientation == 'ver_x' and abs(plane_prop[9]) > 0.998:
            return True
        if orientation == 'ver_y' and abs(plane_prop[10]) > 0.998:
            return True
        if orientation == 'ver_all' and abs(plane_prop[11]) < 0.02:
            return True
        if orientation == 'obl' and all(abs(plane_prop[i]) < 0.998 for i in [9, 10, 11]):
            return True
        return False

    def size_filter(area, dim1, dim2):
        if size_absolute == 'area_greater_than' and area > size_criteria:
            return True
        if size_absolute == 'area_smaller_than' and area < size_criteria:
            return True
        if size_absolute == 'largest_dim_larger_than' and (dim1 > size_criteria or dim2 > size_criteria):
            return True
        return False

    def comparative_size_filter(area, area_list):
        if size_comparative == 'area_largest' and area > area_list[0]:
            area_list[0] = area
            return True
        if size_comparative == 'area_smallest' and area < area_list[0]:
            area_list[0] = area
            return True
        return False

    def location_analysis(c_pos_list, v_pos_list, loc_criteria):
        loc_results_list = []
        if location_option != 'all_winners':
            min_to_beat = max(v_pos_list[0])
            for i, pos in enumerate(c_pos_list):
                if pos < min_to_beat:
                    min_to_beat = pos
                    loc_results_list = [i]
        else:
            for i in range(len(c_pos_list)):
                if all(c_pos_list[i] >= c_pos_list[j] for j in range(len(c_pos_list)) if i != j):
                    loc_results_list.append(i)
        return loc_results_list

    # Initialize lists
    all_planes_data = [get_plane_data(file) for file in obj_list]
    orientation_results_list = [obj_list[i] for i, plane_prop in enumerate(all_planes_data) if orientation_filter(plane_prop)]

    if orientation_results_list:
        size_results_list = []
        area_list = [0]
        dim_list = [0]
        for i, plane_prop in enumerate(all_planes_data):
            area, dim1, dim2 = plane_prop[8], plane_prop[6], plane_prop[7]
            if size_filter(area, dim1, dim2) or comparative_size_filter(area, area_list):
                size_results_list.append(obj_list[i])
        cross_results_list = list(set(orientation_results_list).intersection(size_results_list))
    else:
        cross_results_list = obj_list

    loc_results_list = location_analysis(
        [[plane_prop[i] for plane_prop in all_planes_data] for i in range(3, 6)],
        [[max(plane_prop[i]) for plane_prop in all_planes_data] for i in range(6)],
        location_relative
    ) if location_relative != 'all' else cross_results_list

    return [obj_list, [orientation, size_absolute, size_comparative, location_relative, uniformity], loc_results_list]


def preproc_z_csv(ransac_cloud_folder):  # DEPRECIATED
    for ply_file in os.listdir(ransac_cloud_folder):
        if 'merged' in ply_file:
            cloud_path = os.path.join(ransac_cloud_folder, ply_file)
    (cloud_folder, cloud_file) = os.path.split(cloud_path)
    function_name = 'dist_st'
    cc_cloud = '"' + cloud_path + '"'
    # function to create a CSV with Z coordinates of all points of a point cloud
    function = ' -COORD_TO_SF Z -SAVE_CLOUDS'

    # Prepare CloudCompare fonction
    fun_txt = 'SET MY_PATH="' + cc_path + '" \n' + '%MY_PATH% -SILENT -C_EXPORT_FMT ASC -ADD_HEADER -EXT csv -AUTO_SAVE OFF -NO_TIMESTAMP  -O ' + cc_cloud + function
    batpath = os.path.join(cloud_folder, function_name + ".bat")
    with open(batpath, 'w') as OPATH:
        OPATH.writelines(fun_txt)

    # Execute function
    subprocess.call([batpath])
    os.remove(batpath)

    # find the csv file
    csv_file = cloud_path[:-4] + '_Z_TO_SF.csv'

    return csv_file


def preproc_floors_from_csv(csv_file):  # DEPRECIATED
    # read the csv file
    data = pd.read_csv(csv_file, sep=' ')
    dist = data['Coord._Z']

    # evaluate the range of z coordinates

    # create an histogram of z coordinates
    z_hist = np.histogram(dist, bins=1000)

    # plot histogram
    _ = plt.hist(dist, bins=1000)
    plt.show()

    # compute the number of floors
    n_floors = 0  # TODO: modify to actual value

    return n_floors


"""
======================================================================================
ALGEBRA
======================================================================================
"""


# definition of rotation matrices, useful for camera operations
def rot_x_matrix(angle):
    matrix = np.asarray([[1, 0, 0, 0],
                         [0, math.cos(math.radians(angle)), -math.sin(math.radians(angle)), 0],
                         [0, math.sin(math.radians(angle)), math.cos(math.radians(angle)), 0],
                         [0, 0, 0, 1]])
    return matrix


def rot_y_matrix(angle):
    matrix = np.asarray([[math.cos(math.radians(angle)), 0, math.sin(math.radians(angle)), 0],
                         [0, 1, 0, 0],
                         [-math.sin(math.radians(angle)), 0, math.cos(math.radians(angle)), 0],
                         [0, 0, 0, 1]])
    return matrix


def rot_z_matrix(angle):
    matrix = np.asarray([[math.cos(math.radians(angle)), -math.sin(math.radians(angle)), 0, 0],
                         [math.sin(math.radians(angle)), math.cos(math.radians(angle)), 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
    return matrix


def front_mat():
    matrix = rot_x_matrix(-90)
    inv_matrix = rot_x_matrix(90)
    return matrix, inv_matrix


def back_mat():
    matrix1 = rot_x_matrix(-90)
    matrix2 = rot_y_matrix(180)
    final_matrix = matrix2 @ matrix1
    inv_matrix1 = rot_y_matrix(-180)
    inv_matrix2 = rot_x_matrix(90)
    final_inv_matrix = inv_matrix2 @ inv_matrix1
    return final_matrix, final_inv_matrix


def right_mat():
    matrix1 = rot_x_matrix(-90)
    matrix2 = rot_y_matrix(-90)
    final_matrix = matrix2 @ matrix1
    inv_matrix1 = rot_y_matrix(90)
    inv_matrix2 = rot_x_matrix(90)
    final_inv_matrix = inv_matrix2 @ inv_matrix1
    return final_matrix, final_inv_matrix


def left_mat():
    matrix1 = rot_x_matrix(-90)
    matrix2 = rot_y_matrix(90)
    final_matrix = matrix2 @ matrix1
    inv_matrix1 = rot_y_matrix(-90)
    inv_matrix2 = rot_x_matrix(90)
    final_inv_matrix = inv_matrix2 @ inv_matrix1
    return final_matrix, final_inv_matrix


def iso2_mat():
    matrix1 = rot_x_matrix(60)
    matrix2 = rot_y_matrix(-20)
    matrix3 = rot_z_matrix(190)
    final_matrix1 = matrix3 @ matrix2
    final_matrix = final_matrix1 @ matrix1
    inv_matrix1 = rot_z_matrix(-190)
    inv_matrix2 = rot_y_matrix(20)
    inv_matrix3 = rot_x_matrix(-60)
    final_inv_matrix1 = inv_matrix3 @ inv_matrix2
    final_inv_matrix = final_inv_matrix1 @ inv_matrix1
    return final_matrix, final_inv_matrix


def iso1_mat():
    matrix1 = rot_x_matrix(-60)
    matrix2 = rot_y_matrix(-20)
    matrix3 = rot_z_matrix(-10)
    final_matrix1 = matrix3 @ matrix2
    final_matrix = final_matrix1 @ matrix1
    inv_matrix1 = rot_z_matrix(10)
    inv_matrix2 = rot_y_matrix(20)
    inv_matrix3 = rot_x_matrix(60)
    final_inv_matrix1 = inv_matrix3 @ inv_matrix2
    final_inv_matrix = final_inv_matrix1 @ inv_matrix1
    return final_matrix, final_inv_matrix


def name_to_matrix(orientation):
    if orientation == 'iso_front':
        trans_init, inv_trans = iso1_mat()
    elif orientation == 'iso_back':
        trans_init, inv_trans = iso2_mat()
    elif orientation == 'left':
        trans_init, inv_trans = left_mat()
    elif orientation == 'right':
        trans_init, inv_trans = right_mat()
    elif orientation == 'front':
        trans_init, inv_trans = front_mat()
    elif orientation == 'back':
        trans_init, inv_trans = back_mat()

    return trans_init, inv_trans


def basic_vis_creation(load, orientation, p_size=1.5, back_color=[1, 1, 1]):
    """A function that creates the basic environment for creating things with open3D
            @ parameters :
                pcd_load -- a point cloud loaded into open3D
                orientation -- orientation of the camera; can be 'top', ...
                p_size -- size of points
                back_color -- background color
    """
    if orientation != 'top':
        trans_init, inv_trans = name_to_matrix(orientation)
        load.transform(trans_init)

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(load)
    opt = vis.get_render_option()
    opt.point_size = p_size
    opt.mesh_show_back_face = True
    opt.background_color = np.asarray(back_color)
    ctr = vis.get_view_control()
    ctr.change_field_of_view(step=-90)

    vis.poll_events()
    vis.update_renderer()
    vis.run()

    return vis, opt, ctr

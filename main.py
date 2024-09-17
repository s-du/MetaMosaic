import os
import shutil
from engine import cloudcompare_part as proc
from engine import agisoft_part as agi
import open3d as o3d

"""
EEHB2024 Paper code
The goal is here to start from drone pictures and process them automatically:
- 3D mesh creation (Agisoft)
- Point cloud sampling (Agisoft)
- Ransac plane detection (CloudCompare)
- Orthomosaic creation from the model, from each plane normal (Agisoft)
- (Optional) Orthoviews creation from point cloud (cloudcompare)

Authors: Samuel Dubois, Louis Vandenabeele
"""

# PATHS
INPUT_IMG_DIR = 'case_mozet'  # fill the input images folder
PROCESSED_DATA_DIR = os.path.join(INPUT_IMG_DIR, 'MetaMosaic')
OUT_ORTHO_DIR = os.path.join(PROCESSED_DATA_DIR, 'orthomosaics')  # Output images directory
OUT_AGISOFT_DIR = os.path.join(PROCESSED_DATA_DIR, 'agisoft_outputs')
OUT_CC_DIR = os.path.join(PROCESSED_DATA_DIR, 'cloudcompare_outputs')
LOG_PATH = os.path.join(PROCESSED_DATA_DIR, 'log.txt')
AGISOFT_PC_PATH = os.path.join(OUT_AGISOFT_DIR, 'point_cloud.ply')

proc.new_dir(PROCESSED_DATA_DIR)
proc.new_dir(OUT_ORTHO_DIR)
proc.new_dir(OUT_AGISOFT_DIR)
proc.new_dir(OUT_CC_DIR)

# PARAMETERS
# Ransac detection
MIN_RANSAC_FACTOR = 300  # (Total number of points / MIN_RANSAC_FACTOR) gives the minimum amount of points to define a
RANSAC_DIST = 0.02  # maximum distance for a point to be considered belonging to a plane

# Ortho
PIXEL_SIZE = 0.003 # if 0, Orthomosaics will be created with the highest possible resolution

# create empty log for storing the infos about created data
log_txt = ''

if __name__ == "__main__":
    # user options
    do_agisoft_part1_rgb = False
    do_agisoft_part1_th = False
    do_cc_part1 = True
    do_agisoft_part2_rgb = False
    do_agisoft_part2_th = True
    do_cc_part2 = False

    """
    °°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
    *******************************  AGISOFT (PART1)  ****************************************
    Model creation from input images + subsampled point cloud export
    °°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
    """
    # TODO Add YOLO part
    if do_agisoft_part1_rgb:
        # run method
        computed_pixel_size = agi.run_agisoft_rgb_only_part1(INPUT_IMG_DIR, OUT_AGISOFT_DIR)
        model_texture_res = computed_pixel_size

    elif do_agisoft_part1_th:
        computed_pixel_size = agi.run_agisoft_thermal_part1(INPUT_IMG_DIR, OUT_AGISOFT_DIR)
        model_texture_res = computed_pixel_size

    """
    °°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
    *******************************  CLOUDCOMPARE PROCESSING (PART1) *************************
    RANSAC Based on the point cloud extracted from Agisoft
    °°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
    """
    if do_cc_part1:
        # 1. BASIC DATA ____________________________________________________________________________________________________

        # read full high definition point cloud (using open3d)
        sub_sampled = False
        print('Reading the point cloud!')
        pc_load = o3d.io.read_point_cloud(AGISOFT_PC_PATH)
        print('Pc loaded!')

        # compute some very basic properties
        bound_pc_path = os.path.join(OUT_CC_DIR, "pc_limits.ply")
        bound, bound_points, center, dim, density, n_points = proc.compute_basic_properties(pc_load, save_bound_pc=True,
                                                                                            output_path_bound_pc=bound_pc_path)

        print(f'The point cloud density is: {density:.3f}')

        # add data to log
        log_txt += f'°°°Original point cloud°°°\n' \
                   f'Properties: \n' \
                   f'\t-center location: X:{center[0]:.2f}, Y:{center[1]:.2f}, Z:{center[2]:.2f} \n' \
                   f'\t-number of points: {n_points} points \n' \
                   f'\t-density: {density:.3f} m between points on average \n'
                   
        for bound_point in bound_points:
            log_txt += f'corners: \n' \
            f'{bound_point} \n' 

        # 2. RANSAC DETECTION __________________________________________________________________________________
        print('Launching RANSAC detection...')

        # create RANSAC directories
        ransac_cloud_folder = os.path.join(OUT_CC_DIR, 'RANSAC_pc')
        proc.new_dir(ransac_cloud_folder)

        ransac_obj_folder = os.path.join(OUT_CC_DIR, 'RANSAC_meshes')
        proc.new_dir(ransac_obj_folder)

        # fixing RANSAC Parameters
        min_points = n_points / MIN_RANSAC_FACTOR

        n_planes = proc.preproc_ransac_short(AGISOFT_PC_PATH, min_points, RANSAC_DIST, ransac_obj_folder,
                                             ransac_cloud_folder)
        log_txt += f'\n°°°Dataset: Ransac operations°°° \n' \
                   f'Properties: \n' \
                   f'\t-number of planes detected: {n_planes} planes \n' \
                   f'\t-location on HDD of Ransac planes as obj files: {ransac_obj_folder}\n' \
                   f'\t-location on HDD of Ransac planes as point cloud objects: {ransac_cloud_folder}\n' \
 \
            # 3. ORIENT THE POINT CLOUD PERPENDICULAR TO AXIS_______________________________________________________
        """
        print('Orienting the point cloud perpendicular to the axes...')
        R = proc.preproc_align_cloud(test_pc_path, bound_pc_path, ransac_obj_folder, ransac_cloud_folder)
        print(f'The point cloud has been rotated with {R} matrix...')
    
        transformed_path = proc.find_substring('TRANSFORMED', test_pc_dir)
        transformed_bound_path = proc.find_substring('TRANSFORMED', processed_data_dir)
        """

    """
    °°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
    *******************************  AGISOFT PROCESSING (PART2) ******************************
    CREATE ORTHOS
    °°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
    """
    if do_agisoft_part2_rgb or do_agisoft_part2_th:

        # create full plan list
        plane_list = proc.generate_list('obj', ransac_obj_folder, exclude='merged')
        # print all plane data
        for plane in plane_list:
            plane_data = proc.plane_data_light(plane)
            normal = [plane_data[5], plane_data[6], plane_data[7]]
            print(f'Plane:{plane}, Normal: {normal}')

        # find vertical planes
        ver_planes = proc.filter_planes(plane_list, 'vertical', 5)

        # get true normal from support points
        normals_list = []
        ver_plane_pc_list = []
        for ver_pl in ver_planes:
            _, file = os.path.split(ver_pl)
            ver_pl_pc = os.path.join(ransac_cloud_folder, file[:-4] + '.ply')
            ver_plane_pc_list.append(ver_pl_pc)

        for plane_pc in ver_plane_pc_list:
            pc_load = o3d.io.read_point_cloud(plane_pc)
            normal = proc.get_average_normal(pc_load)
            normals_list.append(normal)
        print(normals_list)

        """
        # get normals direction
        normals_list = []
        for plane in ver_planes:
            plane_data = proc.plane_data_light(plane)
            normal = [plane_data[5], plane_data[6], plane_data[7]]
            normals_list.append(normal)
            print(f'Plane:{plane}, Normal: {normal}, Vertical!')
        print(normals_list)
        """

        # remove duplicate normals
        filtered_normals = proc.remove_duplicate_normals(normals_list)
        print(filtered_normals)

        # Start the log entry with a header
        log_txt += '\n°°° Dataset: Plane normals °°°\n'

        # Iterate over the filtered normals and append each one to the log
        for normal in filtered_normals:
            log_txt += f'Normal: {normal}\n'  # Each normal is added on a new line


        # run agisoft for ortho creation
        psx_path = os.path.join(OUT_AGISOFT_DIR, 'agisoft_test2.psx')
        if do_agisoft_part2_rgb:
            orthos_data = agi.run_agisoft_rgb_only_part2(psx_path, filtered_normals, OUT_CC_DIR, pixel_size=PIXEL_SIZE)
        elif do_agisoft_part2_th:
            orthos_data = agi.run_agisoft_thermal_part2(psx_path, filtered_normals, OUT_CC_DIR, pixel_size=PIXEL_SIZE)

    """
    °°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
    *******************************  CLOUDCOMPARE PROCESSING (PART2)  ************************
    °°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
    """
    if do_cc_part2:
        print('Launching planar views creation...')

        # create new directory for results
        v_planes_img_dir = os.path.join(OUT_ORTHO_DIR, 'facade_planes_views')
        v_planes_pc_dir = os.path.join(OUT_CC_DIR, 'facade_planes_pc')
        proc.new_dir(v_planes_img_dir)
        proc.new_dir(v_planes_pc_dir)

        # create new_clouds from the detected planes (the original point cloud is segmented around the plane)
        n_v_elements = proc.cc_planes_to_build_dist_list(AGISOFT_PC_PATH, plane_list, v_planes_pc_dir)

        # computing the properties for each new point cloud --> Useful to place the images on the entire point cloud
        new_pc_list = proc.generate_list('.las', v_planes_pc_dir)
        # TODO: continue here

        # rendering each element
        list_h_planes_pc = proc.generate_list('.las', v_planes_pc_dir)
        res = round(density * 4, 3) * 1000
        for cloud in list_h_planes_pc:
            proc.render_planar_segment(cloud, res / 1000)
            for file in os.listdir(v_planes_pc_dir):
                if file.endswith('.tif'):
                    os.rename(os.path.join(v_planes_pc_dir, file), os.path.join(v_planes_img_dir, file))

    """
    °°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
    *******************************  LOGGING  ************************************************
    °°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
    """
    # write log text
    with open(LOG_PATH, 'w') as f:
        f.write(log_txt)

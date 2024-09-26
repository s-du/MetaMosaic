import os
import resources as res
import sys
import math
import numpy as np

if getattr(sys, 'frozen', False) and hasattr(sys, '_MEIPASS'):
    print('running in a PyInstaller bundle')

else:
    print('running in a normal Python process')
    # if run in Python process, then the app needs to check if Agisoft is installed
    import subprocess
    import pkg_resources


    def install_agisoft_module():
        # install Metashape module if necessary
        def install(package):
            subprocess.check_call([sys.executable, "-m", "pip", "install", package])

        metashape_module = res.find('other/Metashape-2.1.2-cp37.cp38.cp39.cp310.cp311-none-win_amd64.whl')
        install(metashape_module)


    # check if module is installed
    required = {'metashape'}
    installed = {pkg.key for pkg in pkg_resources.working_set}
    print(installed)
    missing = required - installed
    if missing:
        print(r"Ok let's intall Agisoft!")
        install_agisoft_module()

import Metashape

# general parameters
DOWN_SCALE = 2  # downscale factor for image alignment

# paths
basepath = os.path.dirname(__file__)

"""
SIMPLE METHODS_________________________________________________________________________________________________________
"""


def batch_process_pictures(img_folder, backup_folder, processing_type):
    for img in os.listdir(img_folder):
        img_path = os.path.join(img_folder, img)

        # TODO complete


def run_agisoft_rgb_only_part1(img_folder, output_folder, quality='low', opt_limit_poly=0, pixel_size=0,
                               start_with_psx=False):
    def pre_cleaning(chk):
        '''
        A function to do the precleaning after bundle ajustment
        :param chk: input chunk
        :return:
        '''
        reperr = 0.3
        recunc = 35
        projacc = 10

        f = Metashape.TiePoints.Filter()
        f.init(chk, Metashape.TiePoints.Filter.ReprojectionError)
        f.removePoints(reperr)
        f.init(chk, Metashape.TiePoints.Filter.ReconstructionUncertainty)
        f.removePoints(recunc)
        f.init(chk, Metashape.TiePoints.Filter.ProjectionAccuracy)
        f.removePoints(projacc)
        chk.optimizeCameras()

    print('go')
    # output paths
    psx_path = os.path.join(output_folder, 'agisoft.psx')
    pc_path = os.path.join(output_folder, 'point_cloud.ply')
    mesh_path = os.path.join(output_folder, 'mesh.obj')
    ortho_path = os.path.join(output_folder, 'ortho.tif')
    pdf_path = os.path.join(output_folder, 'document.pdf')

    # create list of images
    rgb_folder = os.path.join(img_folder, 'rgb')
    rgb_list = [os.path.abspath(os.path.join(rgb_folder, filename)) for filename in os.listdir(rgb_folder) if
                filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif'))]

    # create agisoft document
    if not start_with_psx:
        doc = Metashape.Document()
        doc.save(path=psx_path)

        # create chunk
        chk = doc.addChunk()

        # add the ir and rgb pictures
        #   loading images
        chk.addPhotos(filenames=rgb_list, load_xmp_accuracy=True)
        print('photos added!')

        # align photos (based on rgb data)
        print('matching photos!')
        chk.matchPhotos(guided_matching=True, downscale=DOWN_SCALE)
        chk.alignCameras()

        # cleaning of initial tie points
        print('cleaning tie points!')
        pre_cleaning(chk)

        # saving file
        doc.save(path=psx_path)

        # depth maps
        if quality == 'low':
            quality_factor = Metashape.LowFaceCount
            downscale_factor = 8
        elif quality == 'medium':
            quality_factor = Metashape.MediumFaceCount
            downscale_factor = 4
        elif quality == 'high':
            quality_factor = Metashape.HighFaceCount
            downscale_factor = 2

        print('building depth maps!')
        chk.buildDepthMaps(downscale=downscale_factor)
        doc.save(path=psx_path)

        # build mesh and reduce size if needed
        print('building model!')
        chk.buildModel(source_data=Metashape.DataSource.DepthMapsData, face_count=quality_factor)
        if opt_limit_poly > 0:
            chk.decimateModel(face_count=opt_limit_poly)
        doc.save(path=psx_path)

        # compute optimal pixel size
        if pixel_size == 0:
            pixel_size = chk.model.meta['BuildModel/resolution']

        # build uv and texture
        print('building uv!')
        chk.buildUV(mapping_mode=Metashape.GenericMapping, pixel_size=pixel_size)

        # build rgb texture
        print(f'building texture')
        chk.buildTexture()

        doc.save(path=psx_path)

        # save model
        print('exporting model')
        chk.exportModel(path=mesh_path, precision=9, save_texture=True, save_uv=True, save_markers=False,
                        crs=Metashape.CoordinateSystem(
                            'LOCAL_CS["Local CS",LOCAL_DATUM["Local Datum",0],UNIT["metre",1]]'))

        # sampling point cloud
        # Sample 1,000,000 points on the mesh
        chk.buildPointCloud(source_data=Metashape.ModelData, points_spacing=0.05)

        # Save the document after sampling
        doc.save(path=psx_path)

        # exporting point cloud
        chk.exportPointCloud(path=pc_path, format=Metashape.PointCloudFormatPLY, crs=Metashape.CoordinateSystem(
            'LOCAL_CS["Local CS",LOCAL_DATUM["Local Datum",0],UNIT["metre",1]]'))

        print("Successfully sampled 1,000,000 points on the mesh.")

        # exporting initial ortho
        chk.buildOrthomosaic(surface_data=Metashape.ModelData, resolution=0.02)
        chk.exportRaster(ortho_path)

        return pixel_size

    else:
        pass


def run_agisoft_thermal_part1(img_folder, output_folder, drone_name, quality='low', opt_limit_poly=0, pixel_size=0,
                              start_with_psx=False):
    def pre_cleaning(chk):
        '''
        A function to do the precleaning after bundle ajustment
        :param chk: input chunk
        :return:
        '''
        reperr = 0.3
        recunc = 35
        projacc = 10

        f = Metashape.TiePoints.Filter()
        f.init(chk, Metashape.TiePoints.Filter.ReprojectionError)
        f.removePoints(reperr)
        f.init(chk, Metashape.TiePoints.Filter.ReconstructionUncertainty)
        f.removePoints(recunc)
        f.init(chk, Metashape.TiePoints.Filter.ProjectionAccuracy)
        f.removePoints(projacc)
        chk.optimizeCameras()

    print('go')
    # paths
    psx_path = os.path.join(output_folder, 'agisoft.psx')
    pc_path = os.path.join(output_folder, 'point_cloud.ply')
    mesh_path = os.path.join(output_folder, 'mesh.obj')

    # create list of images
    ir_folder = os.path.join(img_folder, 'thermal')
    rgb_folder = os.path.join(img_folder, 'rgb')
    rgb_list = [os.path.abspath(os.path.join(rgb_folder, filename)) for filename in os.listdir(rgb_folder) if
                filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif'))]
    ir_list = [os.path.abspath(os.path.join(rgb_folder, filename)) for filename in os.listdir(ir_folder) if
               filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif'))]

    # check drone model
    # drone model specific data
    if drone_name == 'MAVIC2-ENTERPRISE-ADVANCED':
        calib_file = res.find('other/camera_calib_m2t.xml')

    # create agisoft document
    if not start_with_psx:
        doc = Metashape.Document()
        doc.save(path=psx_path)

        # create chunk
        chk = doc.addChunk()

        # add the ir and rgb pictures
        #   loading images

        images = [None] * (len(rgb_list) + len(ir_list))
        images[::2] = ir_list
        images[1::2] = rgb_list
        print(f'Here is images struct : {images}')
        # transform to abs path:

        for i, bad_path in enumerate(images):
            images[i] = os.path.normpath(images[i])

        filegroups = [2] * (len(images) // 2)
        # images is alternating list of rgb, ir paths
        # filegroups defines the multi-camera system groups: [2, 2, 2, ....]

        chk.addPhotos(filenames=images, filegroups=filegroups, layout=Metashape.MultiplaneLayout,
                      load_xmp_accuracy=True)

        print('photos added!')

        # check master
        for sensor in chk.sensors:
            print(sensor.label)
            if sensor == sensor.master:
                continue
            print(sensor.label)

        chk.sensors[0].makeMaster()

        # temporarily desactivate thermal images
        for camera in chk.cameras:
            if camera.sensor != sensor.master:
                print(f'here is a thermal image! : {camera.label}')
                camera.enabled = False

        # align photos (based on rgb data)
        print('matching photos!')
        chk.matchPhotos(guided_matching=True, downscale=DOWN_SCALE)
        chk.alignCameras()

        # cleaning of initial tie points
        print('cleaning tie points!')
        pre_cleaning(chk)

        # saving file
        doc.save(path=psx_path)

        # depth maps
        if quality == 'low':
            quality_factor = Metashape.LowFaceCount
            downscale_factor = 8
        elif quality == 'medium':
            quality_factor = Metashape.MediumFaceCount
            downscale_factor = 4
        elif quality == 'high':
            quality_factor = Metashape.HighFaceCount
            downscale_factor = 2
        chk.buildDepthMaps(downscale=downscale_factor)
        doc.save(path=psx_path)

        # import intrinsic parameters (IR camera)
        user_calib = Metashape.Calibration()
        user_calib.load(calib_file)  # dependent on drone model

        # load calib to thermal sensor
        chk.sensors[1].user_calib = user_calib
        chk.sensors[1].fixed = True

        # build mesh and reduce size if needed
        print('building model!')
        chk.buildModel(source_data=Metashape.DataSource.DepthMapsData, face_count=quality_factor)
        if opt_limit_poly > 0:
            chk.decimateModel(face_count=opt_limit_poly)

        # compute optimal pixel size
        if pixel_size == 0:
            pixel_size = chk.model.meta['BuildModel/resolution']

        # uv
        print('building uv!')
        chk.buildUV(mapping_mode=Metashape.GenericMapping, pixel_size=pixel_size)

        # build rgb texture
        print(f'building texture, pixel size {pixel_size}!')
        chk.buildTexture()

        # save model
        chk.exportModel(path=mesh_path, precision=9, save_texture=True, save_uv=True, save_markers=False,
                        crs=Metashape.CoordinateSystem(
                            'LOCAL_CS["Local CS",LOCAL_DATUM["Local Datum",0],UNIT["metre",1]]'))

        # get texture and change its name
        os.rename(mesh_path[:-4] + '.jpg', mesh_path[:-4] + '_rgb.jpg')

        # change layer index
        chk.sensors[0].layer_index = 1

        # reactivate thermal images
        for camera in chk.cameras:
            if camera.sensor != sensor.master:
                print(f'here is a thermal image! : {camera.label}')
                camera.enabled = True

        # change master
        chk.sensors[1].makeMaster()

        chk.buildTexture()

        # export model
        chk.exportModel(path=mesh_path, precision=9, save_texture=True, save_uv=True, save_markers=False,
                        crs=Metashape.CoordinateSystem(
                            'LOCAL_CS["Local CS",LOCAL_DATUM["Local Datum",0],UNIT["metre",1]]'))

        # rename texture
        os.rename(mesh_path[:-4] + '.jpg', mesh_path[:-4] + '_thermal.jpg')

        # build ambient occlusion
        # chk.buildTexture(texture_type=Metashape.Model.OcclusionMap)

        # build normal map
        # chk.buildTexture(texture_type=Metashape.Model.NormalMap)

        return pixel_size

    else:
        pass


def run_agisoft_rgb_only_part2(psx_path, normal_list, output_folder, pixel_size=0):
    # Load the Metashape project
    doc = Metashape.Document()
    doc.open(psx_path)
    chunk = doc.chunk
    chunk.resetRegion()
    """
    crs = Metashape.CoordinateSystem(
        'LOCAL_CS["Local Coordinates",LOCAL_DATUM["Local Datum",0],UNIT["metre",1,AUTHORITY["EPSG","9001"]]]')
    chunk.crs = crs
    """

    # render top view
    proj = Metashape.OrthoProjection()
    proj.crs = chunk.world_crs
    proj.type = Metashape.OrthoProjection.Type.Planar
    T = chunk.transform.matrix
    lf = chunk.crs.localframe(T.mulp(Metashape.Vector([0, 0, 0])))

    proj.matrix = Metashape.Matrix.Rotation(lf.rotation())
    ortho_path = os.path.join(output_folder, f'ortho_top.png')
    chunk.buildOrthomosaic(surface_data=Metashape.DataSource.ModelData,
                           blending_mode=Metashape.BlendingMode.MosaicBlending,
                           fill_holes=True,
                           resolution=pixel_size,
                           projection=proj)

    chunk.exportRaster(ortho_path)

    for i, normal in enumerate(normal_list):
        ortho_path = os.path.join(output_folder, f'ortho{i}.png')
        print(f'NORMAL:{normal}')
        good_mat = get_r_mat(normal)
        Rot = Metashape.Matrix.Rotation(Metashape.Matrix(good_mat))  # correspond to front XZ view

        T = chunk.transform.matrix
        lf = chunk.crs.localframe(T.mulp(Metashape.Vector([0, 0, 0])))

        proj = Metashape.OrthoProjection()
        proj.crs = chunk.world_crs
        proj.type = Metashape.OrthoProjection.Type.Planar
        proj.matrix = Rot * Metashape.Matrix.Rotation(lf.rotation())

        # Build the orthomosaic using the custom projection
        chunk.buildOrthomosaic(surface_data=Metashape.DataSource.ModelData,
                               blending_mode=Metashape.BlendingMode.MosaicBlending,
                               fill_holes=True,
                               resolution=pixel_size,
                               projection=proj)

        chunk.exportRaster(ortho_path)

    ortho_data = ''
    return ortho_data


def run_agisoft_thermal_part2(psx_path, normal_list, output_folder, do_top=True, pixel_size=0, th_only=False):
    # Load the Metashape project
    doc = Metashape.Document()
    doc.open(psx_path)
    chunk = doc.chunk

    # check if rgb pictures are master
    # change layer index
    chunk.sensors[0].layer_index = 0
    chunk.sensors[0].makeMaster()

    # render top view
    if do_top:
        proj = Metashape.OrthoProjection()
        proj.crs = chunk.world_crs
        proj.type = Metashape.OrthoProjection.Type.Planar
        T = chunk.transform.matrix
        lf = chunk.crs.localframe(T.mulp(Metashape.Vector([0, 0, 0])))

        proj.matrix =  Metashape.Matrix.Rotation(lf.rotation())
        if not th_only:
            ortho_path = os.path.join(output_folder, f'ortho_top.png')
            chunk.buildOrthomosaic(surface_data=Metashape.DataSource.ModelData,
                                   blending_mode=Metashape.BlendingMode.MosaicBlending,
                                   fill_holes=True,
                                   resolution=pixel_size,
                                   projection=proj)

            chunk.exportRaster(ortho_path)

        # change layer index
        chunk.sensors[0].layer_index = 1
        chunk.sensors[1].makeMaster()

        # render top view thermal
        ortho_path = os.path.join(output_folder, f'ortho_top_th.png')
        chunk.buildOrthomosaic(surface_data=Metashape.DataSource.ModelData,
                               blending_mode=Metashape.BlendingMode.AverageBlending,
                               fill_holes=True,
                               ghosting_filter=True,
                               resolution=pixel_size*10,
                               projection=proj)

        chunk.exportRaster(ortho_path)

    for i, normal in enumerate(normal_list):
        ortho_path = os.path.join(output_folder, f'ortho{i}.png')
        ortho_th_path = os.path.join(output_folder, f'ortho_th{i}.png')
        print(f'NORMAL:{normal}')
        good_mat = get_r_mat(normal)
        Rot = Metashape.Matrix.Rotation(Metashape.Matrix(good_mat))  # correspond to front XZ view

        T = chunk.transform.matrix
        lf = chunk.crs.localframe(T.mulp(Metashape.Vector([0, 0, 0])))

        proj = Metashape.OrthoProjection()
        proj.crs = chunk.world_crs
        proj.type = Metashape.OrthoProjection.Type.Planar
        proj.matrix = Rot * Metashape.Matrix.Rotation(lf.rotation())

        # check if rgb pictures are master
        # change layer index
        if not th_only:
            chunk.sensors[0].layer_index = 0
            chunk.sensors[0].makeMaster()

            # Build the orthomosaic using the custom projection
            chunk.buildOrthomosaic(surface_data=Metashape.DataSource.ModelData,
                                   blending_mode=Metashape.BlendingMode.MosaicBlending,
                                   fill_holes=True,
                                   resolution=pixel_size,
                                   projection=proj)

            chunk.exportRaster(ortho_path)

        # change layer index
        chunk.sensors[0].layer_index = 1
        chunk.sensors[1].makeMaster()

        chunk.buildOrthomosaic(surface_data=Metashape.DataSource.ModelData,
                               blending_mode=Metashape.BlendingMode.AverageBlending,
                               fill_holes=True,
                               ghosting_filter=True,
                               resolution=pixel_size*10,
                               projection=proj)

        chunk.exportRaster(ortho_th_path)

        ortho_data = ''

    doc.save(psx_path)
    return ortho_data


def get_r_mat(orientation_vector):
    print(f'Analysing vector {orientation_vector}')
    # Extract the X and Y components
    x, y = orientation_vector[0], orientation_vector[1]

    # Compute the angle theta
    theta = -np.arctan2(y, x)
    print(f'THETA {theta}')

    # Define the first rotation matrix (top view to horizontal view)
    R_horizontal = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])

    # Define the second rotation matrix (rotation around the Z-axis)
    R_z = np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

    # Compute the final rotation matrix
    R_final = np.dot(R_z, R_horizontal)
    print(f'_____________________________{R_final}')

    return R_final


def run_agisoft_recolor(yolo_img_dir):
    pass

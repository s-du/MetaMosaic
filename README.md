<p align="center">
    <a href=""><img src="resources\img\MetaMosaic.png" alt="Thermogram" border="0" style="width: 100%;"></a>
</p>


## Overview
We propose a script to automate the orthomosaïc generation from drone images (Infrared IR and colour RGB). This workflow leverages the capabilities of multiple libraries, such as Agidoft Metashape, CloudCompare and YOLO, and is compatible with DJI Drones. At the moment, the app includes support for Mavic 2 Enterprise and Mavic 3 Thermal series.
As an input, the user must provide an image folder with organized as:

```
image/
│
├── Th/    # Thermal images 
│   ├── thermal_image1.jpg
│   ├── thermal_image2.jpg
│   └── thermal_image3.jpg
│
└── Rgb/   # Color images
    ├── color_image1.jpg
    ├── color_image2.jpg
    └── color_image3.jpg
```
Thermal image can be processed with any software, but should not be corrected for deformation! See https://github.com/s-du/Thermogram for an easy open-source tool for DJI image processing.

**The project is still in pre-release, so do not hesitate to send your recommendations or the bugs you encountered!**

## Features
The developed data workflow is designed to extract the maximum amount of information from raw drone imagery in the context of energy retrofits, with a high degree of automation. The captured RGB and thermal images are processed through a series of steps, organized into four main stages. The complete workflow is illustrated below, showing the step-by-step transformation from initial data acquisition to the generation of a synthetic, ‘enriched’ model. Each stage is described in detail in the following sections. 

<p align="center">
    <a href=""><img src="resources\img\diagram.png" alt="diagram" border="0";"></a>
    
    Data processing workflow, divided into 4 main stages
</p>

The photogrammetric reconstruction is performed with Agisoft Metashape API (v.2.1.2). The first step: import all images as a multi-camera system. This automatically recognises each RGB/IR image group as being co-registered in space, which greatly facilitates the alignment procedure. In the software, a “master” sensor must be defined that will serve as a support for all “secondary” sensors in the multi-camera system. Here, we first define the RGB images as master to enter the alignment step in Agisoft Metashape. Once the RGB cameras are aligned and the extrinsic camera parameters are estimated, a high-resolution mesh is created from depth maps computation. Thereafter, the IR images are defined as the master sensor to enter the second stage of the 3D workflow in Agisoft Metashape. This will allow projecting the thermal information as a texture onto the detailed mesh. The intrinsic parameters of the IR camera are also imported at this stage. All reconstruction steps are performed automatically using the Python API of the photogrammetry software. 

<p align="center">
    <a href=""><img src="resources\img\mesh.png" alt="diagram" border="0";"></a>

    Example of a mesh produced with Agisoft API
</p>

Once the high-resolution mesh is created, the next step is to determine the orientation of all façades. a so-called 'smart' orthomosaic generation. This is particularly beneficial for complex buildings, where façades may not be orthogonal to one another.
All geometric analyses are performed automatically using the CloudCompare command-line interface. First, a point cloud is sampled from the 3D mesh, with a uniform point spacing of 0.05 meters. This low-resolution point cloud serves as the basis for planar detection using the RANSAC algorithm, which generates a list of all detected planar surfaces. Each detected plane is associated with corresponding support points. By analyzing the normal orientation of these points, vertical planes—representing the building façades—can be filtered easily. The point normal analysis also determines the correct projection direction for generating façade orthomosaics, avoiding back-projection. After filtering out duplicate vectors for building façades facing the same direction, the process outputs a refined list of vectors that can be used for generating optimal façade projections. The next step involves importing this vector list into Agisoft Metashape and generating all orthomosaics in an automated loop. For each direction, both RGB and thermal ortho-image are generated. A log file (.txt) is created to record the normal vector for each image file along with the image resolution (in pixels per meter).

<p align="center">
    <a href=""><img src="resources\img\ransac.png" alt="diagram" border="0";"></a>

    Results of the plane detection phase, for determining the optimal projection orientation of orthomosaics: (a) output of the RANSAC algorithm with all detected planes; (b) plane filtering and final projection orientations; (c) orthomosaics generated automatically in Agisoft Metashape
</p>

A key result of this script is the generation of color and thermal orthomosaic pairs, as illustrated below. For each façade within the zone of interest, diagnostic experts gain access to a rich dataset, enabling the visual detection of structural pathologies and the identification of potential abnormal heat losses.

<p align="center">
    <a href=""><img src="resources\img\results.png" alt="diagram" border="0";"></a>
</p>

## Files and Structure
- `resources/`: Contains essential resources for the application.
- `engine/`: Contains essential data processing logic for the application.
- `ui/`: The user interface files for the application (Coming soon)
- `main.py`: The main Python script for running the application.

## Topics
- Drones
- Infrared Thermography
- Inspection
- Segmentation
- Building pathologies

## Installation
1. Clone the repository:
```
git clone https://github.com/s-du/MetaMosaic
```

2. Navigate to the app directory:
```
cd MetaMosaic
```
3. (Optional) Install and activate a virtual environment

   
4. Install the required dependencies:
```
pip install -r requirements.txt
```

5. Modify the parameters
   
6. Run the app:
```
python main.py
```
## Usage
(Coming soon)

## Contributing
Contributions are welcome! If you find any bugs, have suggestions for new features, or would like to contribute enhancements, please follow these steps:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Make the necessary changes and commit them.
4. Push your changes to your fork.
5. Submit a pull request describing your changes.
6. 

## Citing

```
@software{MetaMosaic,
  author = {Dubois, Samuel and Vandenabeele, Louis},
  title = {MetaMosaic: smart processing of drone data for building diagnostics},
  year = {2024},
  url = {https://github.com/s-du/MetaMosaic}
}
```

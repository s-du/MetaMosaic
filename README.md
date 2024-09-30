<p align="center">
    <a href=""><img src="resources\img\MetaMosaic.png" alt="Thermogram" border="0" style="width: 100%;"></a>
</p>


## Overview
We propose a script to automate the orthomosaïc generation from drone images (Infrared IR and colour RGB). This workflow leverages the capabilities of multiple libraries, such as Agidoft Metashape, CloudCompare and YOLO, and is compatible with DJI Drones. At the moment, the app includes support for Mavic 2 Enterprise and Mavic 3 Thermal series.

**The project is still in pre-release, so do not hesitate to send your recommendations or the bugs you encountered!**

## Features
The developed data workflow is designed to extract the maximum amount of information from raw drone imagery in the context of energy retrofits, with a high degree of automation. The captured RGB and thermal images are processed through a series of steps, organized into four main stages. The complete workflow is illustrated below, showing the step-by-step transformation from initial data acquisition to the generation of a synthetic, ‘enriched’ model. Each stage is described in detail in the following sections. 

<p align="center">
    <a href=""><img src="resources\img\diagram.png" alt="diagram" border="0";"></a>
    
    Data processing workflow, divided into 4 main stages
</p>

A key result of this script is the generation of color and thermal orthomosaic pairs, as illustrated below. For each façade within the zone of interest, diagnostic experts gain access to a rich dataset, enabling the visual detection of structural pathologies and the identification of potential abnormal heat losses.

<p align="center">
    <a href=""><img src="resources\img\results.png" alt="diagram" border="0";"></a>
</p>

## Files and Structure
- `resources/`: Contains essential resources for the application.
- `tools/`: Contains essential image processing logic for the application.
- `ui/`: The user interface files for the application.
- `main.py`: The main Python script for running the application.
- `dialogs.py`: Handles the dialog logic.
- `widgets.py`: Defines Pyside6 widgets and UI components.

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

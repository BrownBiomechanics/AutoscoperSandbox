# Multi-modality Support Notebooks

This folder contains notebooks that demonstrate the upcoming multi-modality support for SlicerAutoscoper^M.

## Notebooks

### Virtual Radiograph (VRG) Generation Pipeline

NOTE: This notebook requires you to download the `2023-05-25-wrist-vol.vtk` file from the [SandboxRecourses Release](https://github.com/BrownBiomechanics/AutoscoperSandbox/releases/download/sandbox-resources/2023-05-25-wrist-vol.vtk). The volume vtk file should be placed with the `Data` subfolder of this folder.

This pipeline code is organized in the `VRGGenPipeline` notebook. This notebook demonstrates the VRG pipeline using VTK. The pipeline is broken down into the following steps:

* Get the volume data
* Set up a camera using `vtkCamera`
* Set up a renderer using `vtkRenderer` and a render window using `vtkRenderWindow`
* Create an ocpacity transfer function using `vtkPiecewiseFunction`
* Set up a volume property using `vtkVolumeProperty`
* Set up a volume mapper using `vtkGPUVolumeRayCastMapper`
* Render the volume using `vtkVolume`
* Extract the rendered volume and save it as a tif file

### Data Intensity Density (DID) Calculation Pipeline

This pipeline code is organized in the `DataIntensityDensityTest` notebook. This notebook demonstrates the DID pipeline using SimpleITK. The pipeline is broken down into the following steps:

* Get the generated VRG tif file
* Perform superpixel segmentation using `sitk.SLICImageFilter`
* Calculate the mean value for each superpixel
* Create a binary mask for all superpixels with a mean value less than a threshold
* Use `sitk.ConnectedComponent` and `sitk.RelabelComponent` to determine the total number of pixels in the mask and the size of the largest connected component.
* The size of the largest connected component is the DID value

### MATLAB vs. VTK VRG Pipeline Comparison

This pipeline code is organized in the `VRGGenPipelineComp` notebook. This notebook takes VRGs generated using MATLAB and VTK and compares them. The pipeline is broken down into the following steps:

* Read the VRG generated using MATLAB and VTK
* Display the VRGs side by side
* Remove the background from the VRGs
* Display the histograms of the VRGs side by side

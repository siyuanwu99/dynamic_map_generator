/****
 * Author: Clarence & ChatGPT
 * Date: 2023-09-2
 * This file gives an example of how to generate a dense point cloud from a text string.
 * To use this file, you need to install the following dependencies:
 *     1. PCL
 *     2. VTK
 *
 * In CmakeLists.txt, you need to add the following lines:
 *    find_package(PCL REQUIRED)
 *    find_package(VTK REQUIRED)
 *    include_directories(${PCL_INCLUDE_DIRS})
 *    target_link_libraries(xxx ${PCL_LIBRARIES} ${VTK_LIBRARIES})
 * */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <vtkDelaunay2D.h>
#include <vtkLinearExtrusionFilter.h>
#include <vtkLinearSubdivisionFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkVectorText.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr convertVTKtoPCL(vtkSmartPointer<vtkPolyData> polydata) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (vtkIdType i = 0; i < polydata->GetNumberOfPoints(); i++) {
    double p[3];
    polydata->GetPoint(i, p);
    cloud->points.push_back(pcl::PointXYZ(p[2], -p[0], p[1]));
  }
  cloud->width  = cloud->points.size();
  cloud->height = 1;
  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr generateDensePointCloudFromText(const std::string &text,
                                                                    int                numLayers,
                                                                    double layerThickness) {
  // Create 3D text
  vtkSmartPointer<vtkVectorText> vectorText = vtkSmartPointer<vtkVectorText>::New();
  vectorText->SetText(text.c_str());

  // Scale for visibility
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->Scale(4.0, 4.0, 4.0);

  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
      vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilter->SetInputConnection(vectorText->GetOutputPort());
  transformFilter->SetTransform(transform);
  transformFilter->Update();

  // 1. No need for triangulation since vtkVectorText already gives us a mesh.
  //    So, we directly proceed with subdivision for density.

  // 2. Subdivide for Density
  vtkSmartPointer<vtkLinearSubdivisionFilter> subdivFilter =
      vtkSmartPointer<vtkLinearSubdivisionFilter>::New();
  subdivFilter->SetInputConnection(transformFilter->GetOutputPort());
  subdivFilter->SetNumberOfSubdivisions(3);
  subdivFilter->Update();

  pcl::PointCloud<pcl::PointXYZ>::Ptr combinedCloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (int layer = 0; layer < numLayers; ++layer) {
    // 3. Extrude for Thickness
    vtkSmartPointer<vtkLinearExtrusionFilter> extrudeFilter =
        vtkSmartPointer<vtkLinearExtrusionFilter>::New();
    extrudeFilter->SetInputConnection(subdivFilter->GetOutputPort());
    extrudeFilter->SetExtrusionTypeToNormalExtrusion();
    extrudeFilter->SetScaleFactor(layerThickness);
    extrudeFilter->Update();

    pcl::PointCloud<pcl::PointXYZ>::Ptr layerCloud = convertVTKtoPCL(extrudeFilter->GetOutput());

    // Adjust the z-values of the layer based on its position
    for (auto &point : layerCloud->points) {
      point.z += layer * layerThickness;
    }

    // Combine the current layer with the main cloud
    *combinedCloud += *layerCloud;
  }

  return combinedCloud;
}

int main(int argc, char **argv) {
  int    numLayers      = 5;     // e.g., 5 layers
  double layerThickness = 0.10;  // e.g., 1 units of thickness per layer

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
      generateDensePointCloudFromText("TUDelft", numLayers, layerThickness);

  // Visualize
  pcl::visualization::CloudViewer viewer("3D Text Viewer");
  viewer.showCloud(cloud);
  while (!viewer.wasStopped()) {
  }

  /* save to pcd file */
  pcl::io::savePCDFileASCII("text.pcd", *cloud);

  return 0;
}

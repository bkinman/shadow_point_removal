#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>

void statistical_outlier_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(input_cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh (1.0);
  sor.filter(*output_cloud);
}

void shadowpoints_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
  /* ShadowPoint stuff begins here */
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
  ne.setInputCloud(input_cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::PointNormal>::Ptr input_normals (new pcl::PointCloud<pcl::PointNormal>);
  ne.setKSearch(15);

  ne.compute(*input_normals);

  pcl::ShadowPoints<pcl::PointXYZ, pcl::PointNormal> sp;
  sp.setInputCloud(input_cloud);
  sp.setThreshold (0.5f);
  sp.setNormals(input_normals);
  sp.filter(*output_cloud);
}

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sp_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  statistical_outlier_filter(cloud, cloud_sor_filtered);

  std::cerr << "Cloud after Statistical Outlier Filtering filtering: " << std::endl;
  std::cerr << *cloud_sor_filtered << std::endl;

  shadowpoints_filter(cloud, cloud_sp_filtered);

  std::cerr << "Cloud after ShadowPoints filtering: " << std::endl;
  std::cerr << *cloud_sp_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ>("table_scene_lms400_sp_inliers.pcd", *cloud_sp_filtered, false);
  writer.write<pcl::PointXYZ>("table_scene_lms400_sor_inliers.pcd", *cloud_sor_filtered, false);

  return (0);
}
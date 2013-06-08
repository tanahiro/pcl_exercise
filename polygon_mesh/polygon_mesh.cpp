/**
 *  greedy_projection_triangulation.cpp
 *
 *  LICENSE: CC0 
 *  (http://creativecommons.org/publicdomain/zero/1.0/)
 *
 */

#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>

#define NUM_THREAD_NORMAL_ESTIMATE 5

using namespace  std;

void generatePointCloudCylinder(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  for (double z = -5.0; z <= 5.0; z += 0.1) {
    for (double th = 0.0; th < 2*M_PI; th += M_PI/360.0) {
      pcl::PointXYZRGB point;

      point.x = 5.0*cos(th);
      point.y = 5.0*sin(th);
      point.z = z;

      point.r = (unsigned int)(127 + z*127.0/5.0);
      point.g = (unsigned int)(127 - z*127.0/5.0);
      point.b = (unsigned int)(fabs(z)*127.0/5.0);

      cloud->push_back(point);
    }
  }
}

void estimateNormals(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal>
    normal_est(NUM_THREAD_NORMAL_ESTIMATE);
  normal_est.setInputCloud(cloud);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree
    (new pcl::search::KdTree<pcl::PointXYZRGB>);
  normal_est.setSearchMethod(tree);
  normal_est.setRadiusSearch(3.0);
  normal_est.compute(*normals);
}

/**
 * reference:
 * http://pointclouds.org/documentation/tutorials/greedy_projection.php
 */
void generatePolygonMeshGP3(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    pcl::PolygonMesh::Ptr mesh)
{
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals
    (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree
    (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

  pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
  gp3.setSearchRadius(5.0);
  /* Typical value: 2.5 - 3.0 (1.5 for grids) */
  gp3.setMu(2.5);
  /* Typical value: 50 - 100 */
  gp3.setMaximumNearestNeighbors(100);
  /* Typical value: 45 degrees */
  gp3.setMaximumSurfaceAngle(M_PI/4);
  /* Typical value: 10 degrees */
  gp3.setMinimumAngle(M_PI/18);
  /* Typical value: 120 degrees */
  gp3.setMaximumAngle(M_PI*2/3);
  /* Typical value: false */
  gp3.setNormalConsistency(false);
  gp3.setConsistentVertexOrdering(false);

  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(tree);
  gp3.reconstruct(*mesh);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> make_viewer() {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
    (new pcl::visualization::PCLVisualizer("polygon mesh"));

  viewer->setBackgroundColor(0.4, 0.6, 0.6);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  viewer->setSize(600, 600);

  return viewer;
}

int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud
    (new pcl::PointCloud<pcl::PointXYZRGB>);
  generatePointCloudCylinder(cloud);

  pcl::PointCloud<pcl::Normal>::Ptr normals
    (new pcl::PointCloud<pcl::Normal>);
  estimateNormals(cloud, normals);

  string polygon_id;

  pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
  generatePolygonMeshGP3(cloud, normals, mesh);
  polygon_id = "GP3";

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = make_viewer();
  viewer->addPolygonMesh(*mesh, polygon_id);

  viewer->spin();

  return 0;
}

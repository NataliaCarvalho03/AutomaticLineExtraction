#include <iostream>
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("non_ground.pcd", *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (10000);
  reg.setMaxClusterSize (100000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  //std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  //std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
  //std::cout << "These are the indices of the points of the initial" <<
    //std::endl << "cloud that belong to the first cluster:" << std::endl;
  //int counter = 0;
  pcl::PCDWriter writer;

  for (int i=0; i < clusters.size(); i++){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

    //Alocando espaço para a nuvem
    cloud2->width = clusters[i].indices.size();
    cloud2->height = 1;
    cloud2->points.resize(cloud2->width * cloud2->height);   

    //montando a nuvem de pontos

    for (int p=0; p < clusters[i].indices.size(); p++){
      cloud2->points[p].x = cloud->points[clusters[i].indices[p]].x;
      cloud2->points[p].y = cloud->points[clusters[i].indices[p]].y;
      cloud2->points[p].z = cloud->points[clusters[i].indices[p]].z;
    }

    writer.write<pcl::PointXYZ> ("group_" + std::to_string(i+1) + ".pcd", *cloud2, false);
    std::cout << "Escrevi o arquivo " << i+1 <<std::endl;
    std::cout << "Tamanho da nuvem " << i+1 << ": "<< cloud2->size() <<std::endl;
  }





  //while (counter < clusters[0].indices.size ())
  //{
    //std::cout << clusters[0].indices[counter] << ", ";
    //counter++;
    //if (counter % 10 == 0)
      //std::cout << std::endl;
  //}
  std::cout << std::endl;

  //pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  //pcl::visualization::CloudViewer viewer ("Cluster viewer");
  //viewer.showCloud(colored_cloud);
  //while (!viewer.wasStopped ())
  //{
  //}

  return (0);
}
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

void write_cloud_txt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    //std::cout<<std::fixed;
    std::ofstream nuvem_txt;
    nuvem_txt.open("planes.txt");

    //Escrevendo os pontos
    for (size_t i = 0; i < cloud->points.size (); ++i){
        if (i < cloud->points.size() - 1){
            nuvem_txt <<std::fixed<< cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;
        }else{
            nuvem_txt <<std::fixed<< cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z;
        }
    }    

    nuvem_txt.close();
  

}

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_planes(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  //seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (15.0);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  //std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
  //                                    << coefficients->values[1] << " "
  //                                    << coefficients->values[2] << " " 
  //                                    << coefficients->values[3] << std::endl;

  //std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

  //Alocando espaço para a nova nuvem com os inliers
  cloud_planes->width = inliers->indices.size();
  cloud_planes->height = 1;
  cloud_planes->points.resize(cloud_planes->width * cloud_planes->height);
  
  for (std::size_t i = 0; i < inliers->indices.size (); ++i){

    cloud_planes->points[i].x = cloud->points[inliers->indices[i]].x;
    cloud_planes->points[i].y = cloud->points[inliers->indices[i]].y;
    cloud_planes->points[i].z = cloud->points[inliers->indices[i]].z;
  }

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("planes.pcd", *cloud_planes, false);
    //td::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                               //<< cloud->points[inliers->indices[i]].y << " "
                                               //<< cloud->points[inliers->indices[i]].z << std::endl;

  write_cloud_txt(cloud_planes);

  return (0);
}
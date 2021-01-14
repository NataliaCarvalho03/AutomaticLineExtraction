#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

void write_cloud_txt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    //std::cout<<std::fixed;
    std::ofstream nuvem_txt;
    nuvem_txt.open("non_ground.txt");

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
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndicesPtr ground (new pcl::PointIndices);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> ("MDE_1000_clip.pcd", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;
  
  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud (cloud);
  pmf.setMaxWindowSize (20);
  pmf.setSlope (0.5f);
  pmf.setInitialDistance (0.5f);
  pmf.setMaxDistance (1.0f);
  //std::cout<<ground->indices.size()<<std::endl;
  pmf.extract (ground->indices);


  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (ground);
  extract.filter (*cloud_filtered);

  std::cerr << "Ground cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("ground.pcd", *cloud_filtered, false);

  // Extract non-ground returns
  extract.setNegative (true);
  extract.filter (*cloud_filtered);

  std::cerr << "Object cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  writer.write<pcl::PointXYZ> ("non_ground.pcd", *cloud_filtered, false);
  write_cloud_txt(cloud_filtered);

  return (0);
}
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>



int main (int argc, char** argv){


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("../group_3.pcd", *cloud3) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZ>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("../group_4.pcd", *cloud4) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud5 (new pcl::PointCloud<pcl::PointXYZ>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("../group_5.pcd", *cloud5) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ> aux = *cloud3 + *cloud4;
    aux += *cloud5;
    
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("cloud_concat.pcd", aux, false);
}
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <fstream>
#include <eigen3/Eigen/Dense>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>


void write_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){    

  pcl::io::savePCDFileASCII ("MDE_1000_clip.pcd", *cloud);
  std::cerr << "Saved " << cloud->size () << " data points to test_pcd.pcd." << std::endl;
}

void write_cloud_txt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    //std::cout<<std::fixed;
    std::ofstream nuvem_txt;
    nuvem_txt.open("MDE_1000_clip.txt");

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

std::vector<std::vector<float>> read_utm_points(){

    FILE* f = fopen("limit_points.txt", "r");

    std::vector<std::vector<float>> utm_points;

    if (NULL == f)
    {
        std::cout << "ERROR: failed to open file: limit_points.txt" << endl;
    }else{
        std::cout<<"SUCESS!"<<std::endl;
    }

    float E, N;

    while (!feof(f))
    {
        int n_args = fscanf(f, "%f %f", &E, &N);

        if (n_args != 2)
            continue;

        std::vector<float> p_aux;
        p_aux.push_back(E);
        p_aux.push_back(N);

        utm_points.push_back(p_aux);

        }

        return utm_points;



}
pcl::PointCloud<pcl::PointXYZ>::Ptr loadAsciCloud(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::cout << "Begin Loading Model" << std::endl;
    FILE* f = fopen(filename.c_str(), "r");

    if (NULL == f)
    {
        std::cout << "ERROR: failed to open file: " << filename << endl;
    }else{
        std::cout<<"SUCESS!"<<std::endl;
    }

    std::vector<std::vector<float>> utm_limits = read_utm_points();

    for (int i=0; i < utm_limits.size(); i++){
           std::cout<<utm_limits.at(i).at(0)<<" "<<utm_limits.at(i).at(1)<<std::endl;
        }

    float x, y, z, X1 = utm_limits.at(0).at(0), Y1 = utm_limits.at(0).at(1), X2 = utm_limits.at(1).at(0), Y2 = utm_limits.at(1).at(1);
    int r;

    while (!feof(f))
    {
        int n_args = fscanf(f, "%f %f %f %d", &x, &y, &z, &r);

        if (n_args != 4)
            continue;

        if(x <= X1 && x>=X2 && y >= Y2 && y <= Y1){

            pcl::PointXYZ point;
            point.x = x; 
            point.y = y; 
            point.z = z;

            cloud->push_back(point);
        }

        
    }

    fclose(f);

    std::cout << "Loaded cloud with " << cloud->size() << " points." << std::endl;

    return cloud;
}

void view(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  
  /*std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl; */
  
   
   //... populate cloud
   pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   viewer.showCloud (cloud);
   while (!viewer.wasStopped ())
   {
     
   }
}

int main(int argc, char** argv){
    
    if(argc < 2 || argc > 2){
        cout<<"Você deve informar o nome de um único arquivo! Ex: ./convert_photoscan_to_pcd nuvem.txt"<<endl;
    }else{
        pcl::PointCloud<pcl::PointXYZ>::Ptr my_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        my_cloud = loadAsciCloud(argv[1], my_cloud);
        //view(my_cloud);
        write_cloud(my_cloud);
        write_cloud_txt(my_cloud);

    }
}
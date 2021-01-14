#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
//#include"stdafx.h"
#include<fstream>

void write_cloud_txt(std::vector<std::vector<int>> px_points){

    //std::cout<<std::fixed;
    std::ofstream nuvem_px_txt;
    nuvem_px_txt.open("pc_pixels_concat.txt");

    //Escrevendo os pontos
    for (size_t i = 0; i < px_points.size(); ++i){
        if (i <  (px_points.size () - 1)){
           nuvem_px_txt << px_points.at(i).at(0)
           <<" "<< px_points.at(i).at(1) << std::endl;
        }else{
            nuvem_px_txt << px_points.at(i).at(0)
           <<" "<< px_points.at(i).at(1);
        }
    }    

    nuvem_px_txt.close();
  

}

std::vector<float> read_orientationParameters(std::string fileName){

    char cstr[fileName.size() + 1];
    strcpy(cstr, fileName.c_str());

    FILE *file = fopen(cstr, "r");

    if (NULL == file)
    {
        std::cout << "\nErro: Falhou em ler o arquivo: " << fileName << std::endl;
    }else{
        std::cout<<"\nArquivo de Parâmetros lido com sucesso!"<<std::endl;
    }

    float parameter;
    std::vector<float> orientation_parameters;

    while(!feof(file)){
        
        fscanf(file,"%f", &parameter);
        orientation_parameters.push_back(parameter);
    }

    return orientation_parameters;
}

std::vector<std::vector<float>> read_pointsUTM(std::string fileName){

    char cstr[fileName.size() + 1];
    strcpy(cstr, fileName.c_str());

    FILE *file_points = fopen(cstr, "r");

    if (NULL == file_points)
    {
        std::cout << "\nErro: Falhou em ler o arquivo: " << fileName << std::endl;
    }else{
        std::cout<<"\nArquivo de pontos lido com sucesso!"<<std::endl;
    }

    float x,y,z;
    std::vector<std::vector<float>> points;

    while(!feof(file_points)){
        fscanf(file_points,"%f %f %f", &x, &y, &z);
        std::vector<float> inner_coords{x,y,z};

        points.push_back(inner_coords);
    }

    return points;
}

std::vector<std::vector<float>> calculate_photo_coords(std::vector<float> orientation_parameters, std::vector<std::vector<float>> point_cloud){

    float X0 = orientation_parameters.at(0);
    float Y0 = orientation_parameters.at(1);
    float Z0 = orientation_parameters.at(2);

    float om = orientation_parameters.at(3);
    float fi = orientation_parameters.at(4);
    float kapa = orientation_parameters.at(5);

    float c = orientation_parameters.at(6);
    
    float m11 = cos(fi) * cos(kapa);
    float m12 = cos(om) *sin(kapa) + sin(om)*sin(fi)*cos(kapa);
    float m13 = sin(om) *sin(kapa) - cos(om)*sin(fi)*cos(kapa);
    
    float m21 = -cos(fi)* sin(kapa);
    float m22 = cos(om)*cos(kapa) - sin(om)*sin(fi)*sin(kapa);
    float m23 = sin(om)*cos(kapa) + cos(om)*sin(fi)*sin(kapa);

    float m31 = sin(fi);
    float m32 = -sin(om)*cos(fi);
    float m33 = cos(om)*cos(fi);

    
    std::vector<std::vector<float>> photo_points;

    for(int i=0; i<point_cloud.size(); i++){
        float X = point_cloud.at(i).at(0);
        float Y = point_cloud.at(i).at(1);
        float Z = point_cloud.at(i).at(2);

        float x = - c * ((m11*(X-X0) + m12*(Y-Y0) + m13*(Z-Z0)) / (m31*(X-X0) + m32*(Y-Y0) + m33*(Z-Z0)));
        float y = - c * ((m21*(X-X0) + m22*(Y-Y0) + m23*(Z-Z0)) / (m31*(X-X0) + m32*(Y-Y0) + m33*(Z-Z0)));

        std::vector<float> photo_coords{x,y};
        photo_points.push_back(photo_coords);
    }

    return photo_points;    
}

std::vector<int> convert_mm_to_px(float x, float y, int columms, int rows){

    float pixel_size_x = 0.008;
    float pixel_size_y = 0.008;
    
    int C = int(x/pixel_size_x + columms/2);
    int L = int(rows/2 - y/pixel_size_y);

    std::vector<int> coords_px{C, L};

    return coords_px;
}

cv::Mat separate_inner_image(cv::Mat source, std::vector<float> op, std::vector<std::vector<int>> points_px){

    //Procura pelo ponto com menor e maior valor L
    int L_min = points_px.at(0).at(1);
    int L_max = points_px.at(0).at(1);

    int C_min = points_px.at(0).at(0);
    int C_max = points_px.at(0).at(0);

    for (int i=0; i<points_px.size(); i++){
        if(points_px.at(i).at(1) < L_min){
            L_min = points_px.at(i).at(1);
        }

        if(points_px.at(i).at(1) > L_max){
            L_max = points_px.at(i).at(1);
        }

        if(points_px.at(i).at(0) < C_min){
            C_min = points_px.at(i).at(0);
        }

        if(points_px.at(i).at(0) > C_max){
            C_max = points_px.at(i).at(0);
        }
    }

    std::cout<<"Lmin "<<L_min<<", "<<"Cmin "<<C_min<<"\t"<<"Lmax "<<L_max << ", "<<"Cmax "<<C_max<<std::endl;

    int dst_rows = L_max - L_min;
    int dst_columns = C_max - C_min;

    std::cout<<dst_rows<<", "<<dst_columns<<std::endl;

    cv::Mat dst = cv::Mat::zeros(cv::Size(dst_rows, dst_columns), CV_8U);

    for(int l = L_min; l<=L_max; l++){
        for(int c = C_min; c<=C_max; c++){
            dst.at<uchar>((l - L_min), (c - C_min)) = source.at<uchar>(l,c);
        }
    }

    cv::imshow("inner", dst);
    cv::waitKey(0);

    cv::imwrite("inner_image.tif", dst);

    return dst;

}

void separate_inner_image_rgb(cv::Mat source_rgb, std::vector<float> op, std::vector<std::vector<int>> points_px){

    cv::Mat channels[3], inner_channels[3], inner_img_rgb;
    cv::split(source_rgb, channels);

    cv::Mat inner_blue = separate_inner_image(channels[0], op, points_px);
    cv::Mat inner_green = separate_inner_image(channels[1], op, points_px);
    cv::Mat inner_red = separate_inner_image(channels[2], op, points_px);

    
    //std::cout<<"Separei as bandas!"<<std::endl;

    inner_channels[0].push_back(inner_blue);
    inner_channels[1].push_back(inner_green);
    inner_channels[2].push_back(inner_red);

    //std::cout<<"Coloquei as bandas na Mat!"<<std::endl;

    cv::merge(inner_channels, 3, inner_img_rgb);

    cv::imwrite("inner_img_rgb.tif", inner_img_rgb);

}

int main(int argc, char** argv){

    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = img;
    cv::Mat img3 = cv::Mat::zeros(cv::Size(3000,4500), 0);

    std::cout<<img2.type()<<std::endl;

    if(img.empty()){
        std::cout<<"ERRO AO TENTAR LER "<< argv <<std::endl;
    }else{
        std::cout<<"IMAGEM LIDA COM SUCESSO!"<<std::endl;
    }

    std::vector<float> img_orientation = read_orientationParameters("../orienta/op.txt");

    std::vector<std::vector<float>> point_cloud = read_pointsUTM("cloud_concat.txt");

    std::vector<std::vector<float>> photo_points = calculate_photo_coords(img_orientation, point_cloud);

    std::vector<std::vector<int>> photo_points_px;

    for (int i=0; i<photo_points.size(); i++){
        std::vector<int> temp = convert_mm_to_px(photo_points.at(i).at(0), photo_points.at(i).at(1), img.cols, img.rows);

        photo_points_px.push_back(temp);
    }

    write_cloud_txt(photo_points_px);

    for(int i=0; i<photo_points_px.size(); i++){
        img2.at<uchar>(photo_points_px.at(i).at(1), photo_points_px.at(i).at(0)) = 0;
    }

    cv::imwrite("points.tif", img2);

    cv::Mat s = cv::imread("../orienta/Photo_054.tif", cv::IMREAD_COLOR); //POG

    cv::Mat inner = separate_inner_image(img, img_orientation, photo_points_px);
    separate_inner_image_rgb(s, img_orientation, photo_points_px);
}
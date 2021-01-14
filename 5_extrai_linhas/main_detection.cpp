/*----------------------------------------------------------------------------  
  This code is part of the following publication and was subject
  to peer review:
  "Multiscale line segment detector for robust and accurate SfM" by
  Yohann Salaun, Renaud Marlet, and Pascal Monasse
  ICPR 2016
  
  Copyright (c) 2016 Yohann Salaun <yohann.salaun@imagine.enpc.fr>
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the Mozilla Public License as
  published by the Mozilla Foundation, either version 2.0 of the
  License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  Mozilla Public License for more details.
  
  You should have received a copy of the Mozilla Public License
  along with this program. If not, see <https://www.mozilla.org/en-US/MPL/2.0/>.

  ----------------------------------------------------------------------------*/
#include <cmath>
#include "interface.hpp"
#include "DETECTION/detection.hpp"
#include "cmdLine/cmdLine.h"

using namespace std;
using namespace cv;

#define t_c 1146;
#define t_l 1670;

std::vector<std::vector<float>> read_point_cloud_px(){

    FILE *file_points = fopen("../4_projeta/pc_pixels_concat.txt", "r");

    if (NULL == file_points)
    {
        std::cout << "\nErro: Falhou em ler o arquivo: " << std::endl;
    }else{
        std::cout<<"\nArquivo de pontos lido com sucesso!"<<std::endl;
    }

    float x,y;
    std::vector<std::vector<float>> points;

    while(!feof(file_points)){
        fscanf(file_points,"%f %f", &x, &y);
        std::vector<float> inner_coords{x,y};

        points.push_back(inner_coords);
    }

    return points;
}

vector<Segment> seleciona_linhas(vector<Segment> lines, Mat imagem){

  vector<Segment> final_lines1, final_lines2;
  float d_max = imagem.cols/6;

  // Primeiro Critério: Tamanho da linha - Se a linha for grande demais, pode ser eliminada
  for(int i=0; i < lines.size(); i++){
    
    //Distância euclidiana entre os pontos da linha
    float d = sqrt(pow((lines.at(i).x2 - lines.at(i).x1),2) + pow((lines.at(i).y2 - lines.at(i).y1),2));

    //Linhas longas demais devem ser descartadas
    if(d <= d_max){
      final_lines1.push_back(lines.at(i));
    }
  }

  std::cout<<"SEGMENTOS SELECIONADOS NA ETAPA 1: "<<final_lines1.size() <<std::endl;

  //Segundo Critério: Verificação da quantidade de pontos da nuvem que estão incluidos dentro de um buffer criado para cada
  //linha detectada.
  std::vector<std::vector<float>> nuvem_px = read_point_cloud_px();
  int window_size = 30;
  

  for(int i=0; i < final_lines1.size(); i++){

    int c_max_1 =final_lines1.at(i).x1 + window_size + t_c;
    int c_min_1 =final_lines1.at(i).x1 - window_size + t_c;

    int l_max_1 =final_lines1.at(i).y1 + window_size + t_c;
    int l_min_1 =final_lines1.at(i).y1 - window_size + t_l;

    int c_max_2 =final_lines1.at(i).x2 + window_size + t_c;
    int c_min_2 =final_lines1.at(i).x2 - window_size + t_c;

    int l_max_2 =final_lines1.at(i).y2 + window_size + t_l;
    int l_min_2 =final_lines1.at(i).y2 - window_size + t_l;
    
    int cont_vizinhos_1 = 0, cont_vizinhos_2 = 0, min_vizinhos = 700; //antes era 300

    for(int j=0; j < nuvem_px.size(); j++){
      
      if(nuvem_px.at(j).at(0) <= c_max_1 && nuvem_px.at(j).at(0) >= c_min_1){
        if(nuvem_px.at(j).at(1) <= l_max_1 && nuvem_px.at(j).at(1) >= l_min_1){

          cont_vizinhos_1++;
        }
      }

      if(nuvem_px.at(j).at(0) <= c_max_2 && nuvem_px.at(j).at(0) >= c_min_2){
        if(nuvem_px.at(j).at(1) <= l_max_2 && nuvem_px.at(j).at(1) >= l_min_2){

          cont_vizinhos_2++;
        }
      }

    }

    if(cont_vizinhos_1 >= min_vizinhos || cont_vizinhos_2 >= min_vizinhos){
        final_lines2.push_back(final_lines1.at(i));
      }

  }

  //for (int i=0; i < 5; i++){
    //std::cout<<final_lines2.at(i).x1<<", "<<final_lines2.at(i).y1<<std::endl;
  //}

  std::cout<<"NUMEROS DE SEGMENTOS SELECIONADOS NA ETAPA 2: "<<final_lines2.size()<<std::endl;

  return final_lines2;

}

int main(int argc, char* argv[]){ 
  // Seed random function
  srand((unsigned int)(time(0)));
  
  // parse arguments
  CmdLine cmd;

  string dirPath;
  string picList;
  
  bool consecutive = true;
  bool withDetection = false;
  double segment_length_threshold = 0.005;
  bool multiscale = true;

 
  Mat im = imread("img_quant.tif", IMREAD_GRAYSCALE);
  vector<Mat> imagePyramid = computeImagePyramid(im, multiscale);
  
  vector<Segment> segments = lsd_multiscale(imagePyramid, segment_length_threshold, multiscale);

  vector<Segment> segments2 = seleciona_linhas(segments, im);
  saveLines(segments, "lines/", "FX_1");

  //cout << "PROCESSED IN " << (clock() - processing_time) / float(CLOCKS_PER_SEC) << endl;
  im = imread("inner_img_rgb.tif", IMREAD_COLOR);

  if(im.empty()){
    cout<<"ERRO AO LER A IMAGEM RGB!"<<endl;
  }

  saveLinesPicture(segments2, im, "lines/", "RGB_concat_quant", false);
  
  
  return 0;
}
#include<iostream>
#include <string>
#include<cmath>
#include<fstream>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

vector<float> transform_coord(vector<float> ponto_coord){

    cv::Mat img = cv::imread("../../orienta/Photo_054.tif", cv::IMREAD_GRAYSCALE);

	int rows = img.rows;
    int columms = img.cols;

	float pixel_size_x = 0.008;
	float pixel_size_y = 0.008;

	float x_mm = 0;
	float y_mm = 0;

    int C = ponto_coord.at(0);
    int L = ponto_coord.at(1);


	x_mm = pixel_size_x * (C - ((columms-1)/2));
	y_mm = -pixel_size_y * (L - ((rows-1)/2));


    vector<float> mm_coords{x_mm,y_mm};

    return mm_coords;

}

vector<double> mono_rest(vector<float> photo_point, vector<float> orient, vector<vector<float>> point_cloud){

    float x0 = orient.at(7);
    float y0 = orient.at(8);
    float f = orient.at(6);

    double om = orient.at(3);
    double fi = orient.at(4);
    double kapa = orient.at(5);

    double Xl = orient.at(0);
    double Yl = orient.at(1);
    double Zl = orient.at(2);

    double Z = 950.0, X, Y;

    float x = photo_point.at(0);
    float y = photo_point.at(1);

    float m11 = cos(fi) * cos(kapa);
    float m12 = cos(om) *sin(kapa) + sin(om)*sin(fi)*cos(kapa);
    float m13 = sin(om) *sin(kapa) - cos(om)*sin(fi)*cos(kapa);
    
    float m21 = -cos(fi)* sin(kapa);
    float m22 = cos(om)*cos(kapa) - sin(om)*sin(fi)*sin(kapa);
    float m23 = sin(om)*cos(kapa) + cos(om)*sin(fi)*sin(kapa);

    float m31 = sin(fi);
    float m32 = -sin(om)*cos(fi);
    float m33 = cos(om)*cos(fi);

    bool iteration = true;
    float threshold = 0.5;
    int cont = 0;
    float stop = 0.1;
    float tp;
    double X_ant, Y_ant;

    while(iteration){

        cout<<"ITERAÇÃO: "<<cont<<endl;

        X = Xl + (Z-Zl) * (m11 * (x-x0) + m21*(y-y0) + m31*(-f)) / (m13*(x-x0) + m23*(y-y0) + m33*(-f));
        Y = Yl + (Z-Zl) * (m12*(x-x0) + m22*(y-y0) + m32*(-f)) / (m13*(x-x0) + m23*(y-y0) + m33*(-f));

        vector<vector<float>> neighboors;

        //Procura o ponto mais próximo na nuvem de pontos

        for (int i=0; i< point_cloud.size(); i++){

            if(X <= (point_cloud.at(i).at(0) + threshold) && X >= (point_cloud.at(i).at(0) - threshold) && Y <= (point_cloud.at(i).at(1) + threshold) && Y >= (point_cloud.at(i).at(1) - threshold)){
                neighboors.push_back(point_cloud.at(i));
            }
        }


        if(neighboors.empty()){
            cout<<"FALHOU EM ENCONTRAR OS VIZINHOS!"<<endl;
            break;
        }else{
            cout<<"ENCONTROU "<<neighboors.size()<<" VIZINHOS!"<<endl;
        }

        vector<float> dists;
        float menor = 100, i_menor;


        //Detectando o ponto com menor distância
        for (int i=0; i<neighboors.size(); i++){

            float d = sqrt(pow((neighboors.at(i).at(0) - X),2) + pow((neighboors.at(i).at(1) - Y),2));

            if(d < menor){
                menor = d;
                i_menor = i;
            }

        }

        Z = neighboors.at(i_menor).at(2);

        if (cont > 0){
            
            float dx = X - X_ant;
            float dy = Y - Y_ant;

            cout<<dx<<", "<<dy<<endl;

            tp = sqrt(pow(dx,2) + pow(dy,2));

            if((abs(dx) < 0.001 && abs(dy) < 0.001) || cont > 15){
                iteration = false;
            }
        }

        X_ant = X;
        Y_ant = Y;

        cont++;
    }

    cout<<"\n\n==============================================================="<<endl;
    cout<<"MONORESTITUIÇÃO CONCLUÍDA"<<endl;
    cout<<"VALOR FINAL TP: "<<tp<<endl;
    cout<<"Z FINAL: "<<Z<<endl;
    cout<<"==============================================================="<<endl;

    vector<double> final_point{X,Y,Z};

    return final_point;

}

std::vector<std::vector<float>> read_pointsUTM(){



    FILE *file_points = fopen("../MDE_1000m.txt", "r");

    if (NULL == file_points)
    {
        std::cout << "\nErro: Falhou em ler o arquivo: "<< std::endl;
    }else{
        std::cout<<"\nArquivo de pontos lido com sucesso!"<<std::endl;
    }

    float x,y,z, r;
    std::vector<std::vector<float>> points;

    while(!feof(file_points)){
        fscanf(file_points,"%f %f %f", &x, &y, &z, &r);
        std::vector<float> inner_coords{x,y,z};

        points.push_back(inner_coords);
    }

    return points;
}

std::vector<float> read_orientationParameters(){

    FILE *file = fopen("../../orienta/op.txt", "r");

    if (NULL == file)
    {
        std::cout << "\nErro: Falhou em ler o arquivo: "<< std::endl;
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

std::vector<std::vector<float>> read_px_points(){

    FILE* f = fopen("../obs.txt", "r");

    std::vector<std::vector<float>> px_points;

    if (NULL == f)
    {
        std::cout << "ERROR: failed to open file: limit_points.txt" << endl;
    }else{
        std::cout<<"SUCESS!"<<std::endl;
    }

    float C, L;

    while (!feof(f))
    {
        int n_args = fscanf(f, "%f %f", &C, &L);

        if (n_args != 2)
            continue;

        std::vector<float> p_aux;
        p_aux.push_back(C);
        p_aux.push_back(L);

        px_points.push_back(p_aux);

        }

        return px_points;

}

void write_limits_txt(vector<vector<double>> pontos){

    
    std::ofstream limits;
    limits.open("../limit_points.txt");

    for (int i=0; i < pontos.size(); i++){
        limits<<fixed<< pontos.at(i).at(0)<<" "<<pontos.at(i).at(1)<<endl;
    }

    limits.close();

}

int main(int argc, char** argv){

    //Lendo o arquivo com as observações
    vector<vector<float>> px = read_px_points();

    //Lendo o arquivo com os parametros de orientação
    vector<float> op = read_orientationParameters();

    //Lendo o arquivo com os pontos
    vector<vector<float>> pc = read_pointsUTM();

    //Convertendo C,L para x, y milimétrico
    vector<vector<float>> mm_points;

    for (int i=0; i<px.size(); i++){
        mm_points.push_back(transform_coord(px.at(i)));

        //cout<<mm_points.at(i).at(0)<<", "<<mm_points.at(i).at(1)<<endl;
    }

    vector<vector<double>> utm_final;

    for(int i=0; i<mm_points.size(); i++){
        utm_final.push_back(mono_rest(mm_points.at(i), op, pc));

        //cout<<utm_final.at(i).at(0)<<", "<<utm_final.at(i).at(1)<<", "<<utm_final.at(i).at(2)<<endl;
    }

    //Escrevendo as coordenadas no arquivo
    write_limits_txt(utm_final);


}
#!/bin/sh

compila_e_executa(){

    #Lendo a nuvem de pontos e capturando a área de interesse
    cd 1_read_cloud/
    cmake .

    if [ $? -eq 0 ];then 
    echo "compilado com sucesso!"
    make
    ./read_cloud MDE_1000m.txt
    else
        echo "nao compilado"
    fi

    #Classificando a área de interesse em superfície e "não superfície"

    cp MDE_1000_clip.pcd ../2_classifica

    cd ../2_classifica/
    cmake .

    if [ $? -eq 0 ];then 
    echo "compilado com sucesso!"
    make
    ./2_classifica
    else
        echo "nao compilado"
    fi

    #Extraindo os planos de tetos com o filtro morfológico

    cp non_ground.pcd ../3_elimina
    cd ../3_elimina

    cmake .

    if [ $? -eq 0 ];then 
    echo "compilado com sucesso!"
    make
    ./3_elimina non_ground.pcd
    else
        echo "nao compilado"
    fi

    #Projetando os pontos extraídos na imagem

    cp planes.txt ../4_projeta
    cd ../4_projeta

    cmake .

    if [ $? -eq 0 ];then 
    echo "compilado com sucesso!"
    make
    ./4_projeta ../orienta/Photo_054.tif
    else
        echo "nao compilado"
    fi


    #Classificando a Imagem com o K-means

    cd ..

    python3.7 k_means_example.py


    #Detectando segmentos de Linhas com o MLSD

    cd ../5_extrai_linhas

    cmake .

    if [ $? -eq 0 ];then 
    echo "compilado com sucesso!"
    make
    ./LSD_MULTISCALE
    else
        echo "nao compilado"
    fi

}

executa(){

    #Lendo o arquivo com as observações em C L
    printf "=========================================================\n"
    printf "LENDO O ARQUIVO COM AS OBSERVAÇÕES FEITAS NA IMAGEM\n"
    printf "E REFINANDO AS COORDENADAS\n"
    printf "=========================================================\n\n"

    cd 1_read_cloud/mono_rest
    ./mono_rest

    #Lendo a nuvem de pontos e capturando a área de interesse
    clear
    printf "=========================================================\n"
    printf "SELECIONANDO REGIÃO DE INTERESSE\n"
    printf "=========================================================\n\n"
    
    cd ..
    ./read_cloud MDE_1000m.txt

    #Classificando a área de interesse em superfície e "não superfície"

    cp MDE_1000_clip.pcd ../2_classifica

    cd ../2_classifica/
    clear
    printf "=========================================================\n"
    printf "ELIMINANDO PONTOS DE SUPERFÍCIE\n"
    printf "=========================================================\n\n"
    ./2_classifica

    #Extraindo os planos de tetos com o filtro morfológico
    clear
    printf "=========================================================\n"
    printf "EXTRAINDO OS PLANOS DE TELHADOS COM O FILTRO MORFOLÓGICO\n"
    printf "=========================================================\n\n"

    cp non_ground.pcd ../3_elimina
    cd ../3_elimina

    ./3_elimina non_ground.pcd

    #Projetando os pontos extraídos na imagem
    clear
    printf "=========================================================\n"
    printf "PROJETANDO OS PONTOS EXTRAÍDOS NA IMAGEM\n"
    printf "CRIANDO UMA NOVA IMAGEM APENAS COM A REGIÃO DE INTERESSE\n"
    printf "=========================================================\n\n"

    cp planes.txt ../4_projeta
    cd ../4_projeta

    ./4_projeta ../orienta/Photo_054.tif

    #Classificando a Imagem com o K-means

    cd ..
    clear
    printf "=========================================================\n"
    printf "CLASSIFICANDO A IMAGEM\n"
    printf "=========================================================\n\n"

    python3.7 k_means_example.py


    #Detectando segmentos de Linhas com o MLSD

    cd 5_extrai_linhas
    clear
    printf "=========================================================\n"
    printf "EXTRAINDO CONTORNOS DE EDIFICAÇÕES\n"
    printf "=========================================================\n\n"

    ./LSD_MULTISCALE

    printf "=========================================================\n"
    printf "\tCONCLUÍDO COM SUCESSO!\n"
    printf "=========================================================\n\n"

}

clear

printf "\n"
echo "-----------------------------------------------------------------------"
echo "ATENÇÃO: ANTES DE USAR A ROTINA, CERTIFIQUE-SE DE QUE O ARQUIVO DE
         OBSERVAÇÕES NA IMAGEM FOI DEVIDAMENTE SALVO!"
echo "-----------------------------------------------------------------------"
printf "\n\n"

read -p "DESEJA CONTINUAR? (y/n): " continuar

if [ $continuar == "n" ];then
    exit 0    
else
    printf "\n\n"
    read -p "DESEJA EXECUTAR COMPILANDO? (y/n): " compilar

    if [$compilar == y -o $compilar == Y];then
        compila_e_executa
    else
        executa
    fi
fi
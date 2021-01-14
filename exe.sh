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

python3.7 kmeans.py


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





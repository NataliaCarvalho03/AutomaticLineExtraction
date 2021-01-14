from pyclustering.cluster import cluster_visualizer
from pyclustering.cluster.xmeans import xmeans
from pyclustering.cluster.center_initializer import kmeans_plusplus_initializer
from pyclustering.utils import read_sample
from pyclustering.samples.definitions import SIMPLE_SAMPLES
import cv2 as cv
import numpy as np


# Read sample 'simple3' from file.
imagem = cv.imread('orienta/Photo_054.tif')
print(imagem.shape)
# Capturando os channels da imagem (BGR)

B = imagem[:,:, 0]
G = imagem[:,:, 1]
R = imagem[:,:, 2]

#Preparando a lista para enviar ao método xmeans

lin, col = B.shape

list_lin = [] #lista que vai armazenar os pixels de cada uma das bandas
pixels_list = [] #lista que vai armazenar em cada linha os valores R G B de cada pixel

for i in range(lin):
    for j in range(col):
        list_lin.append(R[i][j])
        #print(R[i][j])
        list_lin.append(G[i][j])
        #print(G[i][j])
        list_lin.append(B[i][j])
        #print(B[i][j])
        pixels_list.append(list_lin)
        list_lin = []
        continue

#sample = read_sample(SIMPLE_SAMPLES.SAMPLE_SIMPLE3)
sample = pixels_list


# Prepare initial centers - amount of initial centers defines amount of clusters from which X-Means will
# start analysis.
amount_initial_centers = 2
initial_centers = kmeans_plusplus_initializer(sample, amount_initial_centers).initialize()
# Create instance of X-Means algorithm. The algorithm will start analysis from 2 clusters, the maximum
# number of clusters that can be allocated is 20.
xmeans_instance = xmeans(sample, initial_centers, 20)
xmeans_instance.process()
# Extract clustering results: clusters and their centers
clusters = xmeans_instance.get_clusters() #Returns list of allocated clusters, each cluster contains indexes of objects in list of data.
centers = xmeans_instance.get_centers() #Returns list of centers for allocated clusters

print(len(clusters), len(clusters[0]))
# Visualize clustering results
'''visualizer = cluster_visualizer()
visualizer.append_clusters(clusters, sample)
visualizer.append_cluster(centers, None, marker='*')
visualizer.show() '''
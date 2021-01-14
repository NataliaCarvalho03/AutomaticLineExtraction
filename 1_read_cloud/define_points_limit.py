import cv2 as cv
import numpy as np

def readArqs(fileName):

	'''Esta função lê as observações realizadas nas duas fotografias, tanto para os pontos tie quanto para os GCP. Veja a planilha 
	   para saber como montar o arquivo foto_observ.txt
	 '''

	arq = open(fileName, 'r')

	dados1 = arq.readlines() #Armazena o arquivo como um todo

	if dados1:
	   print("Leitura realizada com sucesso!")
	else:
	   print("Leitura falhou!")

	lista_pontos = [] #Lista que armazena cada linha do arquivo como float
	lista_linhas = []

	for i in dados1:
	    dados_separados = i.split(" ") #captura cada numero separado por espaço
	    for j in dados_separados:
	        lista_linhas.append(float(j)) #converte cada número de caractere para float
	        #print(j)
	    lista_pontos.append(lista_linhas) # esta é a lista que interessa
	    lista_linhas = []

	return lista_pontos

def convertCoordinates(C, L):

	img = cv.imread('../orienta/Photo_054.tif'); #You must give the path of your image

	rows, columms, bands = img.shape #The method shape return the dimensions of the image, but here the bands are not interesting

	#print("The image size: ")
	#print(img.shape)

	pixel_size_x = 0.008
	pixel_size_y = 0.008

	x_mm = 0
	y_mm = 0

	x_mm = pixel_size_x * (C - ((columms-1)/2)) #calculate the x coordinate
	y_mm = -pixel_size_y * (L - ((rows-1)/2)) #calculate the y coordinate

	#print("The transformed coordinates: ")
	#print(x_mm, y_mm)

	transformed = [x_mm,y_mm]

	return transformed

def calculate_UTM(x, y):

    x0 = 0.217
    y0 = -0.089
    f = 34.145

    om = 0.023150572143740937
    fi = 0.002920997674262337
    kapa = -1.28518405793718

    Xl = 583801.0409474417
    Yl = 7225364.4858602155
    Zl = 2007.4897763458534

    Z = 947.0

    m11 = np.cos(fi) * np.cos(kapa)
    m12 = np.cos(om) *np.sin(kapa) + np.sin(om)*np.sin(fi)*np.cos(kapa)
    m13 = np.sin(om) *np.sin(kapa) - np.cos(om)*np.sin(fi)*np.cos(kapa)
    
    m21 = -np.cos(fi)* np.sin(kapa)
    m22 = np.cos(om)*np.cos(kapa) - np.sin(om)*np.sin(fi)*np.sin(kapa)
    m23 = np.sin(om)*np.cos(kapa) + np.cos(om)*np.sin(fi)*np.sin(kapa)

    m31 = np.sin(fi)
    m32 = -np.sin(om)*np.cos(fi)
    m33 = np.cos(om)*np.cos(fi)

    #Colinearidade inversa

    X = Xl + (Z-Zl) * (m11 * (x-x0) + m21*(y-y0) + m31*(-f)) / (m13*(x-x0) + m23*(y-y0) + m33*(-f))
    Y = Yl + (Z-Zl) * (m12*(x-x0) + m22*(y-y0) + m32*(-f)) / (m13*(x-x0) + m23*(y-y0) + m33*(-f))

    return [X, Y]

def write_arq(E1, N1, E2, N2):

    arq = open("limit_points.txt", "w")

    arq.write(str(E1) + " " + str(N1) + "\n")
    arq.write(str(E2) + " " + str(N2))

    arq.close()

points_px = readArqs("obs.txt")
nuvem = readArqs("MDE_1000m.txt")

print(nuvem[0][0], nuvem[0][1])


x1, y1 = convertCoordinates(points_px[0][0], points_px[0][1])
x2, y2 = convertCoordinates(points_px[1][0], points_px[1][1])

X1, Y1 = calculate_UTM(x1,y1)
X2, Y2 = calculate_UTM(x2,y2)

write_arq(X1, Y1, X2, Y2)

import numpy as np
import cv2 as cv
from sympy import sin
from sympy import cos
from sympy import atan
from sympy import symbols
from sympy import diff
from numpy.linalg import inv


#coordinates of principal point in milimeters
x0 = 0.217
y0 = -0.089
f = 34.145
var_p = 0.008
it = 1

#Radial Distortion coeficients

K1 = -9.3625961*10**-5
K2 = 1.0931535*10**-7
K3 = 0

#Decentering Distortion coeficients P1 and P2

P1 = 0
P2 = 0

# Functional Model: Collinearity equations
# Rotation order: R(z) R(y) R(x)
	
X, Y, Z, X0, Y0, Z0, om, fi, kapa, c  = symbols('X Y Z X0 Y0 Z0 om fi kapa c')

m11 = cos(fi) * cos(kapa)
m12 = cos(om) *sin(kapa) + sin(om)*sin(fi)*cos(kapa)
m13 = sin(om) *sin(kapa) - cos(om)*sin(fi)*cos(kapa)
 
m21 = -cos(fi)* sin(kapa)
m22 = cos(om)*cos(kapa) - sin(om)*sin(fi)*sin(kapa)
m23 = sin(om)*cos(kapa) + cos(om)*sin(fi)*sin(kapa)

m31 = sin(fi)
m32 = -sin(om)*cos(fi)
m33 = cos(om)*cos(fi)


x = - c * ((m11*(X-X0) + m12*(Y-Y0) + m13*(Z-Z0)) / (m31*(X-X0) + m32*(Y-Y0) + m33*(Z-Z0)))
y = - c * ((m21*(X-X0) + m22*(Y-Y0) + m23*(Z-Z0)) / (m31*(X-X0) + m32*(Y-Y0) + m33*(Z-Z0)))

#---------------------------------- ---------------------------------------------------------------
#----------------------------------Matriz de Pesos ------------------------------------------------
#--------------------------------------------------------------------------------------------------

lin_aux = []
matriz_P = []

#Create weight matrix
for i in range(24):
	for j in range(24):
		if i==j:
			lin_aux.append(var_p/2)
		else:
			lin_aux.append(0)
	matriz_P.append(lin_aux)
	lin_aux = []

def write_EOP(EOPs):

	file = open("eop.txt", "w")

	for i in range(len(EOPs)):
		if i < (len(EOPs)-1):
			file.write(str(EOPs[i]) + "\n")
		else:
			file.write(str(EOPs[i]))
	
	file.close()


def calculateRadialDist(x_obs, y_obs):

	#calculating r
	r = np.sqrt(x_obs**2 + y_obs**2)

	#calculating dx and dy
	dx = (K1*r**2 + K2*r**4 + K3*r**6)*x_obs
	dy = (K1*r**2 + K2*r**4 + K3*r**6)*y_obs

	x_corr = x_obs - dx
	y_corr = y_obs - dy

	corr_radialDist = [x_corr, y_corr]

	return corr_radialDist

def calculateDecentringDist(x_obs, y_obs):

	#calculating r
	r = np.sqrt(x_obs**2 + y_obs**2)

	#calculate dx and dy
	dx = P1*(r**2 + 2*x_obs**2) + 2*P2*x_obs*y_obs
	dy = 2*P1*x_obs*y_obs + P2*(r**2 + 2*y_obs**2)

	decentringDist = [dx, dy]

	return decentringDist



def photogrametricCorrection(radialList, decentringList, x_obs, y_obs): #sei lá por que eu fiz isso aqui

	x_corrected = x_obs + radialList[0] + decentringList[0]
	y_corrected = y_obs + radialList[1] + decentringList[1]

	correctedPhotocoordinates = [x_corrected, y_corrected]

	return correctedPhotocoordinates

def convertCoordinates(C, L):

	img = cv.imread('Photo_054.tif'); #You must give the path of your image

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

def readArqs():

	'''Esta função lê as observações realizadas nas duas fotografias, tanto para os pontos tie quanto para os GCP. Veja a planilha 
	   para saber como montar o arquivo foto_observ.txt
	 '''

	arq = open('gcp_observ_C_L.txt', 'r')

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

#Alocando espaço para a matriz A


#------------------------------------------------------------------------------------------------------------------
#-------------------------------------------Mouting A matrix: Photo 1 -------------------------------------------
#------------------------------------------------------------------------------------------------------------------

def calcula_matA_foto1(parametros, X0_i, Y0_i, Z0_i, omega, phi, kappa):

	a1 = diff(x,X0)
	a2 = diff(x,Y0)
	a3 = diff(x,Z0)
	a4 = diff(x,om)
	a5 = diff(x,fi)
	a6 = diff(x,kapa)
	a7 = diff(y,X0)
	a8 = diff(y,Y0)
	a9 = diff(y,Z0)
	a10 = diff(y,om)
	a11 = diff(y,fi)
	a12 = diff(y,kapa)	

	coef = [a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12]
	val_a = []

	#Calculating the differential coeficients in the parameters

	for i in parametros:
		for j in coef:
			num_a = j.evalf(subs={c: 34.145, X0: X0_i, Y0: Y0_i, Z0: Z0_i, X: i[2], Y: i[3], Z: i[4], om: omega, fi: phi, kapa: kappa})
			val_a.append(num_a)

	A = np.array(val_a, dtype = 'float').reshape(24,6)

	return A


print('---------------------------------------------------------------------------------------------')
print('Bundle Adjustment')
print("Desenvolvido por: Natália Carvalho de Amorim")
print('Prof. Orientador: E. A. Mitishita')
print('Programa de Pós-Graduação em Ciências Geodésicas')
print('Universidade Federal do Paraná - UFPR')
print('----------------------------------------------------------------------------------------------')

#---------------------------------- ---------------------------------------------------------------
#---------------------------------- Reading the file with the observations ------------------------
#----------------------------------------------------- --------------------------------------------


parametros = readArqs()
parametros_copy = parametros

for i in parametros_copy:
	mm_coord = convertCoordinates(i[0], i[1]) #Convert observations to mm
	i[0] = mm_coord[0] - x0 # Convert to photogrammetric coordinate system and correct lens distortions
	i[1] = mm_coord[1] - y0 # Convert to photogrammetric coordinate system and correct lens distortions

#---------------------------------- -----------------------------------------------------------------------------
#---------------------------------- Calculating aproximations ---------------------------------------------------
#---------------------------------- -----------------------------------------------------------------------------

sum_X = 0
sum_Y = 0
sum_Z = 0

for i in parametros_copy:
	sum_X = sum_X + i[2]
	sum_Y = sum_Y + i[3]
	sum_Z = sum_Z + i[4]

real_dist = np.sqrt((parametros_copy[11][2] - parametros_copy[10][2])**2 + (parametros_copy[11][3] - parametros_copy[10][3])**2)
photo_dist = np.sqrt((parametros_copy[11][0] - parametros_copy[10][0])**2 + (parametros_copy[11][1] - parametros_copy[10][1])**2)

real_dist = real_dist * 1000

E = real_dist/photo_dist

H = 1000

print(real_dist, photo_dist, E, f, H)

X0_ini = sum_X/len(parametros_copy)
Y0_ini = sum_Y/len(parametros_copy)
Z0_ini = sum_Z/len(parametros_copy) + H

print('Z médio: ', sum_Z/len(parametros_copy))

omega = 0
phi = 0

# --------------------------------- Determining initial Kappa --------------------------------------
# Choosed points: pt10 and pt11
yp = (parametros_copy[11][1] - parametros_copy[10][1]) / (parametros_copy[11][0] - parametros_copy[10][0])
yg = (parametros_copy[11][3] - parametros_copy[10][3]) / (parametros_copy[11][2] - parametros_copy[10][2])

ang_yp = atan(yp)
ang_yg = atan(yg)

kappa = ang_yg - ang_yp

#kappa = 2.35619

print('Initial aproximation for kappa (rad): ', kappa)

#-----------------------------------------------------------------------

stop = False
threshold = 0.00001
it = 1

#---------------------------------- -----------------------------------------------------------------------------
#---------------------------------- -----------------------------------------------------------------------------
#---------------------------------- -----------------------------------------------------------------------------

while not stop:

	print('----------------------- Iteração: ', it, '-----------------------------')

	matrix_A = calcula_matA_foto1(parametros_copy, X0_ini, Y0_ini, Z0_ini, omega, phi, kappa)

	L_calc = []

	for i in parametros_copy:
		L_calc.append(x.evalf(subs={c: 34.145, X0: X0_ini, Y0: Y0_ini, Z0: Z0_ini, X: i[2], Y: i[3], Z: i[4],  om: omega, fi: phi, kapa: kappa}))
		L_calc.append(y.evalf(subs={c: 34.145, X0: X0_ini, Y0: Y0_ini, Z0: Z0_ini, X: i[2], Y: i[3], Z: i[4],  om: omega, fi: phi, kapa: kappa}))
	#---------------------------------- -----------------------------------------------------------------------------
	#---------------------------------- -----------------------------------------------------------------------------
	#---------------------------------- -----------------------------------------------------------------------------

	L_obs = []

	for i in parametros_copy:
		L_obs.append(i[0])
		L_obs.append(i[1])

	L = []

	for i in range(len(L_calc)):
		dif = L_obs[i] - L_calc[i]
		L.append(dif)

	L = np.array(L, dtype='float').reshape(24,1)

	#---------------------------------- -----------------------------------------------------------------------------
	#---------------------------------- -----------------------------------------------------------------------------
	#---------------------------------- -----------------------------------------------------------------------------

	At = np.transpose(matrix_A)
	part1 = np.dot(At, matrix_A)
	part2 = np.dot(At, L)

	i_part1 = inv(part1)

	Xv = np.dot(i_part1,part2)

	X0_ini = X0_ini + float(Xv[0][0])
	Y0_ini = Y0_ini + float(Xv[1][0])
	Z0_ini = Z0_ini + float(Xv[2][0])

	omega = omega + float(Xv[3][0])
	phi = phi + float(Xv[4][0])
	kappa = kappa + float(Xv[5][0])


	if (np.absolute(np.amax(Xv)) <= threshold) or (it ==10):
		stop = True

	it = it+1

residuos = np.dot(matrix_A, Xv) - L

print('------------Resultado final: -------------------')
print('X0: ', X0_ini)
print('Y0: ', Y0_ini)
print('Z0: ', Z0_ini)
print('om: ', omega)
print('phi: ', phi)
print('kapa: ', kappa)

Z0_ini = Z0_ini

eop = [X0_ini, Y0_ini, Z0_ini, omega, phi, kappa]

print('------------ Resíduos ---------------------------')
for i in residuos:
	print(i)

write_EOP(eop)


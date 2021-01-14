import numpy as np

om = 0
fi = -0.1436
kapa = -0.139626

m11 = np.cos(fi) * np.cos(kapa)
m12 = np.cos(om) *np.sin(kapa) + np.sin(om)*np.sin(fi)*np.cos(kapa)
m13 = np.sin(om) *np.sin(kapa) - np.cos(om)*np.sin(fi)*np.cos(kapa)
 
m21 = -np.cos(fi)* np.sin(kapa)
m22 = np.cos(om)*np.cos(kapa) - np.sin(om)*np.sin(fi)*np.sin(kapa)
m23 = np.sin(om)*np.cos(kapa) + np.cos(om)*np.sin(fi)*np.sin(kapa)

m31 = np.sin(fi)
m32 = -np.sin(om)*np.cos(fi)
m33 = np.cos(om)*np.cos(fi)

rotation = [m11, m12, m13, m21, m22, m23, m31, m32, m33]

R = np.array(rotation).reshape(3,3)

c = 34.145

X = 1070.819
Y = 851.617
Z = 3.021


X0 = 828.1381
Y0 = 692.75
Z0 = 596.0608

for i in R:
    print(i)

x = - c * ((m11*(X-X0) + m12*(Y-Y0) + m13*(Z-Z0)) / (m31*(X-X0) + m32*(Y-Y0) + m33*(Z-Z0)))

print(x)
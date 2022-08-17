import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def get_input(file):
    return_x = []
    return_y = []
    return_z = []
    return_w = []
    with open(file,"r") as read_file:
        lines = read_file.read().split("\n")[1::]
        for line in lines:
            if(line != ''):
                elements = line.split(",")
                return_x.append(float(elements[0]))
                return_y.append([float(elements[1]),float(elements[2]),float(elements[3])])
                # return_y.append(float(elements[1]))
                # return_z.append(float(elements[2]))
                # return_w.append(float(elements[3]))
    
    return np.asarray(return_x).reshape(len(return_x),1),np.asarray(return_y)
    # return return_x,return_y,return_z,return_w

def reorder(x,y):
    new = []
    for i in range(len(x)):
        new.append([x[i],y[i]])
    return new

def get_xy(data):
    new = {}
    for i in range(len(data)):
        new[data[i][0][0]] = data[i][1][0]

    return list(new.keys()),list(new.values())

def get_xz(data):
    new = {}
    for i in range(len(data)):
        new[data[i][0][0]] = data[i][1][1]

    return list(new.keys()),list(new.values())

def get_xw(data):
    new = {}
    for i in range(len(data)):
        new[data[i][0][0]] = data[i][1][2]

    return list(new.keys()),list(new.values())


def Gauss(x, A, B):
    y = A*np.exp(-1*B*np.array(x)**2)
    return y

def test(x, a, b):
    return a * np.sin(b * np.array(x))

def func(x, a, b, c):
    return a * np.exp(-b * np.array(x)) + c

data = (get_input("optimization_data.csv"))
data = reorder(data[0],data[1])
data.sort(key=lambda x:x[0])

x,y = get_xy(data)
_,z = get_xz(data)
_,w = get_xw(data)

# fit_angle = 
parameters, covariance = curve_fit(test, np.asarray(x), np.asarray(y))
fit_A = parameters[0]
fit_B = parameters[1]
fit_y = test(x, fit_A, fit_B)


plt.plot(x,y,'-',label="Angle")
plt.plot(x, fit_y, '-', label='Fit')
plt.legend()
plt.show()

plt.plot(x,z,'-',label="Top")
plt.legend()
plt.show()


plt.plot(x,w,'-',label="Bottom")
plt.legend()
plt.show()
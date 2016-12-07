from math import *
import numpy as np
import pylab as plt
from scipy import linalg

x0, y0 = 0, 0
x1, y1 = 1, 1
t0, t1 = 0, 1

xd0, yd0 = 2, 0
xd1, yd1 = 0, -1
dt = t1 - t0

dx = x1 - x0
dy = y1 - y0

a =  xd0*(t1-t0) - (x1 - x0)
b = -xd1*(t1-t0) + (x1 - x0)
c =  yd0*(t1-t0) - (y1 - y0)
d = -yd1*(t1-t0) + (y1 - y0)

t = np.linspace(t0,t1,100)
def z(t):
    return (t - t0)/(t1 - t0)

zdot = 1/(t1-t0)

def xf(t):
    tr = (1 - t)*x0
    tr = tr + z(t)*x1
    tr = tr + z(t)*(1-z(t))*(a*(1-z(t))+b*z(t))
    return tr

def yf(t):
    tr = (1 - z(t))*y0
    tr = tr + z(t)*y1
    tr = tr + z(t)*(1-z(t))*(c*(1-z(t))+d*z(t))
    return tr

def dxf(t):
    tr = x1 - x0
    tr = tr + (1-2*z(t))*(a*(1-z(t)) + b*z(t))
    tr = tr + z(t)*(1-z(t))*(b-a)
    tr = tr * zdot
    return tr

def dyf(t):
    tr = y1 - y0
    tr = tr + (1-2*z(t))*(c*(1-z(t)) + d*z(t))
    tr = tr + z(t)*(1-z(t))*(d-c)
    tr = tr * zdot
    return tr

def ddxf(t):
    tr = -2*(a*(1-z(t)) +b*z(t))
    tr = tr + 2*(1-2*z(t))*(b-a)
    tr = tr * zdot
    return tr 

def ddyf(t):
    tr = -2*(c*(1-z(t)) +d*z(t))
    tr = tr + 2*(1-2*z(t))*(d-c)
    tr = tr * zdot
    return tr 

x = xf(t)
y = yf(t)
ptx = np.array([x0,x1])
pty = np.array([y0,y1])

plt.figure()
plt.xlim(-.5,1.5)
plt.ylim(-.5,1.5)
plt.plot(ptx,pty, 'ro', x,y,'g-')
plt.legend(['Data','Interpolant'], loc='best')
plt.title('Cubic Spline')
plt.show()

r = 10.0
v= np.sqrt(dxf(t)*dxf(t) + dyf(t)*dyf(t))
kappa = (dxf(t)*ddyf(t) - dyf(t*ddxf(t)))/(v*v*v)
dotphi1 = (v/r)*( kappa + 1)
dotphi2 = (v/r)*(-kappa + 1)
plt.plot(t,dotphi1,'b-',t,dotphi2,'g-')
plt.title('Wheel Speeds')
plt.legend(['Right','Left'],loc='best')

plt.show()
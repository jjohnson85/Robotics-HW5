from math import *
import numpy as np
import pylab as plt
from scipy import linalg

x0, y0 = 0, 0
x1, y1 = 1, 1
t0, t1 = 0, 3

xd0, yd0 = 2, 0
xd1, yd1 = 0, -1
dt = t1 - t0

r = .05
L = .15/2+.02/2

speed = 1

rate = 1.0/10

a =  xd0*(t1-t0) - (x1-x0)
b = -xd1*(t1-t0) + (x1-x0)
c =  yd0*(t1-t0) - (y1-y0)
d = -yd1*(t1-t0) + (y1-y0)

t = np.linspace(t0,t1,t1/rate)
def z(t):
    return (t - t0)/(t1 - t0)

zdot = 1/(t1-t0)

def setABCD():
    global a, b, c, d, t1, t0
    a =  xd0*(t1-t0) - (x1 - x0)
    b = -xd1*(t1-t0) + (x1 - x0)
    c =  yd0*(t1-t0) - (y1 - y0)
    d = -yd1*(t1-t0) + (y1 - y0)

def setTime():
    global t0, t1, speed, dt, xdot, t
    dist = sqrt((x0-x1)**2.0 + (y0-y1)**2.0)
    t0 = 0
    t1 = dist/speed
    print dist, speed, t1
    dt = t1 - t0
    zdot = 1.0/(t1-t0)
    t = np.linspace(t0,t1,t1/rate)
    return

def xf(t):
    tr = (1.0 - t)*x0
    tr = tr + z(t)*x1
    tr = tr + z(t)*(1-z(t))*(a*(1.0-z(t))+b*z(t))
    return tr

def yf(t):
    tr = (1.0 - z(t))*y0
    tr = tr + z(t)*y1
    tr = tr + z(t)*(1.0-z(t))*(c*(1.0-z(t))+d*z(t))
    return tr

def dxf(t):
    global x1, x0, a, b
    tr = x1 - x0
    tr = tr + (1.0-2.0*z(t))*(a*(1.0-z(t)) + b*z(t))
    tr = tr + z(t)*(1.0-z(t))*(b-a)
    tr = tr * zdot
    return tr

def dyf(t):
    global y1, y0, c, d
    tr = y1 - y0
    print y1, y0
    tr = tr + (1.0-2.0*z(t))*(c*(1.0-z(t)) + d*z(t))
    tr = tr + z(t)*(1.0-z(t))*(d-c)
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

def GetTheta(inX0,inY0,inX1,inY1,inXd0,inYd0, inSpeed):
    global x0, y0, x1, y1, xd0, yd0, xd1, yd1, speed, r
    x0 = inX0
    y0 = inY0
    x1 = inX1
    y1 = inY1
    xd0 = inXd0
    yd0 = inYd0
    
    xd1 = (x1-x0)/sqrt((x1-x0)**2+(y1-y0)**2)*inSpeed
    yd1 = (y1-y0)/sqrt((x1-x0)**2+(y1-y0)**2)*inSpeed
    
    speed = inSpeed

    setTime()
    setABCD()

    x = xf(t)
    y = yf(t)
    ptx = np.array([x0,x1])
    pty = np.array([y0,y1])

    #plt.figure()
    #plt.plot(ptx,pty, 'ro', x,y,'g-')
    #plt.legend(['Data','Interpolant'], loc='best')
    #plt.title('Cubic Spline')
    #plt.show()

    v = np.sqrt(dxf(t)*dxf(t) + dyf(t)*dyf(t))
    print dxf(t), dyf(t)
    kappa = (dxf(t)*ddyf(t) - dyf(t*ddxf(t)))/(v*v*v)
    dotphi1 = (v/r)*( kappa + 1)
    dotphi2 = (v/r)*(-kappa + 1)

    return dotphi1, dotphi2

#GetTheta(0.0,0.0,3.0,2.0,1.0,0.0, 4.0)

from math import *
import numpy as np
import pylab as plt
from scipy import linalg

t0 , t1 = 0 , 2
x0 , y0 = 1 , -1
x1 , y1 = 3 , 4
xd0 , yd0 = 1 , -1
xd1 = 0
yd1 = 2
dt = ( t1-t0 )
dx = ( x1-x0 )
dy = ( y1-y0 )
a = xd0 * dt - dx
b = -xd1 * dt+dx
c = yd0 * dt - dy
d = -yd1 * dt+dy
t = np . linspace ( t0 , t1 , 100 )
dotz = 1.0 / dt
z = ( dotz ) * ( t-t0 )
x = (1-z ) * x0 + z* x1+z*(1-z ) * ( a*(1-z )+b*z )
y = (1-z ) * y0 + z* y1+z*(1-z ) * ( c*(1-z )+d*z )
ptx = np.array ( [ x0 , x1 ] )
pty = np.array ( [ y0 , y1 ] )
plt.figure ( )

plt.plot ( ptx , pty , 'ro' , x , y , 'g-' )
plt.legend ( [ 'Data' , 'Interpolant' ] , loc='best' )
plt.title ( 'Cubic Spline' )
plt.show ( )
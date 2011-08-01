from numpy import matrix, linalg
from math import cos, sin, sqrt

def wip(ax,ay,az,x_0,y_0,z_0):
    norm = sqrt(ax**2 + ay**2 + az**2)
    if(norm>0):
        [ax,ay,az] = [ax/norm,ay/norm,az/norm]
    x_0 = (-1)*x_0
    z_0 = (-1)*z_0    
    # Formulas from MatLab
    A = matrix([[cos(x_0)*sin(z_0) - cos(z_0)*sin(x_0)*sin(y_0)  , cos(x_0)*cos(y_0)*cos(z_0), cos(z_0)*sin(x_0) - cos(x_0)*sin(y_0)*sin(z_0)], \
    [- cos(x_0)*cos(z_0) - sin(x_0)*sin(y_0)*sin(z_0), cos(x_0)*cos(y_0)*sin(z_0), sin(x_0)*sin(z_0) + cos(x_0)*cos(z_0)*sin(y_0)],\
    [-cos(y_0)*sin(x_0)                        ,       -cos(x_0)*sin(y_0), 0                                     ]])

    b = matrix([[ax - sin(x_0)*sin(z_0) + x_0*(cos(x_0)*sin(z_0) - cos(z_0)*sin(x_0)*sin(y_0)) + z_0*(cos(z_0)*sin(x_0) - cos(x_0)*sin(y_0)*sin(z_0)) - cos(x_0)*cos(z_0)*sin(y_0) + y_0*cos(x_0)*cos(y_0)*cos(z_0)],\
    [ay + cos(z_0)*sin(x_0) - x_0*(cos(x_0)*cos(z_0) + sin(x_0)*sin(y_0)*sin(z_0)) + z_0*(sin(x_0)*sin(z_0) + cos(x_0)*cos(z_0)*sin(y_0)) - cos(x_0)*sin(y_0)*sin(z_0) + y_0*cos(x_0)*cos(y_0)*sin(z_0)],\
    [az - cos(x_0)*cos(y_0) - x_0*cos(y_0)*sin(x_0) - y_0*cos(x_0)*sin(y_0)]])
    try:
        sol = linalg.lstsq(A,b)
    except:
        print 'lstsq did not converge!'
    sol = sol[0]
    return [(-1)*sol.item(0),sol.item(1),(-1)*sol.item(2)]

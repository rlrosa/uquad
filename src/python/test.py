from sympy import Matrix,Symbol,cos,sin

def wip(ax,ay,az,x_0,y_0,z_0):
    x = Symbol('x')
    y = Symbol('y')
    z = Symbol('z')
    x_0 = (-1)*x_0
    z_0 = (-1)*z_0    
    # Formulas from MatLab
    A = matrix(3,3,[cos(z)*sin(x)*sin(y) - cos(x)*sin(z), -cos(x)*cos(y)*cos(z),  cos(x)*sin(y)*sin(z) - cos(z)*sin(x), \
    cos(x)*cos(z) + sin(x)*sin(y)*sin(z)  , -cos(x)*cos(y)*sin(z),  - sin(x)*sin(z) - cos(x)*cos(z)*sin(y), \
    -cos(y)*sin(x),-cos(x)*sin(y),0])    
    b = matrix(3,1,[ ax + sin(x_0)*sin(z_0) - x_0*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)) - z_0*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)) + cos(x_0)*cos(z_0)*sin(y_0) - y_0*cos(x)*cos(y)*cos(z),\
    ay - cos(z_0)*sin(x_0) + x_0*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)) - z_0*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)) + cos(x_0)*sin(y_0)*sin(z_0) - y_0*cos(x)*cos(y)*sin(z),\
    az - cos(x_0)*cos(y_0) - x_0*cos(y)*sin(x) - y_0*cos(x)*sin(y)])
    sol = A.Inv()*b
    return [(-1)*sol[0],sol[1],(-1)*sol[2]]

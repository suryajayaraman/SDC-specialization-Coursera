import sympy
from sympy import symbols, diff

xk, yk, thetak = symbols('x_k y_k theta_k', real=True)
xl, yl = symbols('x_l y_l', real=True)
d = symbols('d_', real = True)

h1 = ( (xl - xk - d*sympy.cos(thetak))**2 + (yl - yk - d*sympy.sin(thetak))**2 )**0.5
print(' partial derivative of h1 wrt xk is   :')
print(diff(h1,xk))
print('\n')
print(' partial derivative of h1 wrt yk is   :')
print(diff(h1,yk))
print('\n')
print(' partial derivative of h1 wrt thetak is   :')
print(diff(h1,thetak))
print('\n')

h2 = sympy.atan2( (yl - yk - d*sympy.sin(thetak)), (xl - xk - d*sympy.cos(thetak)) ) - thetak
print(' partial derivative of h2 wrt xk is   :')
print(diff(h2,xk))
print('\n')
print(' partial derivative of h2 wrt yk is   :')
print(diff(h2,yk))
print('\n')
print(' partial derivative of h2 wrt thetak is   :')
print(diff(h2,thetak))

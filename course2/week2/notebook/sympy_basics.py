Python 2.7.14 (v2.7.14:84471935ed, Sep 16 2017, 20:19:30) [MSC v.1500 32 bit (Intel)] on win32
Type "copyright", "credits" or "license()" for more information.
>>> import sympy
>>> from sympy import symbols, diff
>>> x, y, z = symbols('x y z', real=True)
>>> import math
>>> f = 4*x*y + x*sin(z) + x**3 + z**8*y

Traceback (most recent call last):
  File "<pyshell#4>", line 1, in <module>
    f = 4*x*y + x*sin(z) + x**3 + z**8*y
NameError: name 'sin' is not defined
>>> f = 4*x*y + x*math.sin(z) + x**3 + z**8*y

Traceback (most recent call last):
  File "<pyshell#5>", line 1, in <module>
    f = 4*x*y + x*math.sin(z) + x**3 + z**8*y
  File "C:\Python27\lib\site-packages\sympy\core\expr.py", line 256, in __float__
    raise TypeError("can't convert expression to float")
TypeError: can't convert expression to float
>>> f = 4*x*y + x* sympy.sin(z) + x**3 + z**8*y
>>> diff(f, x)
3*x**2 + 4*y + sin(z)
>>> x1, y1, z1 = symbols('x y z', real=True)
>>> f = 4*x*y + x* sympy.sin(z) + x**3 + z**8*y
>>> diff(f, x)
3*x**2 + 4*y + sin(z)
>>> sympy.atan2(y,x)
atan2(y, x)
>>> type(xl)

Traceback (most recent call last):
  File "<pyshell#12>", line 1, in <module>
    type(xl)
NameError: name 'xl' is not defined
>>> xl

Traceback (most recent call last):
  File "<pyshell#13>", line 1, in <module>
    xl
NameError: name 'xl' is not defined
>>> x1
x
>>> symbols('x y z', real=True)
(x, y, z)
>>> sympy.atan2(y1,x1)
atan2(y, x)
>>> 

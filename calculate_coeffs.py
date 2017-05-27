import sympy
from sympy.solvers import solve

def calculate_coeffs():
    """
    Functions that calculates coefficients 
    for list of sympy equations.
    """

    #defining symbols used for further calculation
    t = sympy.Symbol('t')
    b0, b1, b2, b3, b4 = sympy.symbols('b0 b1 b2 b3 b4')
    b5, b6, b7, b8, b9 = sympy.symbols('b5 b6 b7 b8 b9')
    x0, dx0, d2x0, d3x0, d4x0, d5x0, d6x0, d7x0, d8x0, d9x0 = sympy.symbols('x0 dx0 d2x0 d3x0 d4x0 d5x0 d6x0 d7x0 d8x0 d9x0')
    xk, dxk, d2xk, d3xk, d4xk, d5xk, d6xk, d7xk, d8xk, d9xk = sympy.symbols('xk dxk d2xk d3xk d4xk d5xk d6xk d7xk d8xk d9xk')

    #defining polinomial trajectory
    x = b0 + b1*t + b2*(t**2) + b3*(t**3) + b4*(t**4)\
         + b5*(t**5) + b6*(t**6) + b7*(t**7) + b8*(t**8) + b9*(t**9)


    #defining derivations of this polinomial
    dx = sympy.diff(x, t)
    d2x = sympy.diff(dx, t) #second derivation
    d3x = sympy.diff(d2x, t) #third derivation
    d4x = sympy.diff(d3x, t) #fourth derivation
    d5x = sympy.diff(d4x, t) #fifth derivation
    d6x = sympy.diff(d5x, t) #sixth derivation
    d7x = sympy.diff(d6x, t) #seventh derivation

    equations = [

        sympy.Eq(x0 - b0),
        sympy.Eq(dx0 - b1),
        sympy.Eq(d2x0 - 2*b2),
        sympy.Eq(d3x0 - 6*b3),
        sympy.Eq(d4x0 - 24*b4),
        sympy.Eq(xk - x),
        sympy.Eq(dxk - dx),
        sympy.Eq(d2xk - d2x),
        sympy.Eq(d3xk - d3x),
        sympy.Eq(d4xk - d4x),
    ]

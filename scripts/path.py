from math import cos, sin, pi
from numpy import vectorize

# initial position of the drone
init_x = 0.0
init_y = 0.0
init_z = 0.0


# function of the planned path
def p_x(t):
    return cos(pi / 5 * t)
    #return init_x - 2 + 2 * cos(pi / 10 * t)


def p_y(t):
    return sin(pi / 5 * t)
    #return init_y + 2 * sin(pi / 5 * t)


def p_z(t):
    return 1
    #return init_z + 0.4 * t


def p_vx(t):
    return 0.2 * pi * sin(pi/5*t)
    #return -0.2 * pi * sin(pi / 10 * t)


def p_vy(t):
    return 0.2 * pi * cos(pi/5*t)
    #return 0.4 * pi * cos(pi / 5 * t)


def p_vz(t):
    return 0
    #return -0.4


# vectorized function of the path
x = vectorize(p_x)
y = vectorize(p_y)
z = vectorize(p_z)
vx = vectorize(p_vx)
vy = vectorize(p_vy)
vz = vectorize(p_vz)

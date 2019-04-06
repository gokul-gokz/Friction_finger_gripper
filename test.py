import scipy.optimize as opt
from numpy import exp
import timeit
from sympy import *
import numpy as np


st1 = timeit.default_timer()
FINGER_END = 15
FINGER_START = 0
SLIDING_RESOLUTION = 0.1
ROTATION_SLIDE_RESOLUTION=0.2
PALM_WIDTH = 5
OBJECT_SIZE= 2.5
TH1_MAX=142.5
TH2_MAX=37.5
FINGER_WIDTH=1
left_position=5.5
right_position=5
def f(variables) :
    (th1,th2) = variables

    eqn1 = FINGER_WIDTH*sin(th1)+FINGER_WIDTH*sin(th2)+left_position * cos(th1) + OBJECT_SIZE * sin(th1) - PALM_WIDTH - right_position * cos(th2)
    eqn2 =-FINGER_WIDTH*cos(th1)-FINGER_WIDTH*cos(th2)+left_position * sin(th1) - OBJECT_SIZE * cos(th1) - right_position * sin(th2)

    eqn3 = FINGER_WIDTH * sin(th1) + FINGER_WIDTH * sin(th2) + left_position * cos(th1) + OBJECT_SIZE * sin(th2) - PALM_WIDTH - right_position * cos(th2)
    eqn4 = -FINGER_WIDTH * cos(th1) - FINGER_WIDTH * cos(th2) + left_position * sin(th1) - OBJECT_SIZE * cos(th2) - right_position * sin(th2)


    return [eqn3, eqn4]

solution = opt.fsolve(f, (0.1,1) )
print(solution)


st2 = timeit.default_timer()
print("RUN TIME : {0}".format(st2-st1))
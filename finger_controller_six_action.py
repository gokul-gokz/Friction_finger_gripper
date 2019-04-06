import time
import math

from mpmath import *
from sympy import *
import numpy as np

FINGER_END = 15
FINGER_START = 0
SLIDING_RESOLUTION = 0.1
ROTATION_SLIDE_RESOLUTION=0.2


def angle_conversion(angle, flag):
    if (flag == 1):
        n_angle = (angle * 180 / 3.14 - 21) * 0.003246 + 0.5626
        print("Gripper_angle=", angle * 180 / 3.14)

    else:
        n_angle = (angle * 180 / 3.14 - 152) * (-0.002113) + 0.3794
        print("Gripper_angle=", angle * 180 / 3.14)
    print("n_angle", n_angle)
    return (n_angle)


def ik_finger(x, y, w0, wp, fw):
    t2, d2 = symbols('t2 d2')
    eqn1 = (d2 - w0 / 2) * cos(t2) + (fw + w0 / 2) * sin(t2)
    eqn2 = (d2 - w0 / 2) * sin(t2) - (fw + w0 / 2) * cos(t2)
    eqn3 = x ** 2 + y ** 2 - eqn1 ** 2 - eqn2 ** 2
    sold2 = solve(eqn3, d2)
    solt2 = solve(eqn1.subs(d2, sold2[1]) - x, t2)

    d2v = np.array([sold2[1] * cos(solt2[1]), sold2[1] * sin(solt2[1])])
    w0v = np.array([w0 * sin(solt2[1]), -w0 * cos(solt2[1])])
    wpv = np.array([wp, 0])
    f1v = np.array([fw * sin(solt2[1]), -fw * cos(solt2[1])])
    av = d2v + f1v + w0v - wpv

    d1 = sqrt((av * av).sum() - fw * fw)
    t1 = np.arctan2(float(av[1]), float(av[0])) - np.arctan2(float(fw), float(d1))
    return t1, solt2[1], d1, sold2[1]


def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    '''
    Python 2 implementation of Python 3.5 math.isclose()
    https://hg.python.org/cpython/file/tip/Modules/mathmodule.c#l1993
    '''
    # sanity check on the inputs
    if rel_tol < 0 or abs_tol < 0:
        raise ValueError("tolerances must be non-negative")

    # short circuit exact equality -- needed to catch two infinities of
    # the same sign. And perhaps speeds things up a bit sometimes.
    if a == b:
        return True

    # This catches the case of two infinities of opposite sign, or
    # one infinity and one finite number. Two infinities of opposite
    # sign would otherwise have an infinite relative tolerance.
    # Two infinities of the same sign are caught by the equality check
    # above.
    if math.isinf(a) or math.isinf(b):
        return False

    # now do the regular computation
    # this is essentially the "weak" test from the Boost library
    diff = math.fabs(b - a)
    result = (((diff <= math.fabs(rel_tol * b)) or
               (diff <= math.fabs(rel_tol * a))) or
              (diff <= abs_tol))
    return result

def cost_function(CUR_POS,GOAL_POS,CUR_ORI,GOAL_ORI):

    if CUR_POS>GOAL_POS:
        COST_T=-1
    elif CUR_POS<GOAL_POS:
        COST_T =1
    else:
        COST_T=0

    if CUR_ORI>GOAL_ORI:
        COST_R=-1
    elif CUR_ORI<GOAL_ORI:
        COST_R=1
    else:
        print(CUR_ORI)
        COST_R=0
    return COST_T,COST_R


def limit_check(position):
    if(position<FINGER_END and position>FINGER_START):
        return True
    else:
         return False


class node:
    def __init__(self):
        self.position_l = 0
        self.position_r = 0
        self.orientation = 0
        self.parent = None
        self.action = None



    def find_neighbours(self, COST_T, COST_R):
        neighbours = []

        if (limit_check(self.position_l + SLIDING_RESOLUTION) and limit_check(self.position_r - SLIDING_RESOLUTION) and self.parent != "l_minus") and COST_T>0:
            l_plus = node()
            l_plus.position_l = self.position_l + SLIDING_RESOLUTION
            l_plus.position_r = self.position_r
            l_plus.orientation = self.orientation
            l_plus.action = "l_plus"
            l_plus.parent = self
            neighbours.append(l_plus)


        if (limit_check(self.position_l - SLIDING_RESOLUTION) and limit_check(self.position_r + SLIDING_RESOLUTION) and self.parent != "l_plus") and COST_T > 0:
            l_minus = node()
            l_minus.position_l = self.position_l - SLIDING_RESOLUTION
            l_minus.position_r = self.position_r
            l_minus.orientation = self.orientation
            l_minus.action = "l_minus"
            l_minus.parent = self
            neighbours.append(l_minus)

        if (limit_check(self.position_l + SLIDING_RESOLUTION) and limit_check(self.position_r - SLIDING_RESOLUTION) and self.parent != "r_plus") and COST_T>0:
            r_minus = node()
            r_minus.position_l = self.position_l
            r_minus.position_r = self.position_r - SLIDING_RESOLUTION
            r_minus.orientation = self.orientation
            r_minus.action = "r_minus"
            r_minus.parent = self
            neighbours.append(r_minus)

        if (limit_check(self.position_l - SLIDING_RESOLUTION) and limit_check(self.position_r + SLIDING_RESOLUTION) and self.parent != "r_minus") and COST_T > 0:
            r_plus=node()
            r_plus.position_l = self.position_l
            r_plus.position_r = self.position_r + SLIDING_RESOLUTION
            r_plus.orientation = self.orientation
            r_plus.action = "r_plus"
            r_plus.parent = self
            neighbours.append(r_plus)

        # if (self.position + ROTATION_SLIDE_RESOLUTION <= FINGER_END and self.position >= FINGER_START and self.parent != "o_minus" and COST_R>0):
        #     o_plus.position = self.position + ROTATION_SLIDE_RESOLUTION
        #     o_plus.orientation = self.orientation + 90
        #     o_plus.action = "o_plus"
        #     o_plus.parent = self
        #     neighbours.append(o_plus)
        #
        # if (self.position + ROTATION_SLIDE_RESOLUTION <= FINGER_END and self.position >= FINGER_START and self.parent != "o_plus" and COST_R<0):
        #     o_minus.position = self.position+ROTATION_SLIDE_RESOLUTION
        #     o_minus.orientation = self.orientation - 90
        #     o_minus.action = "o_minus"
        #     o_minus.parent = self
        #     neighbours.append(o_minus)

        return neighbours


def bfs(start, goal):
    start_time = time.time()
    queue = [start]
    cur = queue.pop()

    while (not(isclose(cur.position_l, goal.position_l, rel_tol=1e-09, abs_tol=0.0) and isclose(cur.position_r, goal.position_r, rel_tol=1e-09, abs_tol=0.0) and (isclose(cur.orientation, goal.orientation, rel_tol=1e-09, abs_tol=0.0)))) :
        #if((cur.position == goal.position) and (cur.orientation == goal.orientation)):
            #print ("yes")
            #break
        #COST_T,COST_R=cost_function(cur.position,goal.position,cur.orientation,goal.orientation)
        queue = queue + cur.find_neighbours(1,0)
        cur = queue.pop(0)
        print (cur.position_l)
        print (cur.position_r)
        # print (goal.position)
        # print (cur.orientation)
        # print len(queue)
        # print ""
        # print(isclose(cur.position, goal.position, rel_tol=1e-09, abs_tol=0.0) and (isclose(cur.orientation, goal.orientation, rel_tol=1e-09, abs_tol=0.0)))

    res = backtrace(cur)

    end_time = time.time()

    return res


def backtrace(cur):
    path = []
    while not cur.parent == None:
        path.append(cur.action)
        cur = cur.parent
    return path


def high_level_plan(start, goal):
    l = bfs(start, goal)
    print l
    prev_action = l[0]
    count = 0
    flag = 0
    path = []
    final_action = ""
    print range(len(l))
    for i in range(len(l)):

        #print i,len(l)-1
        action = l[i]

        #print action, prev_action
        if action == prev_action:
            count = count + 1
            #print count
            flag = 1
            prev_action = action
            if(i!=(len(l)-1)):
                continue

        if (action != prev_action and flag == 1) or i == (len(l)-1) or i == 0:
            final_action = prev_action + str(count)
            flag = 0
            path.append(final_action)
            prev_action = action
            count=1
            continue

        if not flag:
            path.append(action)
            prev_action = action

    print(path)


start = node()
goal = node()
start.position_l = 1.2
start.position_r = 8.0
start.orientation = 0
start.parent = None

goal.position_l = 1.5
goal.position_r = 7.9
goal.orientation = 0

goal.parent = None

high_level_plan(start, goal)

#Robot params
w0=2.5
wp=5
fw=1.6
print(ik_finger(4, 5, w0, wp, fw))


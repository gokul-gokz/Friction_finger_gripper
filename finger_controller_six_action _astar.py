import time
import math

import Queue

import matplotlib.pyplot as plt

#from mpmath import *
from sympy import *
import numpy as np

import scipy.optimize as opt
from numpy import exp
import timeit

FINGER_END = 15
FINGER_START = 0
SLIDING_RESOLUTION = 0.1
ROTATION_SLIDE_RESOLUTION=0.2
PALM_WIDTH = 5
OBJECT_SIZE= 2.5
TH1_MAX= 2.485 #142.5 degrees
TH2_MAX= 0.65 #37.5
FINGER_WIDTH=1
K=0.1
NUMBER_OF_NODES_EXPANDED=0

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

def action_right_equations(variables) :
    (th1,th2) = variables
    eqn1 = FINGER_WIDTH*sin(th1)+FINGER_WIDTH*sin(th2)+left_position * cos(th1) + OBJECT_SIZE * sin(th1) - PALM_WIDTH - right_position * cos(th2)
    eqn2 =-FINGER_WIDTH*cos(th1)-FINGER_WIDTH*cos(th2)+left_position * sin(th1) - OBJECT_SIZE * cos(th1) - right_position * sin(th2)
    return [eqn1, eqn2]

def action_left_equations(variables) :
    (th1, th2) = variables
    eqn1 = FINGER_WIDTH * sin(th1) + FINGER_WIDTH * sin(th2) + left_position * cos(th1) + OBJECT_SIZE * sin(th2) - PALM_WIDTH - right_position * cos(th2)
    eqn2 = -FINGER_WIDTH * cos(th1) - FINGER_WIDTH * cos(th2) + left_position * sin(th1) - OBJECT_SIZE * cos(th2) - right_position * sin(th2)
    return [eqn1, eqn2]

def theta_conversion(left_position, right_position, action_name):

    if (action_name == "r_plus" or action_name == "r_minus"):
        solution = opt.fsolve(action_right_equations, (0.1, 1))

        # print "right"
        # print "left",left_position,"right",right_position
        # #print solution
        return solution
    elif (action_name == "l_plus" or action_name == "l_minus"):
        solution = opt.fsolve(action_left_equations, (0.1, 1))

        # print "left"
        # print "left", left_position, "right", right_position
        #print solution
        return solution




left_position=0
right_position=0
def limit_check(left_pos,right_pos,action):
    global left_position
    global right_position
    left_position=left_pos
    right_position=right_pos

    if(left_position<FINGER_END and left_position>FINGER_START and right_position<FINGER_END and right_position>FINGER_START):
        sol=theta_conversion(left_position, right_position, action)

        th1=sol[0]
        #print th1
        th2=sol[1]
        #print th2

        if(th1<=TH1_MAX and th2>=TH2_MAX):
            return True
        else:
            print "range_limit_exceeding"
            return False
    else:
         return False


class node:


    def __init__(self,res,pose_l,pose_r,pose_o,p,a):
        self.orientation = pose_o
        self.parent = p
        self.action = a
        self.g = 0
        self.h = 0
        self.f = 0

        if (a=="l_plus"):
            self.position_l = pose_l + res
            self.position_r=pose_r
            (self.th1,self.th2)= theta_conversion(self.position_l, self.position_r, a)
        elif(a=="l_minus"):
            self.position_l = pose_l - res
            self.position_r = pose_r
            (self.th1, self.th2) = theta_conversion(self.position_l, self.position_r, a)
        elif(a=="r_plus"):
            self.position_r = pose_r + res
            self.position_l = pose_l
            (self.th1, self.th2) = theta_conversion(self.position_l, self.position_r, a)
        elif(a=="r_minus"):
            self.position_r = pose_r - res
            self.position_l = pose_l
            (self.th1, self.th2) = theta_conversion(self.position_l, self.position_r, a)
        else:
            self.position_r = pose_r
            self.position_l = pose_l

    def update(self,g,goal_l,goal_r):
        self.g = g + 10;

        if (isclose(self.position_l, goal_l, rel_tol=1e-09, abs_tol=0.0)):
            if self.action=="l_plus":
                # print "h1"
                 self.h=-10000
            if self.action=="l_minus":
               #  print "h2"
                 self.h=-10000
        elif (isclose(self.position_r, goal_r, rel_tol=1e-09, abs_tol=0.0)):
            if self.action=="r_plus":
               #  print "h3"
                 self.h=-10000
            if self.action=="r_minus":
                # print "h4"
                 self.h=-10000
        if(1):
            if self.action=="l_plus" and (not(isclose(self.position_l, goal_l, rel_tol=1e-09, abs_tol=0.0))) :
                 #print "h5"
                 self.h=(1/(self.position_l - goal_l))
            if self.action=="l_minus" and not(isclose(self.position_l, goal_l, rel_tol=1e-09, abs_tol=0.0)):
                 #print "h6"
                 self.h=(1/(goal_l-self.position_l))
            if self.action=="r_plus" and (not(isclose(self.position_r, goal_r, rel_tol=1e-09, abs_tol=0.0))):
                 self.h=(1/(self.position_r - goal_r))
                 #print "h7"
                 #self.h = 100
            if self.action=="r_minus" and (not(isclose(self.position_r, goal_r, rel_tol=1e-09, abs_tol=0.0))):
                 self.h=(1/(goal_r-self.position_r))
                 #print "h8"
                 #self.h=0

        self.f = self.g+self.h
        print self.action,".h=",self.h
        print "actual_cost=",self.f



    def find_neighbours(self, COST_T, COST_R,goal_l,goal_r):
        neighbours = []
        actions=("l_plus","l_minus","r_plus","r_minus")

        if (limit_check(self.position_l + SLIDING_RESOLUTION,self.position_r ,actions[0])):
            action="l_plus"
            l_plus=node(SLIDING_RESOLUTION,self.position_l,self.position_r,self.orientation,self,action)
            neighbours.append(l_plus)


        if (limit_check(self.position_l - SLIDING_RESOLUTION,self.position_r,actions[1])):


            action="l_minus"
            l_minus=node(SLIDING_RESOLUTION,self.position_l,self.position_r,self.orientation,self,action)

            neighbours.append(l_minus)

        if (limit_check(self.position_l,self.position_r - SLIDING_RESOLUTION,actions[3])):


            action = "r_minus"
            r_minus=node(SLIDING_RESOLUTION,self.position_l,self.position_r,self.orientation,self,action)

            neighbours.append(r_minus)

        if (limit_check(self.position_l,self.position_r + SLIDING_RESOLUTION,actions[2])):

            action = "r_plus"
            r_plus=node(SLIDING_RESOLUTION,self.position_l,self.position_r,self.orientation,self,action)
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


def A_star(start, goal):
    global NUMBER_OF_NODES_EXPANDED
    print "Start and Goal state validation"
    if limit_check(start.position_l, start.position_r, "r_plus") and  limit_check(start.position_l, start.position_r, "l_plus"):
        print"start_valid"
    else:
        print"Invalid startstate"
        return None
    if limit_check(goal.position_l, goal.position_r, "r_plus") and  limit_check(goal.position_l, goal.position_r, "l_plus"):
        print"Goal_valid"
    else:
        print"Invalid Goalstate"
        return None


    start_time = time.time()
    #queue = [start]
    #cur = queue.pop()
    open_list=Queue.PriorityQueue()
    open_list.put((start.f,start))
    closed_list=[]
    expanded=0
    while(1):

        cur = (open_list.get())[1]
        for exp_nodes in closed_list:
            if isclose(exp_nodes[0],cur.position_l, rel_tol=1e-09, abs_tol=0.0) and isclose(exp_nodes[1],cur.position_r, rel_tol=1e-09, abs_tol=0.0):
                print "already in closed list"
                expanded=1
                continue

        if(expanded):
            expanded=0
            continue

        print "action=",cur.action
        print "l=",(cur.position_l)
        print "r=",(cur.position_r)
        print "total_cost=",cur.f
        exp= [cur.position_l,cur.position_r]
        closed_list.append(exp)
        NUMBER_OF_NODES_EXPANDED=NUMBER_OF_NODES_EXPANDED+1

        if ( (isclose(cur.position_l, goal.position_l, rel_tol=1e-09, abs_tol=0.0) and isclose(cur.position_r,
                                                                                                     goal.position_r,
                                                                                                     rel_tol=1e-09,
                                                                                                        abs_tol=0.0) and (isclose(cur.orientation, goal.orientation, rel_tol=1e-09, abs_tol=0.0)))):
                end_time = time.time()

                # print (cur.position_l)
                # print (cur.position_r)
                # print (goal.position_l)
                # print (goal.position_r)
                print "done"
                print "Time_taken", end_time - start_time
                return backtrace(cur)
                break


        neighbours=cur.find_neighbours(1,0,goal.position_l,goal.position_r)
        for nod in neighbours:
            flag=True
            for exp_nodes in closed_list:
                if(exp_nodes[0] == nod.position_l) and (exp_nodes[1] == nod.position_r):
                    flag=False

            if(flag):
                nod.update(cur.f,goal.position_l,goal.position_r)
                open_list.put((nod.f,nod))

        #cur = queue.pop(0)

        #res = backtrace(cur)





def backtrace(cur):
    path = []
    while not cur.parent == None:
        path.append(cur.action)
        print "node=", cur.action, "cost=", cur.f,"left=",cur.position_l,"right=",cur.position_r
        cur = cur.parent
    print"Number of nodes expanded=",NUMBER_OF_NODES_EXPANDED
    print"Length of solution=",len(path)
    print path
    return path


def high_level_plan(start, goal):
    l = A_star(start, goal)

start = node(0,5.0,5.0,0,None,None)

goal = node(0,5.0,4.0,0,None,None)
high_level_plan(start, goal)

#Robot params
w0=2.5
wp=5
fw=1.6
#print(ik_finger(4, 5, w0, wp, fw))


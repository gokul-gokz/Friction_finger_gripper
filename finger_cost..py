import time
import math

FINGER_END = 15
FINGER_START = 0
SLIDING_RESOLUTION = 0.1
ROTATION_SLIDE_RESOLUTION=0.2


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

class node:
    def __init__(self):
        self.position = 0
        self.orientation = 0
        self.parent = None
        self.action = None

    def find_neighbours(self, COST_T, COST_R):
        neighbours = []
        o_plus = node()
        o_minus = node()

        if (self.position + SLIDING_RESOLUTION < FINGER_END and self.position >= FINGER_START and self.parent != "p_minus") and COST_T>0:
            p_plus = node()
            p_plus.position = self.position + SLIDING_RESOLUTION
            p_plus.orientation = self.orientation
            p_plus.action = "p_plus"
            p_plus.parent = self
            print self.position
            print ""
            neighbours.append(p_plus)

        if (self.position <= FINGER_END and self.position - SLIDING_RESOLUTION > FINGER_START and self.parent != "p_plus") and COST_T<0:
            p_minus = node()
            p_minus.position = self.position - SLIDING_RESOLUTION
            p_minus.orientation = self.orientation
            p_minus.action = "p_minus"
            p_minus.parent = self
            neighbours.append(p_minus)

        if (self.position + ROTATION_SLIDE_RESOLUTION <= FINGER_END and self.position >= FINGER_START and self.parent != "o_minus" and COST_R>0):
            o_plus.position = self.position + ROTATION_SLIDE_RESOLUTION
            o_plus.orientation = self.orientation + 90
            o_plus.action = "o_plus"
            o_plus.parent = self
            neighbours.append(o_plus)

        if (self.position + ROTATION_SLIDE_RESOLUTION <= FINGER_END and self.position >= FINGER_START and self.parent != "o_plus" and COST_R<0):
            o_minus.position = self.position+ROTATION_SLIDE_RESOLUTION
            o_minus.orientation = self.orientation - 90
            o_minus.action = "o_minus"
            o_minus.parent = self
            neighbours.append(o_minus)

        return neighbours


def bfs(start, goal):
    start_time = time.time()
    queue = [start]
    cur = queue.pop()

    while (not(isclose(cur.position, goal.position, rel_tol=1e-09, abs_tol=0.0) and (isclose(cur.orientation, goal.orientation, rel_tol=1e-09, abs_tol=0.0)))) :
        #if((cur.position == goal.position) and (cur.orientation == goal.orientation)):
            #print ("yes")
            #break
        COST_T,COST_R=cost_function(cur.position,goal.position,cur.orientation,goal.orientation)
        queue = queue + cur.find_neighbours(COST_T,COST_R)
        cur = queue.pop(0)
        print (cur.position)
        print (goal.position)
        print (cur.orientation)
        print len(queue)
        print ""
        print(isclose(cur.position, goal.position, rel_tol=1e-09, abs_tol=0.0) and (isclose(cur.orientation, goal.orientation, rel_tol=1e-09, abs_tol=0.0)))

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
start.position = 5.0
start.orientation = 0
start.parent = None

goal.position = 5.0
goal.orientation = -90
0
goal.parent = None

high_level_plan(start, goal)

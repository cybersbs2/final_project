import astar_path
import numpy as np

def compath(start, goal, draw) :
        (path, mymap) = astar_path.astar(start,goal,draw)
        new_path = []
        comp_path = []

        for i in path :
                start = i[0]
                goal = i[1]
                goal = round(goal*0.04,4)
                start = round(start*0.04,4)
                new_path.append((start,goal))

        for i in range(len(new_path)-2) :
                if (new_path[i][1]-new_path[i+1][1]) != 0 :
                        left_r = (new_path[i][0]-new_path[i+1][0])/(new_path[i][1]-new_path[i+1][1])
                else :
                        left_r = np.inf

                if (new_path[i+1][1]-new_path[i+2][1]) != 0 :
                        right_r = (new_path[i+1][0]-new_path[i+2][0])/(new_path[i+1][1]-new_path[i+2][1])
                else :
                        right_r = np.inf    

                if round(left_r,4) != round(right_r,4) :
                        start = new_path[i+1][0]
                        goal = new_path[i+1][1]
                        comp_path.append((start,goal))

        s = path[-1][0]
        g = path [-1][1]
        s = round(s*0.04,4)
        g = round(g*0.04,4)
        comp_path.append((s,g))
        
        return comp_path
"""
a = 0.12-0.16
b = 0.36-0.4
c = 0.2-0.16
d = 0.44-0.4
e = round(a*d,4)
f = round(b*c,4)
print e-f
if e != f:
        print e,
        print "=",
        print f,
        print "--->",
        print e==f
"""

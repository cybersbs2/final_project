import numpy as np
import cv2

def db(x,y) :
    g = np.abs(x[0]-y[0])
    s = np.abs(x[1]-y[1])
    if g >= s :
        return np.abs(s*14+(g-s)*10)
    else :
        return np.abs(g*14+(g-s)*10)

def md(x, y) :
    length = np.abs(x[0]-y[0])+np.abs(x[1]-y[1])
    return length*10

def astar(start, goal, draw) :
    #file_addr = '/home/kimtaehun/mapLounge3.jpg'
    file_addr = '/home/kimtaehun/mapLounge5.jpg'
    totalmap = cv2.imread(file_addr, cv2.IMREAD_GRAYSCALE)

    w = totalmap.shape[0]
    l = totalmap.shape[1]
    
    #start = (99, 1)
    #goal = (10, 89)
    #start = (29, 1)
    #goal = (1, 29)

   

    ######################    
    openSet = []
    closeSet = []
    
    for i in range(0,w):
        for j in range(0,l):
            if totalmap[i,j] > 125 :
                totalmap[i,j] = 0

            else :
		totalmap[i,j] = 1
                closeSet.append((i,j))


    canGo = []
    
    for i in range(0,w) :
        for j in range(0,l) :
            if totalmap[i][j] == 0 :
                canGo.append((i,j))

    min_d = 100000
    
    if totalmap[start[0]][start[1]] == 1:
        for i in canGo:
            if md(start, i) < min_d:
                min_d = md(start, i)
                new_start = i
    else :
        new_start = start

    min_d = 100000
    
    if totalmap[goal[0]][goal[1]] == 1:
        for i in canGo :
            if md(goal, i) < min_d :
                min_d = md(goal,i)
                new_goal = i
    else :
        new_goal = goal

    goal = new_goal
    start = new_start
    
    openSet.append(start)
    fGoal = goal  
    pathSet = {}

    gScore = np.ones((w,l))
    gScore = gScore*np.inf

    fScore = np.ones((w,l))
    fScore = fScore*np.inf

    gScore[start] = 0
    fScore[start] = md(start, goal)

    while len(openSet) != 0 :
        min_val = 10000
        for i in openSet :
            if fScore[i] < min_val :
                min_val = fScore[i]
                current = i

        if current == goal :
            break
        else :
            openSet.remove(current)
            closeSet.append(current)

        neighber = []
        
        for i in range(-1, 2) :
            for j in range(-1, 2) :
                if current[0]+i<0  or current[1]+j<0 or current[0]+i>np.shape(totalmap)[0]-1 or current[1]+j>np.shape(totalmap)[1]-1 :
                    continue
                neighber.append((current[0]+i,current[1]+j))
        neighber.remove(current)
                
        for i in neighber:
            if i in closeSet :
                continue
            if i not in openSet :
                openSet.append(i)

            tentative_gScore = gScore[current] + db(current, i)
            if tentative_gScore >= gScore[i] :
                continue
            else :
                pathSet[i] = current
                gScore[i] = tentative_gScore
                fScore[i] = gScore[i] + md(i,goal)

    total_path = [goal]
    while goal in pathSet.keys() :
        goal = pathSet[goal]
        total_path.append(goal)

    total_path.reverse()
    #print total_path

    #total_path.remove(start)
    #total_path.remove(fGoal)

    if draw == 1 :

        for i in range(0,w):
            for j in range(0,l):

                if start[0] == i and start[1] == j :
                    print 's',
                    continue
                if fGoal[0] ==i and fGoal[1] == j :
                    print 'g',
                    continue

                if (i,j) in total_path :
                    print '0',
                else :
                    if totalmap[i,j] == 0 :
                        print ' ',
                    else :
                        print '.',

            print ''

    return (total_path, totalmap)

import numpy as np
import cv2

kernel = np.array([
            [0,1,0],
            [1,0,1],
            [0,1,0]
            ])/4.0

def mkBVPMap(worldmap, steps=100, walls=None):
    if walls is None:
        walls = np.zeros(worldmap.shape)
    for x in range(steps):
        walls[worldmap>0]=1
        walls[worldmap==0]=-1
        walls = cv2.filter2D(walls,-1,kernel)
    return walls

def mkRoute(planMap, start, steps=20, stepSize=5):
    route = []
    cur = start
    for s in range(steps):
        local = planMap[cur[0]-stepSize:cur[0]+stepSize,cur[1]-stepSize:cur[1]+stepSize]
        pos = np.unravel_index(np.argmin(local),local.shape)
        pos = (pos[0]-stepSize+cur[0], pos[1]-stepSize+cur[1])
        route.append(pos)
        # local minimum
        if cur == pos:
            break
        cur = pos
    return route

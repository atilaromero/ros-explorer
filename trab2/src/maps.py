import numpy as np
import cv2

def joinMaps(worldmap, localmap, x, y, rot):
    rows,cols = localmap.shape
    M = cv2.getRotationMatrix2D((cols/2,rows/2),rot*180/np.pi,1)
    res = np.zeros((max([worldmap.shape[0],rows/2+x]),max(worldmap.shape[1],cols/2+y)))
    res[:worldmap.shape[0],:worldmap.shape[1]] = worldmap.copy()
    res[x-rows/2:,y-cols/2:][:rows,:cols] += cv2.warpAffine(localmap,M,(rows,cols))
    res = np.sin(res*1.1)/np.sin(1.2)/1.1
    if np.max(np.abs(res.flat))>1:
        print "error in joinMaps: ", np.max(np.abs(res.flat))
    return res

def trypos(worldmap, localmap, x, y, rot):
    rows,cols = localmap.shape
    M = cv2.getRotationMatrix2D((rows/2,cols/2),rot*180/np.pi,1)
    map4 = worldmap.copy()
    map4 = map4[x-rows/2:,y-cols/2:][:rows,:cols]
    map4[map4<0]=0 # mapa apenas com obstaculos
    map4*=cv2.warpAffine(localmap,M,(cols,rows))
    return sum(map4.flat)

def localrandom(worldmap, laserscan, x, y, rot, smallmax=5, bigmax=15):
    best = (x, y, rot)
    bestresult = trypos(worldmap, laserscan, *best)
    small = 0
    big = 0
    while(big<bigmax and small<smallmax):
        small += 1
        big += 1
        x = np.random.normal()
        y = np.random.normal()
        alpha = np.random.normal()/150
        sample = (int(x+best[0]), int(y+best[1]), alpha + best[2])
        sampleresult = trypos(worldmap, laserscan, *sample)
        if sampleresult > bestresult:
            small = 0
            best = sample
            bestresult = sampleresult
    return best, bestresult

def localbest(worldmap, laserscan, x, y, rot, maxattempts=10, curresult=None, angle_increment=0.00163668883033):
    if curresult is None:
        curresult = trypos(worldmap, laserscan, x, y, rot)
    if maxattempts > 0:
        for dx,dy,drot in [( 1, 0,0),
                           (-1, 0,0),
                           ( 0, 1,0),
                           ( 0,-1,0),
                           ( 0, 0, angle_increment),
                           ( 0, 0,-angle_increment)
                          ]:
            maxattempts -= 1
            sample = (x+dx, y+dy, rot+drot)
            sampleresult = trypos(worldmap, laserscan, *sample)
            if sampleresult > curresult:
                return localbest(worldmap, laserscan, *sample, maxattempts=maxattempts, curresult=sampleresult)
    return (x,y,rot), curresult


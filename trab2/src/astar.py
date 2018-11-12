import collections
import numpy as np

def mkRoute(worldmap, start, goal):
    def h(start, goal, borders=8):
        x,y = goal
        walls = worldmap.copy()
        walls[walls<0]=0
        if np.sum(walls[x-borders:x+borders, y-borders:y+borders]) >0:
            return np.inf
        dist = np.sqrt(np.sum((np.array(start)-np.array(goal))**2))
        return dist
    def stopfun(cur, borders=3):
        # stop when reached unknow area
        x,y = cur
        return np.sum(worldmap[x-borders:x+borders,y-borders:y+borders]**2)==0
    return a_star(start, goal, h, neighbors, stopfun=stopfun)

def neighbors(pos, dist=4):
    pos = tuple(pos)
    r = set()
    for x in range(-1,2):
        for y in range(-1,2):
            r.add((pos[0]+x*dist,pos[1]+y*dist))
    return [(x,y) for (x,y) in r if (x,y)!=pos and x>=0 and y>=0]

def traceback(cur, came):
    p = [cur]
    while cur in came:
        cur = came[cur]
        p.append(cur)
    if len(p)==1:
        return []
    return p[::-1]

def a_star(start, goal, h, neighbors, stopfun=lambda x: False, maxruns=4000):
    todo = set()
    todo.add(start)
    done = set()
    came = {}
    g = collections.defaultdict(lambda:np.inf)
    g[start] = 0
    f = collections.defaultdict(lambda:np.inf)
    f[start] = h(start, goal)
    while(len(todo)>0 and maxruns>0):
        maxruns-=1
        cur = min(todo, key=lambda x: f[x])
        if cur == goal or stopfun(cur):
            return traceback(cur, came)
        todo.remove(cur)
        done.add(cur)
        for neighbor in neighbors(cur):
            if neighbor in done:
                continue
            if not neighbor in todo:
                todo.add(neighbor)
            tempg = g[cur] + h(cur, neighbor)
            if tempg >= g[neighbor]:
                continue
            g[neighbor] = tempg
            came[neighbor] = cur
            f[neighbor] = g[neighbor] + h(neighbor,goal)
    return traceback(cur,came)

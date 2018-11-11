#!/usr/bin/env python

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import cv2
import threading
import time
from matplotlib.colors import Normalize
import matplotlib.cm as cm
import collections

def normRad(rad):
    return (rad + np.pi) % (2*np.pi) - np.pi

def ranges2cart(ranges, range_min, range_max, angle_min, angle_increment):
    imgsize = 100
    rangestep = range_max/imgsize
    ranges = np.array(ranges)
    r, phi = np.mgrid[0:range_max:rangestep,-np.pi:np.pi:angle_increment]
    phimin_index = int((angle_min+np.pi)/angle_increment)
    ranges2d = np.zeros(r.shape)
    ranges2d[:,phimin_index:phimin_index+ranges.shape[0]]=ranges
    ranges2d[np.isnan(ranges2d)] = range_max*2
    v = np.zeros(r.shape)
    v[r>ranges2d] = 0
    v[r<ranges2d] = -1
    v[r<range_min] = 0
    v[(ranges2d!=0) & (ranges2d!=range_max) & (np.abs(r-ranges2d)<0.1)] =1
    v *= np.cos(r/range_max)
    # v *= np.cos(2*phi)
    dst = cv2.linearPolar(v.T, (imgsize,imgsize),imgsize,cv2.WARP_FILL_OUTLIERS | cv2.WARP_INVERSE_MAP)
    dst2 = np.zeros((imgsize*2,imgsize*2))
    dst2[:,imgsize:] = dst[:imgsize*2,:][:,::-1] 
    return dst2

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

def updateWorldMapThread(obj):
    norm = Normalize(vmin=-2, vmax=1)
    while(not rospy.is_shutdown()):
        if not hasattr(obj, "scan"):
            time.sleep(0.1)
            continue
        msg = obj.scan
        x, y, rot = obj.pos.x, obj.pos.y, obj.pos.rot
        x0,y0,rot0 = x,y,rot
        localmap = ranges2cart(msg.ranges, msg.range_min, msg.range_max, msg.angle_min, msg.angle_increment)
        grade = 0
        # (x, y, rot), grade = localrandom(obj.worldmap, localmap, x, y, rot)
        # (x, y, rot), grade = localbest(obj.worldmap, localmap, x, y, rot, angle_increment=msg.angle_increment)
        if grade > 0:
            obj.pos.x += (x-x0)/2  # /2 to smooth correction      
            obj.pos.y += (y-y0)/2       
            obj.pos.rot += (rot-rot0)/2
        obj.worldmap = joinMaps(obj.worldmap, localmap, x, y, rot)
        obj.localmap = localmap

def showImagesThread(obj):
    norm = Normalize(vmin=-2, vmax=1)
    cv2.namedWindow('worldmap')
    cv2.namedWindow('localmap')
    while(not rospy.is_shutdown()):
        worldmap = obj.worldmap.copy()
        for x,y in obj.route:
            worldmap[x-4:x+4,y-4:y+4] = -2
        cv2.imshow("worldmap", cm.rainbow(norm(worldmap)))
        cv2.imshow("localmap", cm.rainbow(norm(obj.localmap)))
        cv2.waitKey(1)
    cv2.destroyWindow('worldmap')
    cv2.destroyWindow('localmap')

def calcPathPlanThread(obj):
    while(not rospy.is_shutdown()):
        print "making path"
        if len(obj.route)>0: # do not recalculate if there is a plan in action
            time.sleep(5)
            obj.route = obj.route[1:] # fade path
            continue
        route = mkPathPlanA(obj.worldmap, (obj.pos.x, obj.pos.y), (200,380))
        obj.route = route

def mkPathPlanA(worldmap, start, goal):
    time.sleep(10) # wait initial spin
    def h(start, goal):
        x,y = goal
        walls = worldmap.copy()
        walls[walls<0]=0
        r = 5
        if np.sum(walls[x-r:x+r, y-r:y+r]) >0:
            return np.inf
        dist = np.sqrt(np.sum((np.array(start)-np.array(goal))**2))
        return dist
    def stopfun(cur):
        x,y = cur
        return np.sum(worldmap[x-3:x+3,y-3:y+3]**2)==0
    return a_star(start, goal, h, neighbors, stopfun=stopfun)

def neighbors(pos):
    r = set()
    for x in range(-1,2):
        for y in range(-1,2):
            r.add((pos[0]+x*8,pos[1]+y*8))
    return [(x,y) for (x,y) in r if (x,y)!=pos and x>=0 and y>=0]

def traceback(cur, came):
    p = [cur]
    while cur in came:
        cur = came[cur]
        p.append(cur)
    return p

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

class Trab2():
    def __init__(self):
        self.linearResolution = 0.2
        self.worldmap = np.ndarray((400,400), float)
        self.route = []
        self.localmap = np.ndarray((10,10),float)
        self.pos = lambda:None
        self.pos.x = self.worldmap.shape[0]/2
        self.pos.y = self.worldmap.shape[1]/2
        self.pos.rot = 0
        rospy.init_node('Trab2', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.getScan)
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

        threading.Thread(target=lambda:updateWorldMapThread(self)).start()
        threading.Thread(target=lambda:calcPathPlanThread(self)).start()
        threading.Thread(target=lambda:showImagesThread(self)).start()

        # TurtleBot will stop if we don't keep telling it to move.
        # How often should we tell it to move? 10 HZ
        self.rate = 10
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # wait for 0.1 seconds (10 HZ) and publish again
            self.move()
            r.sleep()

    def getScan(self, msg):
        self.scan = msg

    def move(self):
        spin = self.calcSpin()
        vel = self.calcVel(spin)
        self.pos.rot += spin/self.rate
        self.pos.x -= np.sin(self.pos.rot)*vel*2
        self.pos.y += np.cos(self.pos.rot)*vel*2
        move_cmd = Twist()
        move_cmd.angular.z = spin
        move_cmd.linear.x = vel
        self.cmd_vel.publish(move_cmd)

    def calcSpin(self):
        if len(self.route)==0:
            return 0.5
        # while(len(self.route)>0):
        #         continue
        #     break
        rx,ry = self.route[-1]
        dx, dy = rx-self.pos.x, ry-self.pos.y
        if np.abs(dx)+np.abs(dy)<8: # reached point
            self.route.pop()
        rot = np.arctan2(-dx,dy) # cv2 weird axis
        rot = (rot+np.pi)%(2*np.pi)-np.pi
        spin = rot-self.pos.rot
        spin = (spin+np.pi)%(2*np.pi)-np.pi
        # spinmax = 1.0
        # if abs(spin) > spinmax:
        #     return spin/abs(spin)*spinmax
        return spin * 0.3

    def calcVel(self, spin):
        if not hasattr(self, "scan"):
            return 0
        # ranges = self.scan.ranges
        # if np.nanmin(ranges) < 0.5: #colision detected
        #     return 0
        if abs(spin) < 0.1:
            return (1 - abs(spin))*0.3
        return 0

    def shutdown(self):
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        Trab2(*sys.argv[1:])
    except rospy.exceptions.ROSInterruptException:
        pass

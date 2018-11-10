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

def showImage(obj):
    norm = Normalize(vmin=-1, vmax=1)
    cv2.namedWindow('worldmap')
    cv2.namedWindow('localmap')
    worldmap = obj.worldmap
    while(not rospy.is_shutdown()):
        if not hasattr(obj, "scan"):
            time.sleep(0.1)
            continue
        msg = obj.scan
        x, y, rot = obj.pos.x, obj.pos.y, obj.pos.rot
        x0,y0,rot0 = x,y,rot
        localmap = ranges2cart(msg.ranges, msg.range_min, msg.range_max, msg.angle_min, msg.angle_increment)
        # (x, y, rot), grade = localrandom(worldmap, localmap, x, y, rot)
        (x, y, rot), grade = localbest(worldmap, localmap, x, y, rot, angle_increment=msg.angle_increment)
        if grade > 0:
            obj.pos.x += (x-x0)/2        
            obj.pos.y += (y-y0)/2       
            obj.pos.rot += (rot-rot0)/2
            if abs(rot0-rot)>0:
                print rot0-rot  
        worldmap = joinMaps(worldmap, localmap, x, y, rot)
        cv2.imshow("worldmap", cm.rainbow(norm(worldmap)))
        cv2.waitKey(1)
        cv2.imshow("localmap", cm.rainbow(norm(localmap)))
        cv2.waitKey(1)
    cv2.destroyWindow('worldmap')
    cv2.destroyWindow('localmap')
        

class Trab2():
    def __init__(self):
        self.linearResolution = 0.2
        self.worldmap = np.ndarray((400,400), float)
        self.pos = lambda:None
        self.pos.x = self.worldmap.shape[0]/2
        self.pos.y = self.worldmap.shape[1]/2
        self.pos.rot = 0
        rospy.init_node('Trab2', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.getScan)
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

        self.threads = []
        th = threading.Thread(target=lambda:showImage(self))
        th.start()
        self.threads.append(th)

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
        self.pos.x += (np.cos(self.pos.rot)*vel)/self.rate
        self.pos.y += (np.sin(self.pos.rot)*vel)/self.rate
        move_cmd = Twist()
        move_cmd.angular.z = spin
        move_cmd.linear.x = vel
        self.cmd_vel.publish(move_cmd)

    def calcVel(self, spin):
        if abs(spin) < 1.0:
            return (1 - abs(spin))*0.7
        return 0.1

    def calcSpin(self):
        return 1.0

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

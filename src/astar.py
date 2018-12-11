#!/usr/bin/env python

import sys
import collections
import numpy as np
import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import matplotlib.image
from matplotlib.colors import Normalize
import matplotlib.cm
import threading
import cv2
import laserscan
import particle
import time
import traceback

def mkRoute(worldmap, start, goal):
    def h(start, goal, borders=8):
        x,y = goal
        walls = worldmap.copy()
        walls[walls<0]=0
        if np.sum(walls[x-borders:x+borders, y-borders:y+borders]) >0:
            return np.inf
        dist = np.sqrt(np.sum((np.array(start)-np.array(goal))**2))
        return dist
    return a_star(start, goal, h, neighbors)

def neighbors(pos, dist=4):
    pos = tuple(pos)
    r = set()
    for x in range(-1,2):
        for y in range(-1,2):
            r.add((pos[0]+x*dist,pos[1]+y*dist))
    return [(x,y) for (x,y) in r if (x,y)!=pos and x>=0 and y>=0]

def tracepath(cur, came):
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
            return tracepath(cur, came)
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
    return tracepath(cur,came)

def showImagesThread(obj):
    norm = Normalize(vmin=-2, vmax=1)
    pix = 1
    while(not rospy.is_shutdown()):
        try:
            worldmap = obj.worldmap.copy()
            for x,y in obj.route:
                worldmap[x-pix:x+pix,y-pix:y+pix] = -2
            if hasattr(obj, 'cloud'):
                for x,y,rot in obj.cloud:
                    worldmap[x-pix:x+pix,y-pix:y+pix] = -4
            cv2.imshow("worldmap", matplotlib.cm.rainbow(norm(worldmap)))
            cv2.waitKey(1)
        except Exception, e:
            traceback.print_exc()
    cv2.destroyWindow('worldmap')

def localizationThread(obj):
    mean = 0.0
    std = 1.0
    while(not rospy.is_shutdown()):
        try:
            if not (hasattr(obj, 'scan') and hasattr(obj, 'pos') and hasattr(obj, 'worldmap')):
                time.sleep(0.1)
                continue
            def scorefunc(cloud):
                return np.array([laserscan.score((x, y), rot, obj.worldmap, 1.0, (0,0), obj.scan)
                    for x,y,rot in cloud])
            if not hasattr(obj, 'cloud'):
                obj.cloud = particle.uniform_particles(1000, (0,0,0), (obj.worldmap.shape[0], obj.worldmap.shape[1], 2*np.pi))
                obj.cloud_odom = (obj.pos.x, obj.pos.y, obj.pos.rot)
            else:
                mean, std = particle.estimate(obj.cloud)
                weights = particle.updateWeights(scorefunc, obj.cloud, mean, 10.0)
                obj.cloud = particle.simple_resample(obj.cloud, weights)
            #     weights = particle.updateWeights(lambda x:1, obj.cloud, mean, std)
            #     particle.simple_resample(obj.cloud, weights)
            #     obj.cloud = particle.predict(obj.cloud, 
            #         (obj.pos.x-obj.cloud_odom[0], 
            #          obj.pos.y-obj.cloud_odom[1],
            #          obj.pos.rot-obj.cloud_odom[2]), 
            #         std)
            #     obj.cloud_odom = (obj.pos.x, obj.pos.y, obj.pos.rot)
            #     mean, std = particle.estimate(obj.cloud)
            #     obj.guesspos=lambda:None
            #     obj.guesspos.x = mean[0]
            #     obj.guesspos.y = mean[1]
            #     obj.guesspos.rot = mean[2]
        except Exception, e:
            traceback.print_exc()

class AStar():
    def __init__(self, goalX, goalY):
        self.goalX = int(goalX)
        self.goalY = int(goalY)
        rospy.init_node('AStar', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber('/odom', Odometry, self.getOdom)
        rospy.Subscriber('/map', OccupancyGrid, self.getMap)
        rospy.Subscriber('/scan', LaserScan, self.getScan)
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

        threading.Thread(target=lambda:localizationThread(self)).start()

        # TurtleBot will stop if we don't keep telling it to move.
        # How often should we tell it to move? 10 HZ
        self.rate = 10
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown() and (not hasattr(self, 'route') or len(self.route)>0):
            # wait for 0.1 seconds (10 HZ) and publish again
            self.move()
            r.sleep()

    def getOdom(self, msg):
        if not hasattr(self, 'pos'):
            self.pos = lambda:None
        self.pos.x = msg.pose.pose.position.x/self.map.info.resolution
        self.pos.y = msg.pose.pose.position.y/self.map.info.resolution
        quat = msg.pose.pose.orientation
        self.pos.rot = tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))[2]
    def getMap(self, msg):
        self.map = msg
        self.worldmap = np.array(msg.data).reshape((msg.info.height,msg.info.width))
    def getScan(self, msg):
        self.scan = msg

    def move(self):
        if not (hasattr(self, 'pos') and hasattr(self, 'worldmap')):
            return
        if not hasattr(self, 'route'):
            self.route = mkRoute(self.worldmap, 
                (self.pos.x, self.pos.y),
                (self.goalX, self.goalY))
            threading.Thread(target=lambda:showImagesThread(self)).start()
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
            return 0
        rx,ry = self.route[0]
        dx, dy = rx-self.pos.x, ry-self.pos.y
        if np.abs(dx)+np.abs(dy)<2: # reached point
            self.route = self.route[1:]
            return self.calcSpin()
        rot = np.arctan2(-dx,dy) # cv2 weird axis
        rot = (rot+np.pi)%(2*np.pi)-np.pi
        spin = rot-self.pos.rot
        spin = (spin+np.pi)%(2*np.pi)-np.pi
        return spin * 0.9

    def calcVel(self, spin):
        return 0
        if len(self.route)==0:
            return 0
        # wait for goal to be ahead
        if abs(spin) < 0.1:
            return (1 - abs(spin))*0.8
        return 0

    def shutdown(self):
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        AStar(*sys.argv[1:])
    except rospy.exceptions.ROSInterruptException:
        pass

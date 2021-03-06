#!/usr/bin/env python

import sys
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import cv2
import threading
import time
import matplotlib.image
from matplotlib.colors import Normalize
import matplotlib.cm
import datetime
import harmonicpotentialfield
import laserscan
import maps
import traceback

def normRad(rad):
    return (rad + np.pi) % (2*np.pi) - np.pi

def updateWorldMapThread(obj):
    while(not rospy.is_shutdown()):
        if not hasattr(obj, "scan"):
            time.sleep(0.1)
            continue
        try:
            msg = obj.scan
            x, y, rot = obj.pos.x, obj.pos.y, obj.pos.rot
            localmap = laserscan.ranges2cart(msg.ranges, msg.range_min, msg.range_max, msg.angle_min, msg.angle_increment)
            obj.worldmap = maps.joinMaps(obj.worldmap, localmap, x, y, rot).copy()
            obj.localmap = localmap
            maps.publishMap(obj.worldmap, "mymap", obj.cmd_map, obj.pos.x, obj.pos.y, obj.pos.rot)
        except Exception, e:
            traceback.print_exc()

def showImagesThread(obj):
    norm = Normalize(vmin=-2, vmax=1)
    pix = 1
    while(not rospy.is_shutdown()):
        try:
            worldmap = obj.worldmap.copy()
            for x,y in obj.route:
                worldmap[x-pix:x+pix,y-pix:y+pix] = -2
            cv2.imshow("both", matplotlib.cm.rainbow(
                np.concatenate(
                    (norm(worldmap), 
                    norm(obj.bvpMap)), axis=1)))
            cv2.waitKey(1)
        except Exception, e:
            traceback.print_exc()
    cv2.destroyWindow('both')

def calcBVPThread(obj):
    walls = None
    while(not rospy.is_shutdown()):
        try:
            walls = harmonicpotentialfield.mkBVPMap(obj.worldmap.copy(), walls=walls)
            obj.bvpMap = walls
            maps.publishMap(obj.bvpMap, "myfield", obj.cmd_field, obj.pos.x, obj.pos.y, obj.pos.rot)
        except Exception, e:
            traceback.print_exc()

def calcPathPlanThread(obj):
    time.sleep(1) # wait initial spin
    while(not rospy.is_shutdown()):
        route = []
        if hasattr(obj, "bvpMap"):
            try:
                route = harmonicpotentialfield.mkRoute(obj.bvpMap, (obj.pos.x, obj.pos.y), steps=50, stepSize=1)
            except Exception, e:
                traceback.print_exc()
        obj.route = route

class Explore():
    def __init__(self, worldmap = None):
        if worldmap is None:
            worldmap = np.zeros((400,400))
        self.linearResolution = 0.2
        self.worldmap = worldmap
        self.bvpMap = np.zeros(worldmap.shape)
        self.route = []
        self.localmap = np.zeros((10,10))
        self.pos = lambda:None
        self.pos.x = self.worldmap.shape[0]/2
        self.pos.y = self.worldmap.shape[1]/2
        self.pos.rot = 0
        rospy.init_node('Explore', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber('/scan', LaserScan, self.getScan)
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.cmd_map = rospy.Publisher('mymap', OccupancyGrid, queue_size=1)
        self.cmd_field = rospy.Publisher('myfield', OccupancyGrid, queue_size=1)

        threading.Thread(target=lambda:updateWorldMapThread(self)).start()
        threading.Thread(target=lambda:calcPathPlanThread(self)).start()
        threading.Thread(target=lambda:calcBVPThread(self)).start()
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
        rx,ry = self.route[0]
        dx, dy = rx-self.pos.x, ry-self.pos.y
        if np.abs(dx)+np.abs(dy)<8: # reached point
            self.route = self.route[1:]
            return self.calcSpin()
        rot = np.arctan2(-dx,dy) # cv2 weird axis
        rot = (rot+np.pi)%(2*np.pi)-np.pi
        spin = rot-self.pos.rot
        spin = (spin+np.pi)%(2*np.pi)-np.pi
        return spin * 0.3

    def calcVel(self, spin):
        if not hasattr(self, "scan"):
            return 0
        ranges = self.scan.ranges
        minrange = np.nanmin(ranges)
        if minrange < 0.4:
            print ("colision detected: ", minrange)
            self.route = []
            return 0
        # wait for goal to be ahead
        if abs(spin) < 0.1:
            return (1 - abs(spin))*0.8
        return 0

    def shutdown(self):
        self.worldmap[self.worldmap<0]=0
        filename = datetime.datetime.now().strftime("worldmap-%Y%m%d-%H%M.png")
        matplotlib.image.imsave(filename, self.worldmap,cmap='gray_r')

        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        Explore(*sys.argv[1:])
    except rospy.exceptions.ROSInterruptException:
        pass

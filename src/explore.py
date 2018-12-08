#!/usr/bin/env python

import sys
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
import cv2
import threading
import time
from matplotlib.colors import Normalize
import matplotlib.cm as cm
import datetime
import astar
import harmonicpotentialfield
import laserscan
import maps
import tf

def normRad(rad):
    return (rad + np.pi) % (2*np.pi) - np.pi

def updateWorldMapThread(obj):
    while(not rospy.is_shutdown()):
        if not hasattr(obj, "scan"):
            time.sleep(0.1)
            continue
        msg = obj.scan
        x, y, rot = obj.pos.x, obj.pos.y, obj.pos.rot
        localmap = laserscan.ranges2cart(msg.ranges, msg.range_min, msg.range_max, msg.angle_min, msg.angle_increment)
        # x0,y0,rot0 = x,y,rot
        # initialgrade = maps.trypos(obj.worldmap, localmap, x, y, rot)
        # # (x, y, rot), grade = maps.localrandom(obj.worldmap, localmap, x, y, rot)
        # (x, y, rot), grade = maps.localbest(obj.worldmap, localmap, x, y, rot, angle_increment=msg.angle_increment)
        # if grade/initialgrade > 0.1:
        #     obj.pos.x += (x-x0)/2  # /2 to smooth correction      
        #     obj.pos.y += (y-y0)/2       
        #     obj.pos.rot += (rot-rot0)/2
        obj.worldmap = maps.joinMaps(obj.worldmap, localmap, x, y, rot)
        obj.localmap = localmap
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "mymap"
        msg.info.resolution = 0.04
        msg.info.width = obj.worldmap.shape[0]
        msg.info.height = obj.worldmap.shape[1]
        msg.info.origin.orientation = Quaternion(0,0,0,1)
        msg.info.origin.position.x = -8
        msg.info.origin.position.y = -8
        msg.data = 100/(1+np.exp(-obj.worldmap))
        msg.data[obj.worldmap == 0]=-1
        msg.data = msg.data.T.astype(np.int8).ravel()
        try:
            obj.cmd_map.publish(msg)
        except Exception as e:
            print('error', e)
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "mymap",
                         "map")

def calcBVPThread(obj):
    walls = None
    while(not rospy.is_shutdown()):
        bvp = harmonicpotentialfield.mkBVPMap(obj.worldmap, walls=walls)
        # for i in range(4):
        #     route = harmonicpotentialfield.mkRoute(bvp, (obj.pos.x, obj.pos.y))
        #     if len(route)>1:
        #         x,y = route[-1]
        #         if obj.worldmap[x,y] == 0: # unexplored
        #             break
        #     bvp = harmonicpotentialfield.mkBVPMap(obj.worldmap,steps=400,walls=walls)
        obj.bvpMap = bvp
        walls = bvp

def calcPathPlanThread(obj):
    time.sleep(1) # wait initial spin
    while(not rospy.is_shutdown()):
        # # do not recalculate if there is a plan in action
        # if len(obj.route)>0: 
        #     time.sleep(0.1)
        #     obj.route = obj.route[1:] # fade path
        #     continue
        # print "making route"
        route = []
        if hasattr(obj, "bvpMap"):
            # print "using BVP"
            route = harmonicpotentialfield.mkRoute(obj.bvpMap, (obj.pos.x, obj.pos.y), steps=1, stepSize=5)
            # print "got %s points"%len(route)
        # # local minimum?
        # if len(route)<2:
        #     print "using A*"
        #     route = astar.mkRoute(obj.worldmap, (obj.pos.x, obj.pos.y), (0,0))
        obj.route = route

class Trab2():
    def __init__(self):
        self.linearResolution = 0.2
        self.worldmap = np.ndarray((400,400), float)
        self.bvpMap = np.ndarray((400,400), float)
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
        self.cmd_map = rospy.Publisher('mymap', OccupancyGrid, queue_size=1)

        threading.Thread(target=lambda:updateWorldMapThread(self)).start()
        threading.Thread(target=lambda:calcPathPlanThread(self)).start()
        threading.Thread(target=lambda:calcBVPThread(self)).start()
        # threading.Thread(target=lambda:showImagesThread(self)).start()

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
            return (1 - abs(spin))*0.5
        return 0

    def shutdown(self):
        filename = datetime.datetime.now().strftime("worldmap-%Y%m%d-%H%M.npy")
        np.save(filename, self.worldmap)
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        Trab2(*sys.argv[1:])
    except rospy.exceptions.ROSInterruptException:
        pass

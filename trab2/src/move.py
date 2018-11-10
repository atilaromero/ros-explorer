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
    imgsize = 200
    rangestep = 10.0/imgsize
    ranges = np.array(ranges)
    r, phi = np.mgrid[0:range_max:rangestep,-np.pi:np.pi:angle_increment]
    phimin_index = int((angle_min+np.pi)/angle_increment)
    ranges2d = np.zeros(r.shape)
    ranges2d[:,phimin_index:phimin_index+ranges.shape[0]]=ranges
    ranges2d[np.isnan(ranges2d)] = range_max+2
    v = np.zeros(r.shape)
    v[r>ranges2d] = 0
    v[r<ranges2d] = -1
    v[r<range_min] = 0
    v[(ranges2d!=0) & (np.abs(r-ranges2d)<0.1)] =1
    dst = cv2.linearPolar(v.T, (imgsize,imgsize),imgsize,cv2.WARP_FILL_OUTLIERS | cv2.WARP_INVERSE_MAP)
    dst2 = np.zeros((imgsize*2,imgsize*2))
    dst2[:,imgsize:] = dst[:imgsize*2,:][:,::-1] 
    return dst2

def showImage(obj):
    norm = Normalize(vmin=-1, vmax=1)
    cv2.namedWindow('showImage')
    while(not rospy.is_shutdown()):
        if not hasattr(obj, "scan"):
            time.sleep(0.1)
            continue
        msg = obj.scan
        image = ranges2cart(msg.ranges, msg.range_min, msg.range_max, msg.angle_min, msg.angle_increment)
        cv2.imshow("showImage", cm.hot(norm(image)))
        cv2.waitKey(1)
    cv2.destroyWindow('showImage')
        

class Trab2():
    def __init__(self):
        self.linearResolution = 0.2
        self.pos = lambda:None
        self.pos.x = 0
        self.pos.y = 0
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
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # wait for 0.1 seconds (10 HZ) and publish again
            self.move()
            r.sleep()

    def getScan(self, msg):
        self.scan = msg

    def move(self):
        spin = self.calcSpin()
        vel = self.calcVel(spin)
        self.pos.rot += spin
        self.pos.x += np.cos(self.pos.rot)*vel
        self.pos.y += np.sin(self.pos.rot)*vel
        move_cmd = Twist()
        move_cmd.angular.z = spin
        move_cmd.linear.x = vel
        self.cmd_vel.publish(move_cmd)

    def calcVel(self, spin):
        if abs(spin) < 1.0:
            return (1 - abs(spin))*0.7
        return 0

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

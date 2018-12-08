#!/usr/bin/env python

import sys
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import tf
import time
import laserscan
import maps

class WorldMap():
    def __init__(self):
        self.worldmap = np.ndarray((400,400), float)
        self.localmap = np.ndarray((10,10),float)
        self.pos = lambda:None
        self.pos.x = self.worldmap.shape[0]/2
        self.pos.y = self.worldmap.shape[1]/2
        self.pos.rot = 0
        rospy.init_node('Trab2', anonymous=False)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.getScan)
        self.scan_odom = rospy.Subscriber('/odom', Odometry, self.getOdom)
        self.cmd_map = rospy.Publisher('/mymap', OccupancyGrid, queue_size=1)
        self.updateMap()

    def getScan(self, msg):
        self.scan = msg

    def getOdom(self, msg):
        self.pos.x = msg.pose.pose.position.x
        self.pos.y = msg.pose.pose.position.y
        euler = tf.transformations.euler_from_quaternion(msg.pose.pose.orientation)
        self.pos.rot = euler[2]
    
    def updateMap(self):
        while(not rospy.is_shutdown()):
            if not hasattr(self, "scan"):
                time.sleep(0.1)
                continue
            msg = self.scan
            x, y, rot = self.pos.x, self.pos.y, self.pos.rot
            localmap = laserscan.ranges2cart(msg.ranges, msg.range_min, msg.range_max, msg.angle_min, msg.angle_increment)
            self.worldmap = maps.joinMaps(self.worldmap, localmap, x, y, rot)
            self.localmap = localmap
            self.cmd_map.publish()


if __name__ == '__main__':
    try:
        WorldMap(*sys.argv[1:])
    except rospy.exceptions.ROSInterruptException:
        pass

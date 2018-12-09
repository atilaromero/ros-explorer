#!/usr/bin/env python

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf

def normRad(rad):
    return (rad + np.pi) % (2*np.pi) - np.pi

class Trab1():
    def __init__(self, goalX, goalY):
        self.goalX = float(goalX)
        self.goalY = float(goalY)
        self.linearResolution = 0.2
        rospy.init_node('Trab1', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        def getPose(msg):
            if not hasattr(self, 'pose'):
                print msg.pose.pose
            self.pose = msg.pose.pose
        self.odom_sub = rospy.Subscriber('/odom', Odometry, getPose)
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

        # TurtleBot will stop if we don't keep telling it to move.
        # How often should we tell it to move? 10 HZ
        r = rospy.Rate(10);
        while not rospy.is_shutdown() and not self.reached(self.goalX, self.goalY):
            # wait for 0.1 seconds (10 HZ) and publish again
            self.move()
            r.sleep()

    def reached(self, x, y):
        if not hasattr(self, 'pose'):
            return False
        diffX = abs(self.pose.position.x-x)
        diffY = abs(self.pose.position.y-y)
        return (diffX < self.linearResolution) and (diffY < self.linearResolution)

    def move(self):
        if not hasattr(self, 'pose'):
            return
        spin = self.calcSpin(self.pose.orientation, self.pose.position.x, self.pose.position.y, self.goalX, self.goalY)
        move_cmd = Twist()
        move_cmd.angular.z = spin * 0.6
        move_cmd.linear.x = self.calcVel(spin)
        self.cmd_vel.publish(move_cmd)

    def calcVel(self, spin):
        if abs(spin) < 1.0:
            return (1 - abs(spin))*0.7
        return 0

    def calcSpin(self, origOrient, origX, origY, destX, destY):
        euler = tf.transformations.euler_from_quaternion((origOrient.x, origOrient.y, origOrient.z, origOrient.w))
        curRads = euler[2]
        x = destX - origX
        y = destY - origY
        destRads = normRad(np.arctan2(y,x))
        return normRad(destRads - curRads)

    def shutdown(self):
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        Trab1(*sys.argv[1:])
    except rospy.exceptions.ROSInterruptException:
        pass

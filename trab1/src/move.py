#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Trab1():
    def __init__(self, goalX, goalY):
        self.goalX = int(goalX)
        self.goalY = int(goalY)
        rospy.init_node('Trab1', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.getPose)
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        # TurtleBot will stop if we don't keep telling it to move.
        # How often should we tell it to move? 10 HZ
        r = rospy.Rate(10);
        while not rospy.is_shutdown() and not self.reached():
            # wait for 0.1 seconds (10 HZ) and publish again
            self.move()
            r.sleep()

    def reached(self):
        if not hasattr(self, 'pose'):
            return False
        resolution = 1.0
        diffX = abs(self.pose.position.x-self.goalX)
        diffY = abs(self.pose.position.y-self.goalY)
        print diffX, diffY
        return diffX < resolution and diffY < resolution
        # try:
        #     pass
        # except:
        #     return False

    def getPose(self, msg):
        # print msg.pose.pose
        self.pose = msg.pose.pose

    def move(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.1
        move_cmd.angular.z = 0
        self.cmd_vel.publish(move_cmd)

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

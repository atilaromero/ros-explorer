#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class Trab1():
    def __init__(self):
        rospy.init_node('Trab1', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        # TurtleBot will stop if we don't keep telling it to move.
        # How often should we tell it to move? 10 HZ
        r = rospy.Rate(10);
        while not rospy.is_shutdown():
            # wait for 0.1 seconds (10 HZ) and publish again
            self.move()
            r.sleep()

    def move(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.2
        move_cmd.angular.z = 0
        self.cmd_vel.publish(move_cmd)

    def shutdown(self):
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        Trab1()
    except:
        pass

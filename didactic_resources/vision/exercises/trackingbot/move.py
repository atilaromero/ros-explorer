#!/usr/bin/env python

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import tf
import cv2
import numpy as np
import cv_bridge
import matplotlib.pyplot as plt

def normRad(rad):
    return (rad + np.pi) % (2*np.pi) - np.pi

class Move():
    def __init__(self):
        self.linearResolution = 0.2
        self.bridge = cv_bridge.CvBridge()
        rospy.init_node('Move', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        def getPose(msg):
            self.pose = msg.pose.pose
        def getImage(msg):
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([40, 255, 255]))
            if False and not hasattr(self, 'image'):
                print img.shape
                plt.imshow(mask, cmap='gray')
                plt.show()
            self.image = mask
        self.odom_sub = rospy.Subscriber('/odom', Odometry, getPose)
        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, getImage)
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

        # TurtleBot will stop if we don't keep telling it to move.
        # How often should we tell it to move? 10 HZ
        r = rospy.Rate(10);
        while not rospy.is_shutdown():
            # wait for 0.1 seconds (10 HZ) and publish again
            self.move()
            r.sleep()

    def move(self):
        if not hasattr(self, 'pose'):
            return
        spin = self.calcSpin()
        move_cmd = Twist()
        move_cmd.angular.z = spin * 1.0
        move_cmd.linear.x = self.calcVel(spin)
        self.cmd_vel.publish(move_cmd)

    def calcVel(self, spin):
        # if abs(spin) < 1.0:
        #     return (1 - abs(spin))*0.7
        return 1.0

    def calcSpin(self):
        lin = (np.array(range(self.image.shape[1]))+1)
        for line in self.image[::-1]:
            if sum(line) == 0:
                continue
            q = np.average(lin, weights=line)
            if q > 0:
                rot = q/(len(line)/2) - 1
                print q, rot, max(line), len(line)
                return -rot

        # euler = tf.transformations.euler_from_quaternion((origOrient.x, origOrient.y, origOrient.z, origOrient.w))
        # curRads = euler[2]
        # x = destX - origX
        # y = destY - origY
        # destRads = normRad(np.arctan2(y,x))
        # return normRad(destRads - curRads)
        return 0.5

    def shutdown(self):
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        Move(*sys.argv[1:])
    except rospy.exceptions.ROSInterruptException:
        pass

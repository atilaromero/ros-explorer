import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
import tf

def publishMap(worldmap, topicName, publisher, x,y, rot):
    msg = OccupancyGrid()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = topicName
    msg.info.resolution = 0.05
    msg.info.width = worldmap.shape[0]
    msg.info.height = worldmap.shape[1]
    msg.info.origin.orientation = Quaternion(0,0,0,1)
    msg.info.origin.position.x = -x*msg.info.resolution
    msg.info.origin.position.y = -y*msg.info.resolution
    msg.data = 100/(1+np.exp(-worldmap))
    msg.data[worldmap == 0]=-1
    msg.data = msg.data.T.astype(np.int8).ravel()
    publisher.publish(msg)
    br = tf.TransformBroadcaster()
    br.sendTransform((0,0,0),
                    tf.transformations.quaternion_from_euler(0, 0, -rot-np.pi/2),
                    rospy.Time.now(),
                    topicName,
                    "base_link")

def joinMaps(worldmap, localmap, x, y, rot):
    rows,cols = localmap.shape
    M = cv2.getRotationMatrix2D((cols/2,rows/2),rot*180/np.pi,1)
    res = np.zeros((max([worldmap.shape[0],rows/2+x]),max(worldmap.shape[1],cols/2+y)))
    res[:worldmap.shape[0],:worldmap.shape[1]] = worldmap.copy()
    res[int(x-rows/2):,int(y-cols/2):][:rows,:cols] += cv2.warpAffine(localmap,M,(rows,cols))
    res = 0.9786 * np.sin(1.1 * res)
    if np.max(np.abs(res.flat))>1:
        print "error in joinMaps: ", np.max(np.abs(res.flat))
    return res

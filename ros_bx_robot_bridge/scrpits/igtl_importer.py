#!/usr/bin/env python

import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Transform
from std_msgs.msg import String
from ros_igtl_bridge.msg import igtlpoint
from ros_igtl_bridge.msg import igtlstring
from ros_igtl_bridge.msg import igtltransform

def cb_transform(data):
    pass

def cb_point(data):

    if data.name == 'Target':
        rospy.loginfo(rospy.get_caller_id() + " Target = (%f, %f, %f).", data.pointdata.x, data.pointdata.y, data.pointdata.z)
    elif data.name == 'Entry':
        rospy.loginfo(rospy.get_caller_id() + " Entry = (%f, %f, %f).", data.pointdata.x, data.pointdata.y, data.pointdata.z)

def cb_string(data):
    pass


def igtl_importer():

    global pub_igtl_transform_out
    
    pub_igtl_transform_out = rospy.Publisher('IGTL_TRANSFORM_OUT', igtltransform, queue_size=10)
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('igtl_importer', anonymous=True)
    
    rospy.Subscriber("IGTL_TRANSFORM_IN", igtltransform, cb_transform)
    rospy.Subscriber("IGTL_POINT_IN", igtlpoint, cb_point)
    rospy.Subscriber("IGTL_STRING_IN", igtlstring, cb_string)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    igtl_importer()




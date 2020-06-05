#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import math
import tf
import geometry_msgs.msg
from ros_igtl_bridge.msg import igtlstring
from ros_igtl_bridge.msg import igtltransform
from geometry_msgs.msg import Transform


def push_transform(pub, name, trans, rot):

    transform = Transform()
    transform.translation.x = trans[0] * 1000
    transform.translation.y = trans[1] * 1000
    transform.translation.z = trans[2] * 1000
    transform.rotation.x = rot[0]
    transform.rotation.y = rot[1]
    transform.rotation.z = rot[2]    
    transform.rotation.w = rot[3]
    
    transmsg = igtltransform()
    transmsg.name = name
    transmsg.transform = transform

    pub.publish(transmsg)
    

def igtl_exporter():

    base_link_name = 'base_link'

    link_name = [
        'shoulder_link',
        'upper_arm_link',
        'forearm_link', 
        'wrist_1_link', 
        'wrist_2_link', 
        'wrist_3_link', 
        'wrist_3_link', 
        'wrist_3_link',
        'tool0',
        'needle_holder',
        'needle'
    ]

    rospy.init_node('igtl_exporter', anonymous=True)

    pub_igtl_transform_out = rospy.Publisher('IGTL_TRANSFORM_OUT', igtltransform, queue_size=10)    

    listener = tf.TransformListener()
    rate = rospy.Rate(10) # 10hz

    n_link = len(link_name)

    trans = [[0]*3]*n_link
    rot = [[0]*4]*n_link
    
    while not rospy.is_shutdown():
        
        try:
            for i in range(n_link):
                (trans[i],rot[i]) = listener.lookupTransform(base_link_name, link_name[i], rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        for i in range(n_link):
            push_transform(pub_igtl_transform_out, link_name[i], trans[i], rot[i])

        #log_str = "igtl_exporter %s" % rospy.get_time()
        #rospy.loginfo(log_str)
        #pub.publish(log_str)
        rate.sleep()
        

if __name__ == '__main__':

    igtl_exporter()



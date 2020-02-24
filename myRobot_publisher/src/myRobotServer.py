#!/usr/bin/env python

import rospy
from threading import Thread
from sensor_msgs.msg import JointState
from mrm_publisher.msg import instructionsMessage, responseMessage
import math
import numpy as np


class myRobotServer:
    def __init__(self):
        self.setupJointState()
        self.setupPublishersAndSubscribers()
        self.msg = instructionsMessage()
        self.clearMsg()

    def setupJointState(self):
        self.joint_state = JointState()
        self.joint_state.name = ['joint1', 'joint2']
        self.joint_state.position = [0.0, 0.0]

    def setupPublishersAndSubscribers(self):
        rospy.Subscriber("chatter", instructionsMessage, self.callback)
        self.pub = rospy.Publisher('chatter', instructionsMessage, queue_size=1)
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=1)

    def writeResponseMessage(self, data):
        position = self.getJointStatePosition(data)
        #Write the message for valid positions
        if position[2] == 'Valid':
            joint1,joint2 = position[0],position[1]
            #Get the forward kinematics from the joint angles
            x,y,z = self.getForwardKinematics(joint1,joint2)
            self.joint_state.position = [joint1, joint2]
        #Write messages for invalid positions
        else:
            x,y,z = 0.0,0.0,0.0
            joint1,joint2 = self.joint_state.position[0],self.joint_state.position[1]
        self.msg.x = x
        self.msg.y = y
        self.msg.z = z
        self.msg.joint1 = joint1
        self.msg.joint2 = joint2
        self.msg.moveSuccessful = response[2]
        self.msg.finalPosition = "Position of Joint 1: " + str(
            self.joint_state.position[0]) + ", Position of Joint 2: " + str(self.joint_state.position[1])

    def setJointPosition(self):
        while not rospy.is_shutdown():
            self.updateJoints()
            self.pub.publish(self.msg)

    def callback(self, data):
        if data.side == 'Client':
            self.writeResponseMessage(data)

    def updateJoints(self):
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_pub.publish(self.joint_state)

    def getJointStatePosition(self, data):
        #If the movement type1 is selected the position of the joint is retrieve
        if data.movementType == 1:
            return data.joint1, data.joint2, 'Valid'

        #For movement type2 the angles for the joint are obtained using the inverse kinematics from the positions
        #of the data structure positions in the cartesian space (x, y, z).
        theta1, theta2 = self.getInverseKinematics(data.x, data.y, data.z)
        if self.isValidPosition(theta1, theta2, data):
            return theta1, theta2, 'Valid'
        return self.joint_state.position[0], self.joint_state.position[1], 'End Effector Position Not Valid. Try Again!!'

    #Check that the given values for theta1 and theta2 are real (inverse kinematics working fine). Np.isclose will
    #return True if the values are similar, in this case, the given value for the end effector and the joint positions
    def isValidPosition(self, theta1, theta2, data):
        x, y, z = self.getForwardKinematics(theta1, theta2)
        if np.isclose(x, data.x) and np.isclose(y, data.y) and np.isclose(z, data.z):
            return True
        return False

    def getInverseKinematics(self, x, y, z):
    #Given the positions in x,y,z axis calculate the values for the angles of both joints

        link1 = 1.0
        robotHeight = 0.6
        theta1 = 0.0
        #Calculate the value of the angle for the first joint based in its position in the x-y plane
        if x != 0.0 or y != 0.0:
            theta1 = math.atan2(x, y)
        theta2 = 0.0
        newHeight = z - robotHeight #Calculate the change of position in z
        #Calculate the position of the robot based in the change of the position of z and the actual link length.
        if newHeight <= 1 and newHeight >= -0.4 and newHeight != 0.0: #Avoid all the non valid positions from inspection
            theta2 = math.asin(newHeight / link1)
        return theta1, theta2

    def getForwardKinematics(self, theta1, theta2):
    #Given the angles of both joints calculate the position in x,y,z axis

        link1 = 1.0 #From robot information
        height = 0.6 #From the robot information
        z = link1 * math.sin(theta2) + height #z position
        x = 0.0
        y = 0.0
        if z != link1 + height: #z-position for the robot is valid
            x = link1 * math.sin(theta1) #x position
            y = link1 * math.cos(theta1) #y position
        return x, y, z

    def clearMsg(self):
    #CLear the previous messages
        self.msg.driveStyle = 0
        self.msg.x = 0
        self.msg.y = 0
        self.msg.z = 0
        self.msg.joint1 = 0
        self.msg.joint2 = 0
        self.msg.moveSuccessful = 'N/A'
        self.msg.finalPosition = 'N/A'
        self.msg.side = 'Server'

if __name__ == '__main__':
    try:
        rospy.init_node('RobotModelServer')
        rs = myRobotServer()
        rs.setJointPosition()
    except rospy.ROSInterruptException:
        pass

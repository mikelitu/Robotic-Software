#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import numpy as np
from mrm_publisher.msg import instructionsMessage, responseMessage


class myRobotClient:

    def __init__(self):
        rospy.init_node('robotModelClient', anonymous=True)
        self.pub = rospy.Publisher('chatter', instructionsMessage, queue_size=1)
        self.msg = instructionsMessage()

    def setMovement(self):
        while not rospy.is_shutdown():
            self.clearMsg()
            print('Rotate the joints or move the end effector position')
            print('Type 1: Rotate the joints | Type 2: Move end effector')
            movementType = input('Choose movement type: ')
            print("\n\n")
            self.msg.movementType = movementType
            if self.msg.movementType == 1:
                print('The valid range of angles for joint1 to joint2:')
                print('Joint1: [-pi, pi]rad and Joint2 [-pi, pi]rad')
                theta1 = input('Angle for Joint1 in radians: ')
                theta2 = input('Angle for Joint2 in radians: ')
                self.msg.joint1 = theta1
                self.msg.joint2 = theta2
            elif self.msg.movementType == 2:
                print("Valid Workspace for the end effector:")
                print("x: [-1, 1]; y: [-1, 1]; z: [-0.4, 1.6]")
                xEE = input("x position:")
                yEE = input("y position:")
                zEE = input("z position:")
                self.msg.x = xEE
                self.msg.y = yEE
                self.msg.z = zEE
            else:
                print("Movement Type does not exist.")
                print("Try Again!")
            self.pub.publish(self.msg)
            response = rospy.wait_for_message("chatter", instructionsMessage)
            rospy.loginfo(response)
            print("\n\n")

    def callback(self, data):
        rospy.loginfo(data)

    def clearMsg(self):
        self.msg.driveStyle = 0
        self.msg.x = 0
        self.msg.y = 0
        self.msg.z = 0
        self.msg.joint1 = 0
        self.msg.joint2 = 0
        self.msg.moveSuccessful = 'N/A'
        self.msg.finalPosition = 'N/A'
        self.msg.side = 'Client'


if __name__ == '__main__':
    try:
        rc = myRobotClient()
        rc.setMovement()
    except rospy.ROSInterruptException:
        pass

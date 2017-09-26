#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class GoForward():
    def __init__(self):
        rospy.init_node('forward', anonymous=False)

	rospy.loginfo("To stop robot CTRL + C")

        rospy.on_shutdown(self.shutdown)
        
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
     
        r = rospy.Rate(10);

        move_cmd = Twist()
        move_cmd.linear.x = 0.2
	move_cmd.angular.z = 0.2

        while not rospy.is_shutdown():
	    # publish the velocity
            self.cmd_vel.publish(move_cmd)
	    # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()
                        
        
    def shutdown(self):
        rospy.loginfo("Stop robot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        GoForward()
    except:
        rospy.loginfo("GoForward node terminated.")


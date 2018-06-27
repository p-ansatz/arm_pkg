#! /usr/bin/env python

import rospy
import actionlib

from arm_pkg.msg import *
from geometry_msgs.msg import Point

rospy.init_node('arm_tester_node')
client=actionlib.SimpleActionClient('move_arm', MoveArmAction)
client.wait_for_server()

goal = MoveArmGoal()
goal.p=Point()
goal.p.x = 0.10
goal.p.y = 0.2
goal.p.z = 0.10
goal.cmd = 4
client.send_goal(goal)
client.wait_for_result()

# goal2 = MoveArmGoal()
# goal2.cmd = 2
# client.send_goal(goal2)
# client.wait_for_result()
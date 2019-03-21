#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty

rospy.init_node('rovio_reset')

rospy.wait_for_service('/rovio/reset')
rovio_reset = rospy.ServiceProxy('/rovio/reset', Empty)

rovio_reset()

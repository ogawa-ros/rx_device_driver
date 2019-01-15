#!/usr/bin/env python3

import time
import rospy
import std_msgs.msg

import NASCORX_System.device.ML2437A as ML2437A

name = "ML2437A"

class ML2437(object):

    def __init__(self, port):
        self.num = rospy.get_param("~number")
        self.ip = rospy.get_param("~ip")
        self.dev = ML2437A.ml2437a(IP = self.ip, GPIB = port)

        self.pub_power = rospy.Publisher(
	        name = '/power_{}'.format(self.num),
		data_class = std_msgs.msg.Float64,
		latch = True,
		queue_size = 1
		)

        pass

    def pub_function(self):
        power = self.dev.measure()
        self.pub_power.publish(power)
        return

if __name__ == "__main__":
    rospy.init_node(name)
    m = ML2437(port = 13)

    while not rospy.is_shutdown():
        m.pub_function()
        time.sleep(0.001)
        continue

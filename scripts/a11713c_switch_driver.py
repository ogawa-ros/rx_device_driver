#!/usr/bin/env python3

import threading
import rospy
import std_msgs.msg

import NASCORX_System.device.A11713C as A11713C

name = "switch_8765a"

class A8765(object):

    def __init__(self, ip, port):
        self.pa = A11713C.a11713c(ip, port)
        self.ch_list = ['1X', '1Y', '2X', '2Y']
        self.ch = rospy.get_param('~ch')

        self.pub_level = rospy.Publisher(
            name = '/8765a_level_{}'.format(self.ch),
            data_class = std_msgs.msg.Int8,
            latch = True,
            queue_size = 1
        )

        self.sub_level = rospy.Subscriber(
            name = '/8765a_level_cmd_{}'.format(self.ch),
            data_class = std_msgs.msg.Int8,
            callback = self.callback,
            queue_size = 1,
        )

        self.pub_function()
        pass

    def callback(self, status):
        self.cmd = status.data

        self.pa.set_level(level = self.cmd, ch = self.ch)

        self.pub_function()
        return

    def pub_function(self):
        mode = self.pa.query_level()
        self.pub_level.publish(mode[self.ch_list.index(self.ch)])
        return


if __name__ == "__main__":
    rospy.init_node(name)
    switch = A8765(ip = '192.168.100.114', port = 5025)

    rospy.spin()

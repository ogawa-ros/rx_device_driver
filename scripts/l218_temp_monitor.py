#! /usr/bin/env python3

import sys
import time
import threading
import pymeasure

import rospy
import std_msgs
from std_msgs.msg import Float64


class lakeshore218_driver(object):

    def __init__(self, IP='', GPIB=1):
        self.IP = IP
        self.GPIB = GPIB
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)

    def measure(self, ch=1):
        self.com.open()
        self.com.send('KRDG? %d'%(ch))
        ret = self.com.readline()
        self.com.close()
        temperature = float(ret)

        return temperature

if __name__ == '__main__':
    node_name = 'lakeshore_218'
    host = rospy.get_param('~host')
    port = rospy.get_param('~port')
    rate = rospy.get_param('~rate')

    topic_name1 = rospy.get_param('~topic_name1')
    topic_name2 = rospy.get_param('~topic_name2')
    topic_name3 = rospy.get_param('~topic_name3')
    topic_name4 = rospy.get_param('~topic_name4')
    topic_name5 = rospy.get_param('~topic_name5')
    topic_name6 = rospy.get_param('~topic_name6')
    topic_name7 = rospy.get_param('~topic_name7')
    topic_name8 = rospy.get_param('~topic_name8')

    rospy.init_node(node_name)

    try:
        temperature = lakeshore2_driver(host, port)
    except OSError as e:
        rospy.logerr("{e.strerror}. host={host}".format(**locals()))
        sys.exit()

    pub1 = rospy.Publisher(topic_name1[0], Float64, queue_size=1)
    pub2 = rospy.Publisher(topic_name2[0], Float64, queue_size=1)
    pub3 = rospy.Publisher(topic_name3[0], Float64, queue_size=1)
    pub4 = rospy.Publisher(topic_name4[0], Float64, queue_size=1)
    pub5 = rospy.Publisher(topic_name5[0], Float64, queue_size=1)
    pub6 = rospy.Publisher(topic_name6[0], Float64, queue_size=1)
    pub7 = rospy.Publisher(topic_name7[0], Float64, queue_size=1)
    pub8 = rospy.Publisher(topic_name8[0], Float64, queue_size=1)

    while not rospy.is_shutdown():
        if topic_name1[1] == 0: pass
        elif topic_name1[1] == 1:
            msg1 = Float64()
            msg1.lakeshore_218_ch1 = temp.measure(1)
            pub.publish(msg1)
            return

        if topic_name2[1] == 0: pass
        elif topic_name2[1] == 1:
            msg2 = Float64()
            msg2.lakeshore_218_ch2 = temp.measure(2)
            pub.publish(msg2)
            return

        if topic_name3[1] == 0: pass
        elif topic_name3[1] == 1:
            msg3 = Float64()
            msg3.lakeshore_218_ch3 = temp.measure(3)
            pub.publish(msg3)
            return

        if topic_name4[1] == 0: pass
        elif topic_name4[1] == 1:
            msg4 = Float64()
            msg4.lakeshore_218_ch4 = temp.measure(4)
            pub.publish(msg4)
            return

        if topic_name5[1] == 0: pass
        elif topic_name5[1] == 1:
            msg5 = Float64()
            msg5.lakeshore_218_ch5 = temp.measure(5)
            pub.publish(msg5)
            return

        if topic_name6[1] == 0: pass
        elif topic_name1[1] == 1:
            msg6 = Float64()
            msg6.lakeshore_218_ch6 = temp.measure(6)
            pub6.publish(msg6)
            return

        if topic_name7[1] == 0: pass
        elif topic_name1[1] == 1:
            msg7 = Float64()
            msg7.lakeshore_218_ch1 = temp.measure(1)
            pub7.publish(msg7)
            return

        if topic_name8[1] == 0: pass
        elif topic_name8[1] == 1:
            msg8 = Float64()
            msg8.lakeshore_218_ch8 = temp.measure(8)
            pub.publish(msg8)
            return

        continue

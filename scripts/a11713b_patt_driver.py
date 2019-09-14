#! /usr/bin/env python3


import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64

import sys
import time
import pymeasure


node_name = 'a11713b'


class InvalidValueError(Exception):
    pass


class a11713b_driver(object):

    def __init__(self, IP='192.168.100.1', port=5025, connection='GPIB'):
        self.IP = IP
        self.port = port

        if connection == 'GPIB':
            self.com = pymeasure.gpib_prologix(self.IP, self.port)
            self.com.open()
            # time.sleep(0.5)

        elif connection == 'LAN':
            self.com = pymeasure.ethernet(self.IP, self.port)
            self.com.open()
            # time.sleep(0.5)
        return

    def set_level(self, level, ch):
        # self.com = pymeasure.gpib_prologix(self.IP, self.port)

        if 0 <= level <= 11 and type(level) == int:
            # self.com.open()

            if ch == '1X':
                self.com.send('ATTenuator:BANK1:X {}'.format(level))
                # time.sleep(0.5)

            elif ch == '1Y':
                self.com.send('ATTenuator:BANK1:Y {}'.format(level))
                # time.sleep(0.5)

        # self.com.close()
        else:
            msg = 'Available level: 0, 1, 2, ..., 11,'
            msg += ' while is {} given.'.format(level)
            raise InvalidValueError(msg)
        return

    def query_level(self):
        # time.sleep(0.5)
        self.com.send('ATTenuator:BANK1:X?')
        # time.sleep(0.5)
        ret1 = self.com.readline()
        # time.sleep(0.5)
        self.com.send('ATTenuator:BANK1:Y?')
        # time.sleep(0.5)
        ret2 = self.com.readline()
        # time.sleep(0.5)
        att1X = int(ret1)
        att1Y = int(ret2)
        level = [att1X, att1Y]

        return level


class a11713b_controller(object):

    def __init__(self):
        host = rospy.get_param('~host')
        port = rospy.get_param('~port')
        connection = rospy.get_param('~connection')
        self.driver_no = rospy.get_param('~driver_no')
        self.driver = a11713b_driver(host, port, connection)

        self.ch_list = ['1X', '1Y']

        self.pub_list = [
            rospy.Publisher(
                name = '{0}_{1}_{2}_level'.format(node_name, ch, self.driver_no),
                data_class = Int32,
                latch = False,
                queue_size = 1
            )
            for ch in self.ch_list
        ]

        self.sub_list = [
            rospy.Subscriber(
                name = '{0}_{1}_{2}_level_cmd'.format(node_name, ch, self.driver_no),
                data_class = Int32,
                callback = self.callback,
                callback_args = ch,
                # queue_size = 1
            )
            for ch in self.ch_list
        ]
        return

    def callback(self, q, ch):
        level = q.data
        self.driver.set_level(level=level, ch=ch)
        time.sleep(1.)
        current = self.driver.query_level()[self.ch_list.index(ch)]
        self.pub_list[self.ch_list.index(ch)].publish(current)
        return

if __name__ == '__main__':
    rospy.init_node(node_name)
    a11713b_controller()
    rospy.spin()

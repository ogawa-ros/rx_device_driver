#!/usr/bin/env python3


import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64

import sys
import time
import pymeasure


node_name = 'fsw0020'


class InvalidRangeError(Exception):
    pass


class fsw0020_driver(object):

    def __init__(self, IP='192.168.100.1', port=10001):
        self.IP = IP
        self.port = port
        self.com = pymeasure.ethernet(self.IP, self.port)
        self.sg = pymeasure.Phasematrix.FSW0020(self.com)

    def set_freq(self, freq):
        self.sg.freq_set(freq, 'GHz') # 0.5 - 20 [GHz]
        return

        def get_freq(self):
        ret = self.sg.freq_query()

        return float(ret) / 1e+12

    def set_power(self, power):
        if -10.0 <= power <= 13.0:
            self.sg.power_set(power, 'dBm')
        else:
            msg = 'Power range is -10.0[dBm] -- 13.0[dBm],'
            msg += ' while {}[dBm] is given.'.format(power)
            raise InvalidRangeError(msg)
        return

    def get_power(self):
        ret = self.sg.power_query()

        return float(ret)

    def set_onoff(self, onoff=0):
        if onoff==0:
            self.sg.output_set('OFF')
        elif onoff==1:
            self.sg.output_set('ON')
        return

    def get_output(self):
        ret = self.sg.output_query()
        switch = ret.startswith('OFF')
        if switch==True:
            onoff = 0
        else:
            onoff = 1

        return onoff


class fsw0020_controller(object):

    def __init__(self):
        host = rospy.get_param('~host')
        port = rospy.get_param('~port')
        self.sg_name = rospy.get_param('~sg_name')
        self.sg = fsw0020_driver(host, port)

        self.pub_freq = rospy.Publisher(
            name = '{}_freq'.format(self.sg_name),
            data_class = Float64,
            latch = True,
            queue_size = 1
            )
        self.pub_power = rospy.Publisher(
            name = '{}_power'.format(self.sg_name),
            data_class = Float64,
            latch = True,
            queue_size = 1
            )
        self.pub_freq = rospy.Publisher(
            name = '{}_onoff'.format(self.sg_name),
            data_class = Int32,
            latch = True,
            queue_size = 1
            )

        self.sub_freq = rospy.Subscriber(
            name = '{}_freq_cmd'.format(self.sg_name),
            data_class = Float64,
            callback = self.callback_freq,
            queue_size = 1
            )
        self.sub_power = rospy.Subscriber(
            name = '{}_power_cmd'.format(self.sg_name),
            data_class = Float64,
            callback = self.callback_power,
            queue_size = 1
            )
        self.sub_onoff = rospy.Subscriber(
            name = '{}_onoff_cmd'.format(self.sg_name),
            data_class = Float64,
            callback = self.callback_onoff,
            queue_size = 1
            )

    def callback_freq(self, q):
        target = q.data
        self.sg.set_freq(target)
        time.sleep(1)
        current = self.sg.get_freq()
        self.pub_freq.publish(current)
        return

    def callback_power(self, q):
        target = q.data
        self.sg.set_power(target)
        time.sleep(1)
        current = self.sg.get_power()
        self.pub_power.publish(current)
        return

    def callback_onoff(self, q):
        target = q.data
        self.sg.set_onoff(target)
        time.sleep(1)
        current = self.sg.get_onoff()
        self.pub_onoff.publish(current)
        return


if __name__ == '__main__':
    rospy.init_node(node_name)
    fsw0020_controller()
    rospy.spin()

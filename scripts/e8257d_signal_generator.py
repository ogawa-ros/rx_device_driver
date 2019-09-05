#! /usr/bin/env python3


import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64

import sys
import time
import pymeasure


node_name = 'e8257d'


class InvalidRangeError(Exception):
    pass


class e8257d_driver(object):

    def __init__(self, IP='', GPIB=1):
        self.IP = IP
        self.GPIB = GPIB
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        self.com.open()


    def set_freq(self, freq, unit='GHz'):
        self.com.send('FREQ:CW %.10f %s'%(freq, unit))
        return

    def get_freq(self):
        self.com.send('FREQ:CW?')
        ret = self.com.readline()

        return float(ret) / 1e+9

    def set_power(self, power=-20.0):
        if -20.0<=power<=30.0:
            self.com.send('POW %f dBm'%(power))
        else:
            msg = 'Power range is -20.0[dBm] -- 30.0[dBm],'
            msg += ' while {}[dBm] is given.'.format(power)
            raise InvalidRangeError(msg)
        return

    def get_power(self):
        self.com.send('POW?')
        ret = self.com.readline()

        return float(ret)

    def set_onoff(self, onoff=0):
        if onoff==1:
            self.com.send('OUTP ON')
        else:
            self.com.send('OUTP OFF')
        return

    def get_onoff(self):
        self.com.send('OUTP?')
        ret = self.com.readline()

        return int(ret)


class e8257d_controller(object):

    def __init__(self):
        host = rospy.get_param('~host')
        port = rospy.get_param('~port')
        self.sg_name = rospy.get_param('~sg_name')
        self.sg = e8257d_driver(host, port)

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
    e8257d_controller()
    rospy.spin()
